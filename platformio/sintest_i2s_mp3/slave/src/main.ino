/*
============================================================
 uzuCastSubLP — LP マスタ単一スケッチ（ESP32 Arduino Core 2.x）
 （旧 BTModule_44_48_LP + BTModule_44_48 を1ファイルに統合）
------------------------------------------------------------
 BTModule 44.1固定版（入力44.1/48対応）
 GAP探索 + 再接続フォールバック対応
============================================================

■ 概要
ESP32 を使用した Bluetooth A2DP 送信モジュールです。
保存済みデバイスへの再接続を基本とし、再接続に失敗した場合は
Classic Bluetooth GAP 探索へ自動でフォールバックして再接続を試みます。

本版は試作版 Final として、以下を実装しています。
・保存済みデバイスへの直接再接続
・再接続失敗時の自動GAP探索
・COD / RSSI による候補選別
・探索候補の中から BEST デバイスを選択して接続
・ペアリング情報の消去（ERASE）

------------------------------------------------------------

■ 操作方法

【短押し】
・IDLE 中
    - 保存済みデバイスがある場合：
        → 再接続開始（青点滅）
    - 保存済みデバイスがない場合：
        → ペアリング開始（赤点滅）

・保存済みデバイスへの再接続中（青点滅）
    → 強制ペアリングモードへ移行（赤点滅）

・ペアリング中（赤点滅）
    → 接続／探索処理を中止して IDLE へ戻る

------------------------------------------------------------

【長押し】
・どの状態からでも現在の処理を中止
・待機状態（IDLE）へ戻る
・赤＋青を短時間点灯した後、消灯

------------------------------------------------------------

【超長押し】
・ペアリング情報消去（ERASE）
・保存済み MAC アドレスおよび bond 情報を削除
・ERASE 完了後、自動的に IDLE へ戻る

------------------------------------------------------------

■ LED 表示

・IDLE
    - 消灯

・ペアリング中（探索中）
    - 赤高速点滅

・ペアリング中（接続試行中）
    - 赤低速点滅

・保存済みデバイスへの再接続中
    - 青低速点滅

・接続中
    - 青点灯

・TIMEOUT / ERROR
    - 赤点灯

・ERASE
    - 赤点灯

------------------------------------------------------------

■ 再接続仕様

1. 保存済みデバイスがある場合は、その MAC アドレスへ直接再接続します。
2. 直接再接続が失敗した場合は、自動で GAP 探索へ移行します。
3. フォールバック探索では保存済み MAC を優先して探索します。
4. 一致デバイスが見つかれば、そのデバイスへ接続します。

------------------------------------------------------------

■ 探索アルゴリズム

・Classic Bluetooth GAP Discovery を使用
・探索候補条件
    - COD Major Class = 0x04（Audio/Video）
    - COD bit21 = 1
    - RSSI > -70 dBm

・同一 MAC アドレスは重複除去
・候補の中で RSSI 最大のデバイスを BEST として選択

------------------------------------------------------------

■ 注意事項

・Bluetooth スピーカーがマルチポイント対応の場合、
  他の機器に接続が切り替わる可能性があります。

・確実な接続が必要な場合は、
  接続対象以外の Bluetooth 機器の電源を切る運用を推奨します。

============================================================
*/

////////////////////////////////////////////////////////////////////////////////
// Arduino auto-prototype workaround: keep enums near top
////////////////////////////////////////////////////////////////////////////////
enum BtState {
  ST_IDLE = 0,
  ST_PAIRING,
  ST_CONNECTING,
  ST_RECONNECT_WAIT,
  ST_RECONNECT_SCAN,
  ST_CONNECT,
  ST_ERASE,
  ST_ERROR,
  ST_TIMEOUT
};

enum PairingSubState {
  PSS_SCAN = 0,
  PSS_CONNECT_NEW
};

enum UzcSlotFmt {
  UZC_SLOT_FMT_NONE = 0,
  UZC_SLOT_FMT_V2_META,
  UZC_SLOT_FMT_V1_REAL_FIRST
};

enum UzcWireRxState {
  UZC_WIRE_HUNT_SYNC = 0,
  UZC_WIRE_LOCKED,
};

enum UzcPeriodPhase {
  UZC_PHASE_SLOT = 0,
  UZC_PHASE_PAD,
};

#include <Arduino.h>
#include <Preferences.h>
#include "BluetoothA2DPSource.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include <esp_arduino_version.h>
#include <cstdarg>
#include <cstdio>
#include "uzu_i2c_config.h"
#if UZU_ENABLE_I2C
#include "i2c_protocol.h"
#include "i2c_slave.h"
#include "i2c_slave_port.h"
#endif

#if ESP_ARDUINO_VERSION_MAJOR != 2 && ESP_ARDUINO_VERSION_MAJOR != 3
  #error "This project requires ESP32 Arduino Core 2.x or 3.x."
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include "esp_bt.h"
#if ESP_ARDUINO_VERSION_MAJOR >= 3 && __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3 && __has_include("esp32-hal-bt.h")
#include "esp32-hal-bt.h"
#endif

// ----- 低消費電力（CPU 80MHz は音切れ時 false に）-----
static constexpr bool LP_ENABLE_CPU_80MHZ = false;

static void applyLowPowerPresetBeforeBtInit() {
  WiFi.mode(WIFI_OFF);
  (void)esp_wifi_stop();
#if ESP_ARDUINO_VERSION_MAJOR < 3
  // Core 3.x: releasing BLE heap breaks Classic BT / GAP (INVALID_STATE on all calls).
  (void)esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
#endif
  if (LP_ENABLE_CPU_80MHZ) {
    setCpuFrequencyMhz(80);
  }
}

// ============================================================
// Debug
// ============================================================
#define DEBUG_BT_MODULE 1

#if DEBUG_BT_MODULE
static void DBG(const char* fmt, ...) {
  Serial.print("[DBG] ");
  va_list args;
  va_start(args, fmt);
  char buf[384];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}
#else
#define DBG(...) ((void)0)
#endif

// 不要なコードを削除: 48kHz->44.1kHz の判別/リサンプルは Main 側で完結
#ifndef BUILD_NUMBER
#define BUILD_NUMBER 89
#endif

// 1=BT未接続でもI2Sタグ処理を有効化（Master-Slave I2Sベンチテスト用）
#ifndef UZU_I2S_TEST_BYPASS_BT
#define UZU_I2S_TEST_BYPASS_BT 0
#endif

#define VERSION_STRING  "sintest_i2s_mp3 Stage2 I2S-441-R-TAG-MP3 (from uzuCastV3_Sub build 73)"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_STATIC_SCRATCH
#include "minimp3.h"
#include "uzc_mp3_pcm_ref.h"

// ============================================================
// GPIO
// ============================================================
static const int PIN_BUTTON   = 4;
static const int PIN_LED_BLUE = 25;
static const int PIN_LED_RED  = 26;
static const uint16_t UZC_MAX_FRAME_SIZE_FIELD_OFF = 0;
static const uint16_t UZC_REAL_FRAME_SIZE_FIELD_OFF = 2;
static const uint16_t UZC_FRAME_DATA_OFF = 4;
static const uint16_t UZC_SLOT_META_BYTES = 4;
static const uint16_t UZC_SLOT_BUF_MAX = 1024;
static const uint16_t UZC_MAX_MP3_FRAME = 896;
static const size_t UZC_SYNC_SLIDE_MAX = 32;

// ============================================================
// Device identity / BT visibility
// ============================================================
static const int DEVICE_CHANNEL_NO = 1;   // change per board: 1,2,3...
static const char* DEVICE_NAME_PREFIX = "UZU";
static char g_local_bt_name[32] = {0};

static inline bool uzcUseRightChannel() {
  return (DEVICE_CHANNEL_NO == 2);
}

static inline void setBlueLed(bool on) {
  digitalWrite(PIN_LED_BLUE, on ? HIGH : LOW);
}

static inline void setRedLed(bool on) {
  digitalWrite(PIN_LED_RED, on ? HIGH : LOW);
}

static inline void setAllLedOff() {
  setBlueLed(false);
  setRedLed(false);
}

static inline bool isButtonPressed() {
  return (digitalRead(PIN_BUTTON) == LOW);
}

// I2S R-ch 制御タグ (Master / FPGA 制御 ch と共通)
static const int16_t I2S_TAG_RAW        = (int16_t)0xAAAA;
static const int16_t I2S_TAG_MP3_START  = (int16_t)0xAA00;
static const int16_t I2S_TAG_MP3_END    = (int16_t)0x5500;
static const int16_t I2S_TAG_PAD        = (int16_t)0xFFFF;
static const int16_t I2S_TAG_SLOT_DATA  = (int16_t)0xAA55;
static const size_t UZC_MP3_PERIOD_STEREO_441 = 1152;

static bool g_i2s_tag_mp3_open = false;
static uint16_t g_i2s_tag_data_words = 0;
static uint32_t g_i2s_tag_pad_skip = 0;
static uint32_t g_tag_stat_slots_rx = 0;

#if UZU_ENABLE_I2C
static volatile uint8_t g_i2c_slave_output_mode = I2C_MODE_WAV_PCM;
static volatile bool g_i2c_bt_pending = false;
static int16_t g_i2c_delay_ms = 0;

static constexpr uint8_t I2C_IGNORE_MAX = 4;
static constexpr uint32_t I2C_IGNORE_MS = 30000;
struct I2cIgnoredMac {
  esp_bd_addr_t mac;
  uint32_t until_ms;
  bool active;
};
static I2cIgnoredMac g_i2c_ignored[I2C_IGNORE_MAX];
#endif

static esp_bd_addr_t g_connected_peer_mac = {0};
static bool g_connected_peer_valid = false;
static uint32_t g_tag_stat_decode_ok = 0;
static uint32_t g_tag_stat_word_err = 0;
static uint32_t g_tag_stat_align_err = 0;
static uint32_t g_tag_stat_hdr_reject = 0;
static int16_t g_i2s_dbg_last_l = 0;
static uint16_t g_i2s_dbg_last_r = 0;

static inline uint16_t i2sRxTagValue(int16_t r) {
  return (uint16_t)r;
}

// ============================================================
// I2S RX (external master -> ESP32 slave)
// ============================================================
static const int RX_BCK = 32;
static const int RX_WS  = 33;
static const int RX_DIN = 35;

static bool i2s_rx_init_slave_stereo_16() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  // In slave mode the external master drives WS/BCK, so this is only a nominal value.
  // Slave: BCK/WS は Master に従う。WAV=44100Hz / UZC=bitRate/8 Hz（例 12000Hz）。
  cfg.sample_rate = 12000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  // BT Classic と DRAM を共有するため控えめだが、loop ブロック時の DMA 欠落を抑える
  cfg.dma_buf_count = 16;
  cfg.dma_buf_len   = 256;
  cfg.use_apll      = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = RX_BCK;
  pins.ws_io_num    = RX_WS;
  pins.data_in_num  = RX_DIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;

  esp_err_t e1 = i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr);
  if (e1 != ESP_OK) {
    DBG("i2s_driver_install failed: %d", (int)e1);
    return false;
  }

  esp_err_t e2 = i2s_set_pin(I2S_NUM_1, &pins);
  if (e2 != ESP_OK) {
    DBG("i2s_set_pin failed: %d", (int)e2);
    return false;
  }

  return true;
}

static void i2s_rx_apply_nominal_rate(uint32_t hz) {
  esp_err_t e = i2s_set_clk(I2S_NUM_1, hz, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  DBG("i2s_set_clk %lu Hz -> %d", (unsigned long)hz, (int)e);
}

static void makeLocalBtName(char *out, size_t outSize) {
  const uint8_t *mac = esp_bt_dev_get_address();
  if (mac) {
    snprintf(out, outSize, "%s-CH%d-%02X%02X",
             DEVICE_NAME_PREFIX, DEVICE_CHANNEL_NO, mac[4], mac[5]);
  } else {
    uint64_t chip = ESP.getEfuseMac();
    snprintf(out, outSize, "%s-CH%d-%02X%02X",
             DEVICE_NAME_PREFIX, DEVICE_CHANNEL_NO,
             (unsigned)((chip >> 8) & 0xFF),
             (unsigned)(chip & 0xFF));
  }
}

static void setDiscoverableConnectable(bool enable) {
  esp_err_t err;
  if (enable) {
    err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  } else {
    err = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  }
  DBG("scan_mode -> %s (%d)", enable ? "DISCOVERABLE" : "HIDDEN", (int)err);
}

// ============================================================
// A2DP / NVS
// ============================================================
static Preferences prefs;

static const char* NVS_NS  = "a2dp";
static const char* KEY_MAC = "lastmac";

static volatile esp_a2d_connection_state_t g_conn_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
static volatile esp_a2d_audio_state_t      g_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
static volatile bool g_audio_enable = false;
static volatile uint32_t g_i2s_input_rate = 44100;
static volatile uint32_t g_a2dp_output_rate = 44100;
static volatile uint32_t g_a2dp_negotiated_rate = 44100;
static volatile bool g_media_start_pending = false;
static volatile uint32_t g_media_start_due_ms = 0;

// Arduino-ESP32 packages differ in how esp_a2d_mcc_t is exposed.
// Some versions expose parsed SBC fields (cie.sbc_info.*), others only raw bytes (cie.sbc[]).
// To keep this sketch portable, parse the SBC capability/config IE directly from the raw bytes.
static constexpr uint8_t SBC_SF_16K = 0x80;
static constexpr uint8_t SBC_SF_32K = 0x40;
static constexpr uint8_t SBC_SF_44K = 0x20;
static constexpr uint8_t SBC_SF_48K = 0x10;
static constexpr uint8_t SBC_CH_MONO = 0x08;
static constexpr uint8_t SBC_CH_DUAL = 0x04;
static constexpr uint8_t SBC_CH_STEREO = 0x02;
static constexpr uint8_t SBC_CH_JOINT = 0x01;
static constexpr uint8_t SBC_BLOCK_16 = 0x10;
static constexpr uint8_t SBC_SUBBANDS_8 = 0x04;
static constexpr uint8_t SBC_ALLOC_LOUDNESS = 0x01;

static uint32_t decodeSbcSampleRate(const esp_a2d_mcc_t &mcc) {
  if (mcc.type != ESP_A2D_MCT_SBC) return 44100;
  const uint8_t b0 = mcc.cie.sbc[0];
  if (b0 & SBC_SF_48K) return 48000;
  if (b0 & SBC_SF_44K) return 44100;
  if (b0 & SBC_SF_32K) return 32000;
  if (b0 & SBC_SF_16K) return 16000;
  return 44100;
}

// Kept for future use on newer cores that expose sink codec-capability callbacks.
static bool sinkCapsSupport48k(const esp_a2d_mcc_t &mcc) {
  return (mcc.type == ESP_A2D_MCT_SBC) && ((mcc.cie.sbc[0] & SBC_SF_48K) != 0);
}

static bool sinkCapsSupport44k(const esp_a2d_mcc_t &mcc) {
  return (mcc.type == ESP_A2D_MCT_SBC) && ((mcc.cie.sbc[0] & SBC_SF_44K) != 0);
}

static esp_a2d_mcc_t buildPreferredMcc(uint32_t rate_hz) {
  esp_a2d_mcc_t mcc = {};
  mcc.type = ESP_A2D_MCT_SBC;
  mcc.cie.sbc[0] = (rate_hz >= 47000 ? SBC_SF_48K : SBC_SF_44K) | (SBC_CH_JOINT | SBC_CH_STEREO);
  mcc.cie.sbc[1] = SBC_BLOCK_16 | SBC_SUBBANDS_8 | SBC_ALLOC_LOUDNESS;
  mcc.cie.sbc[2] = 2;   // min_bitpool
  mcc.cie.sbc[3] = 53;  // max_bitpool
  return mcc;
}

// Note:
// This project is intentionally fixed to 44.1 kHz on the Bluetooth/A2DP side.
// The ESP32-A2DP source library documents PCM for the source side as normally
// formatted as 44.1 kHz, stereo, 16-bit. Therefore we always feed 44.1 kHz PCM
// to the library, and Main 側で 48kHz を 44.1kHz にリサンプル済みのため
// Sub 側ではリサンプルしない。
// We still log the negotiated rate reported by ESP_A2D_AUDIO_CFG_EVT when
// available, but playback processing stays fixed at 44.1 kHz.
static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

class UzuBluetoothA2DPSource : public BluetoothA2DPSource {
protected:
  void app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) override {
    if (event == ESP_A2D_AUDIO_CFG_EVT) {
      uint32_t rate = decodeSbcSampleRate(param->audio_cfg.mcc);
      g_a2dp_negotiated_rate = rate;
      DBG("A2DP negotiated rate = %lu Hz (BT output forced to 44100 Hz path)", (unsigned long)rate);
    }
    BluetoothA2DPSource::app_a2d_callback(event, param);
  }

  void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) override {
    gapCallback(event, param);
    BluetoothA2DPSource::app_gap_callback(event, param);
  }
};

static UzuBluetoothA2DPSource a2dp;
static volatile BtState g_state = ST_IDLE;

// ============================================================
// I2S/MP3 入力ジッタ（hold + ring。A2DP audio_cb が hold から直接取得）
// ============================================================
static portMUX_TYPE g_rb_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint32_t PCM_HOLD_FRAMES = 4096;
// A2DP audio_cb は ~512frames/回のバースト → MP3 は 2 スロット分溜めてから開始
static constexpr uint32_t PCM_PREBUFFER_FRAMES = 2304;  // 2 × 1152 @44.1kHz
static int16_t g_pcm_hold[PCM_HOLD_FRAMES * 2];
static uint32_t g_pcm_hold_count = 0;
static uint32_t g_pcm_hold_rd = 0;

static constexpr uint32_t RB_FRAMES = 3072;
static int16_t g_rb[RB_FRAMES * 2];
static volatile uint32_t g_rb_w = 0;
static volatile uint32_t g_rb_r = 0;

static bool g_pcm_playback_active = false;
static bool g_pcm_input_armed = false;
static uint32_t g_pcm_underrun_frames = 0;
static uint32_t g_pcm_push_drop_frames = 0;
static bool g_media_ctrl_started = false;
static uint32_t g_pcm_service_drain_ms = 0;

#if UZU_I2S_TEST_BYPASS_BT
static bool g_i2s_bench_playback = false;
static uint32_t g_i2s_bench_drain_ms = 0;
static int16_t g_i2s_bench_peak = 0;
static uint32_t g_i2s_bench_pcm_out = 0;
#endif

static inline uint32_t rb_used_nolock(uint32_t w, uint32_t r) {
  return (w >= r) ? (w - r) : (RB_FRAMES - (r - w));
}

static inline uint32_t rb_free_nolock(uint32_t used) {
  return (RB_FRAMES - 1) - used;
}

static void pushPcmFramesToRing(const int16_t* stereoFrames, uint32_t frameCount);
static void startMedia();

static inline uint32_t pcmHoldUsedNolock() {
  return (g_pcm_hold_count > g_pcm_hold_rd) ? (g_pcm_hold_count - g_pcm_hold_rd) : 0U;
}

static uint32_t pcmHoldFreeFrames() {
  portENTER_CRITICAL(&g_rb_mux);
  const uint32_t holdUsed = pcmHoldUsedNolock();
  const uint32_t holdFree =
      (holdUsed < PCM_HOLD_FRAMES) ? (PCM_HOLD_FRAMES - holdUsed) : 0U;
  portEXIT_CRITICAL(&g_rb_mux);
  return holdFree;
}

static void waitForPcmPipelineSpace(uint32_t needFrames) {
  uint32_t spins = 0;
  while (needFrames > 0) {
#if UZU_I2S_TEST_BYPASS_BT
    if (g_i2s_bench_playback) {
      return;
    }
#endif
    if (g_state != ST_CONNECT || !g_audio_enable) {
      return;
    }
    if (pcmHoldFreeFrames() >= needFrames) {
      return;
    }
    vTaskDelay(1);
    if (++spins > 50U) {
      return;
    }
  }
}

static void tryStartMediaWhenBuffered() {
  if (g_media_ctrl_started || g_state != ST_CONNECT || !g_audio_enable) {
    return;
  }
  g_media_ctrl_started = true;
  startMedia();
}

static void pcmPlaybackBegin() {
  g_pcm_playback_active = true;
  g_pcm_underrun_frames = 0;
  g_pcm_push_drop_frames = 0;
  g_pcm_service_drain_ms = 0;
  tryStartMediaWhenBuffered();
}

// I2S デコード → hold（小さめジッタ）→ ring → A2DP audio_cb
static void servicePcmHoldToRing() {
  if (!g_pcm_playback_active || g_i2s_bench_playback) {
    return;
  }
  if (g_state != ST_CONNECT || !g_audio_enable) {
    return;
  }
  const uint32_t now = millis();
  if (g_pcm_service_drain_ms == 0) {
    g_pcm_service_drain_ms = now;
    return;
  }
  const uint32_t dt = now - g_pcm_service_drain_ms;
  if (dt < 5U) {
    return;
  }
  g_pcm_service_drain_ms = now;
  uint32_t want = (44100UL * dt) / 1000UL;
  if (want == 0U) {
    want = 1U;
  }

  portENTER_CRITICAL(&g_rb_mux);
  while (want > 0U && g_pcm_hold_rd < g_pcm_hold_count) {
    const uint32_t w = g_rb_w;
    const uint32_t r = g_rb_r;
    const uint32_t used = rb_used_nolock(w, r);
    const uint32_t freef = rb_free_nolock(used);
    if (freef == 0U) {
      break;
    }
    const uint32_t hidx = g_pcm_hold_rd * 2;
    const uint32_t rbidx = (w % RB_FRAMES) * 2;
    g_rb[rbidx + 0] = g_pcm_hold[hidx + 0];
    g_rb[rbidx + 1] = g_pcm_hold[hidx + 1];
    g_rb_w = w + 1U;
    g_pcm_hold_rd++;
    want--;
  }
  if (g_pcm_hold_rd >= g_pcm_hold_count) {
    g_pcm_hold_count = 0;
    g_pcm_hold_rd = 0;
  }
  portEXIT_CRITICAL(&g_rb_mux);
}

static bool pcmBufferReadyForPlayback(uint32_t holdUsed) {
  if (!g_pcm_input_armed || holdUsed < PCM_PREBUFFER_FRAMES) {
    return false;
  }
  if (g_tag_stat_decode_ok > 0) {
    return true;
  }
  return g_tag_stat_slots_rx > 0;
}

static bool pcmReadyForMediaStart(uint32_t holdUsed) {
  if (g_state != ST_CONNECT || !g_audio_enable) {
    return false;
  }
  return pcmBufferReadyForPlayback(holdUsed);
}

#if UZU_I2S_TEST_BYPASS_BT
static void i2sBenchBeginPlayback(uint32_t holdUsed) {
  if (g_i2s_bench_playback) {
    return;
  }
  g_i2s_bench_playback = true;
  g_pcm_playback_active = true;
  g_pcm_underrun_frames = 0;
  g_pcm_push_drop_frames = 0;
  g_i2s_bench_peak = 0;
  g_i2s_bench_pcm_out = 0;
  g_i2s_bench_drain_ms = millis();
  Serial.printf("[I2S-BENCH] play start hold=%lu decodeOk=%lu rx=%lu\n",
                (unsigned long)holdUsed,
                (unsigned long)g_tag_stat_decode_ok,
                (unsigned long)g_tag_stat_slots_rx);
}

static void i2sBenchDrainPcm() {
  if (!g_i2s_bench_playback) {
    return;
  }
  const uint32_t now = millis();
  if (g_i2s_bench_drain_ms == 0) {
    g_i2s_bench_drain_ms = now;
    return;
  }
  const uint32_t dt = now - g_i2s_bench_drain_ms;
  if (dt < 5U) {
    return;
  }
  g_i2s_bench_drain_ms = now;
  uint32_t want = (44100UL * dt) / 1000UL;
  if (want == 0U) {
    want = 1U;
  }
  portENTER_CRITICAL(&g_rb_mux);
  while (want > 0U && g_pcm_hold_rd < g_pcm_hold_count) {
    const uint32_t idx = g_pcm_hold_rd * 2;
    const int16_t s = g_pcm_hold[idx + 0];
    const int16_t a = (s < 0) ? (int16_t)(-s) : s;
    if (a > g_i2s_bench_peak) {
      g_i2s_bench_peak = a;
    }
    g_i2s_bench_pcm_out++;
    g_pcm_hold_rd++;
    want--;
  }
  if (g_pcm_hold_rd >= g_pcm_hold_count) {
    g_pcm_hold_count = 0;
    g_pcm_hold_rd = 0;
  }
  if (want > 0U) {
    g_pcm_underrun_frames += want;
  }
  portEXIT_CRITICAL(&g_rb_mux);
}

static int16_t pcmHoldPeakAbs(uint32_t maxScan) {
  int16_t peak = 0;
  portENTER_CRITICAL(&g_rb_mux);
  uint32_t n = pcmHoldUsedNolock();
  if (n > maxScan) {
    n = maxScan;
  }
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t idx = (g_pcm_hold_rd + i) * 2;
    const int16_t s = g_pcm_hold[idx + 0];
    const int16_t a = (s < 0) ? (int16_t)(-s) : s;
    if (a > peak) {
      peak = a;
    }
  }
  portEXIT_CRITICAL(&g_rb_mux);
  return peak;
}
#endif

static void tryBeginPlaybackWhenReady(uint32_t holdUsed) {
  if (!pcmBufferReadyForPlayback(holdUsed) || g_pcm_playback_active) {
    return;
  }
#if UZU_I2S_TEST_BYPASS_BT
  // ベンチは IDLE のみ。CONNECTING/PAIRING 中は PCM を hold に溜めて BT 接続後に再生する
  if (g_state == ST_IDLE && !g_audio_enable) {
    i2sBenchBeginPlayback(holdUsed);
    return;
  }
  if (g_state != ST_CONNECT || !g_audio_enable) {
    return;
  }
#endif
  if (g_audio_state != ESP_A2D_AUDIO_STATE_STARTED) {
    return;
  }
  pcmPlaybackBegin();
}

static void pcmHoldReset() {
  portENTER_CRITICAL(&g_rb_mux);
  g_pcm_hold_count = 0;
  g_pcm_hold_rd = 0;
  portEXIT_CRITICAL(&g_rb_mux);
  g_pcm_playback_active = false;
  g_pcm_input_armed = false;
  g_pcm_service_drain_ms = 0;
#if UZU_I2S_TEST_BYPASS_BT
  g_i2s_bench_playback = false;
  g_i2s_bench_drain_ms = 0;
  g_i2s_bench_peak = 0;
  g_i2s_bench_pcm_out = 0;
#endif
}

static void pcmHoldAppend(const int16_t* stereoFrames, uint32_t frameCount) {
  if (frameCount == 0 || !g_pcm_input_armed) {
    return;
  }

  // BT 再生開始前はプリバッファ上限まで（満杯→audio_cb 未稼働→溢れ）
  if (!g_pcm_playback_active && frameCount > 1U) {
    portENTER_CRITICAL(&g_rb_mux);
    const bool prebufFull = (pcmHoldUsedNolock() >= PCM_PREBUFFER_FRAMES);
    portEXIT_CRITICAL(&g_rb_mux);
    if (prebufFull) {
      return;
    }
  }

  uint32_t holdUsed = 0;
  portENTER_CRITICAL(&g_rb_mux);
#if UZU_I2S_TEST_BYPASS_BT
  if (g_i2s_bench_playback && frameCount > 1U) {
    while (pcmHoldUsedNolock() > 0U) {
      const uint32_t holdFreeNow = PCM_HOLD_FRAMES - pcmHoldUsedNolock();
      if (holdFreeNow >= frameCount) {
        break;
      }
      g_pcm_hold_rd++;
      if (g_pcm_hold_rd >= g_pcm_hold_count) {
        g_pcm_hold_count = 0;
        g_pcm_hold_rd = 0;
      }
    }
  }
#endif
  if (g_pcm_hold_rd > 0) {
    const uint32_t remain = g_pcm_hold_count - g_pcm_hold_rd;
    if (remain > 0) {
      memmove(g_pcm_hold, g_pcm_hold + g_pcm_hold_rd * 2, remain * 2 * sizeof(int16_t));
    }
    g_pcm_hold_count = remain;
    g_pcm_hold_rd = 0;
  }

  uint32_t n = frameCount;
  if (g_pcm_hold_count + n > PCM_HOLD_FRAMES) {
    g_pcm_push_drop_frames += frameCount - (PCM_HOLD_FRAMES - g_pcm_hold_count);
    n = PCM_HOLD_FRAMES - g_pcm_hold_count;
  }
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t idx = (g_pcm_hold_count + i) * 2;
    g_pcm_hold[idx + 0] = stereoFrames[i * 2 + 0];
    g_pcm_hold[idx + 1] = stereoFrames[i * 2 + 1];
  }
  g_pcm_hold_count += n;
  holdUsed = pcmHoldUsedNolock();
  portEXIT_CRITICAL(&g_rb_mux);

  tryBeginPlaybackWhenReady(holdUsed);
}

static void resetAudioPipeline(bool clearRateWindow) {
  (void)clearRateWindow;
  g_media_ctrl_started = false;
  pcmHoldReset();
  portENTER_CRITICAL(&g_rb_mux);
  g_rb_w = 0;
  g_rb_r = 0;
  portEXIT_CRITICAL(&g_rb_mux);
}

// ----- UZC / MP3 slot over I2S (fixed-length slot from Master) -----
static mp3dec_t g_mp3dec;
static bool g_mp3dec_ready = false;
static uint8_t g_slot_buf[UZC_SLOT_BUF_MAX];
static uint8_t g_decode_slot[UZC_SLOT_BUF_MAX];
static size_t g_slot_fill = 0;
static size_t g_slot_size = 0;
static uint16_t g_slot_max_frame = 0;
static bool g_slot_size_learned = false;
static UzcSlotFmt g_slot_fmt = UZC_SLOT_FMT_NONE;
static uint32_t g_uzc_skip_stereo_frames = 0;  // UZC 期間パディング（ゼロ）を捨てる
// Master calcUzcI2sTxRate と一致: bitRatePerChannel/8 Hz（96000bps → 12000Hz）
static const uint32_t UZC_I2S_RATE_HZ = 12000;
static const uint32_t UZC_FRAME_MS_DEFAULT = 26;
static const char UZC_WIRE_SYNC_MAGIC[] = "UZSY";
static const uint8_t UZC_WIRE_SYNC_VERSION = 1;
static const uint8_t UZC_WIRE_SYNC_CMD_START = 1;
static const uint8_t UZC_WIRE_SYNC_CMD_STOP = 2;
static const size_t UZC_WIRE_SYNC_BYTES = 14;
static UzcWireRxState g_uzc_wire_state = UZC_WIRE_HUNT_SYNC;
static UzcPeriodPhase g_uzc_period_phase = UZC_PHASE_SLOT;
static uint32_t g_uzc_phase_frames_left = 0;
static uint32_t g_uzc_pad_frames_skip = 0;
static uint32_t g_uzc_slot_frames_left = 0;
static bool g_uzc_slot_pending_decode = false;
static uint32_t g_uzc_stream_frame = 0;
static bool g_uzc_skip_lock_batch = false;
static uint32_t g_uzc_r_slot_frames = 0;
static uint32_t g_uzc_locked_hb_rx_bytes = 0;
static uint32_t g_uzc_locked_last_fill = 0;
static uint32_t g_uzc_locked_stale_ms = 0;
static uint32_t g_uzc_frame_ms = UZC_FRAME_MS_DEFAULT;

static uint32_t uzcPeriodStereoFrames() {
  // Master calcUzcPeriodStereoFrames と同等: 1152/44100 秒を I2S レートで表現
  const uint32_t mp3FrameMs = (uint32_t)((1152ULL * 1000ULL) / 44100ULL);
  if (g_uzc_frame_ms != 0 && g_uzc_frame_ms != mp3FrameMs) {
    return (UZC_I2S_RATE_HZ * g_uzc_frame_ms) / 1000U;
  }
  return (uint32_t)((UZC_I2S_RATE_HZ * 1152ULL + 44100U - 1U) / 44100U);
}

static void uzcInitPaddingSkip(bool logOnce) {
  if (!g_slot_size_learned || g_slot_size == 0) {
    g_uzc_skip_stereo_frames = 0;
    return;
  }
  const uint32_t slotStereoFrames = (uint32_t)((g_slot_size + 1U) / 2U);
  const uint32_t periodStereoFrames = uzcPeriodStereoFrames();
  g_uzc_skip_stereo_frames =
      (periodStereoFrames > slotStereoFrames) ? (periodStereoFrames - slotStereoFrames) : 0;
  if (logOnce) {
    DBG("UZC wire %lu Hz period=%lu slot=%lu padSkip=%lu",
        (unsigned long)UZC_I2S_RATE_HZ,
        (unsigned long)periodStereoFrames,
        (unsigned long)slotStereoFrames,
        (unsigned long)g_uzc_skip_stereo_frames);
  }
}
static mp3d_sample_t g_mp3_pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
static int16_t g_mp3_pcm_cache[1152 * 2];
static uint32_t g_mp3_pcm_frame_count = 0;
static uint32_t g_mp3_play_start = 0;
static uint32_t g_mp3_play_frames = 0;
static uint32_t g_mp3_stream_phase = 0;
static int16_t g_mp3_stereo_out[1152 * 2];
static TaskHandle_t g_uzc_decode_task = nullptr;
static volatile bool g_uzc_decode_busy = false;
static void uzcClearDecodeNotifications();
static void flushI2sRxFifo();
static bool uzcLockedSlotHeaderLooksValid(const uint8_t* slot);
static void uzcTryConsumeReadyLockedSlots();

static void resetUzcSlotAssembler() {
  g_slot_fill = 0;
  g_slot_size = 0;
  g_slot_max_frame = 0;
  g_slot_size_learned = false;
  g_slot_fmt = UZC_SLOT_FMT_NONE;
  g_uzc_skip_stereo_frames = 0;
  g_uzc_wire_state = UZC_WIRE_HUNT_SYNC;
  g_uzc_period_phase = UZC_PHASE_SLOT;
  g_uzc_phase_frames_left = 0;
  g_uzc_pad_frames_skip = 0;
  g_uzc_slot_frames_left = 0;
  g_uzc_slot_pending_decode = false;
  g_uzc_stream_frame = 0;
  g_uzc_skip_lock_batch = false;
  g_uzc_r_slot_frames = 0;
  g_i2s_tag_mp3_open = false;
  g_i2s_tag_data_words = 0;
  g_i2s_tag_pad_skip = 0;
  g_tag_stat_slots_rx = 0;
  g_tag_stat_decode_ok = 0;
  g_tag_stat_word_err = 0;
  g_tag_stat_align_err = 0;
  g_tag_stat_hdr_reject = 0;
  g_uzc_frame_ms = UZC_FRAME_MS_DEFAULT;
  g_mp3dec_ready = false;
  memset(&g_mp3dec, 0, sizeof(g_mp3dec));
  g_mp3_pcm_frame_count = 0;
  g_mp3_play_start = 0;
  g_mp3_play_frames = 0;
  g_mp3_stream_phase = 0;
  pcmHoldReset();
}

static void pushPcmFramesToRing(const int16_t* stereoFrames, uint32_t frameCount) {
  if (frameCount == 0) {
    return;
  }
  if (!(g_state == ST_CONNECT || g_state == ST_CONNECTING || g_state == ST_PAIRING)) {
    return;
  }

  portENTER_CRITICAL(&g_rb_mux);
  uint32_t w = g_rb_w;
  uint32_t r = g_rb_r;
  uint32_t used = rb_used_nolock(w, r);
  uint32_t freef = rb_free_nolock(used);
  uint32_t push = (frameCount <= freef) ? frameCount : freef;

  for (uint32_t i = 0; i < push; i++) {
    uint32_t idx = (w % RB_FRAMES) * 2;
    g_rb[idx + 0] = stereoFrames[i * 2 + 0];
    g_rb[idx + 1] = stereoFrames[i * 2 + 1];
    w++;
  }
  g_rb_w = w;
  portEXIT_CRITICAL(&g_rb_mux);
  if (push < frameCount) {
    g_pcm_push_drop_frames += frameCount - push;
  }
}

static void appendSlotBytesFromStereo(const int16_t* stereo, uint32_t stereoFrameCount, bool useRight) {
  for (uint32_t i = 0; i < stereoFrameCount && g_slot_fill < UZC_SLOT_BUF_MAX; i++) {
    const int16_t w = useRight ? stereo[i * 2 + 1] : stereo[i * 2 + 0];
    g_slot_buf[g_slot_fill++] = (uint8_t)(w & 0xFF);
    if (g_slot_fill < UZC_SLOT_BUF_MAX) {
      g_slot_buf[g_slot_fill++] = (uint8_t)((w >> 8) & 0xFF);
    }
  }
}

static uint16_t uzcReadU16Le(const uint8_t* p) {
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static bool uzcMp3SyncAt(const uint8_t* slot, size_t off) {
  return (slot[off] == 0xFF && ((slot[off + 1] & 0xE0) == 0xE0));
}

static uint16_t uzcMp3FrameByteLength(const uint8_t* h) {
  if (h[0] != 0xFF || ((h[1] & 0xE0) != 0xE0)) {
    return 0;
  }
  const uint8_t verBits = (h[1] >> 3) & 3;
  const uint8_t layerBits = (h[1] >> 1) & 3;
  if (layerBits != 1) {
    return 0;
  }
  const bool mpeg1 = (verBits == 3);
  const uint8_t brIdx = (h[2] >> 4) & 0x0F;
  const uint8_t srIdx = (h[2] >> 2) & 3;
  if (brIdx == 0 || brIdx == 0x0F || srIdx == 3) {
    return 0;
  }
  static const uint16_t kBrKbps[] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
  static const uint32_t kSrMpeg1[] = {44100, 48000, 32000, 0};
  static const uint32_t kSrMpeg2[] = {22050, 24000, 16000, 0};
  const uint32_t bitrate = (uint32_t)kBrKbps[brIdx] * 1000UL;
  const uint32_t samplerate = mpeg1 ? kSrMpeg1[srIdx] : kSrMpeg2[srIdx];
  if (bitrate == 0 || samplerate == 0) {
    return 0;
  }
  const uint8_t padding = (h[2] >> 1) & 1;
  const uint32_t numerator = mpeg1 ? (144UL * bitrate) : (72UL * bitrate);
  const uint32_t frameLen = (numerator / samplerate) + padding;
  if (frameLen < 4 || frameLen > 4096) {
    return 0;
  }
  return (uint16_t)frameLen;
}

static uint16_t uzcEffectiveMp3ByteLength(const uint8_t* mp3Data, uint16_t realField) {
  if (realField < 4) {
    return 0;
  }
  const uint16_t hdrLen = uzcMp3FrameByteLength(mp3Data);
  if (hdrLen == 0) {
    return realField;
  }
  return (realField < hdrLen) ? realField : hdrLen;
}

static bool uzcMp3Frame44kLooksValid(const uint8_t* mp3Hdr) {
  if (!uzcMp3SyncAt(mp3Hdr, 0)) {
    return false;
  }
  const bool mpeg1 = ((mp3Hdr[1] & 0x08) != 0);
  const uint8_t srIdx = (mp3Hdr[2] >> 2) & 3;
  return mpeg1 ? (srIdx == 0) : (srIdx == 1);
}

static bool uzcSlotMetaLooksValid(const uint8_t* slot) {
  const uint16_t maxField = uzcReadU16Le(slot + UZC_MAX_FRAME_SIZE_FIELD_OFF);
  const uint16_t realField = uzcReadU16Le(slot + UZC_REAL_FRAME_SIZE_FIELD_OFF);
  if (maxField < 4 || maxField > UZC_MAX_MP3_FRAME) {
    return false;
  }
  if (g_slot_size_learned && g_slot_max_frame != 0 && maxField != g_slot_max_frame) {
    return false;
  }
  if (realField < 4 || realField > maxField) {
    return false;
  }
  if (!uzcMp3SyncAt(slot, UZC_FRAME_DATA_OFF)) {
    return false;
  }
  if (!uzcMp3Frame44kLooksValid(slot + UZC_FRAME_DATA_OFF)) {
    return false;
  }
  const uint16_t hdrLen = uzcMp3FrameByteLength(slot + UZC_FRAME_DATA_OFF);
  return hdrLen >= 4 && hdrLen <= realField;
}

static bool uzcSlotV1LooksValidAt(const uint8_t* base) {
  const uint16_t realField = uzcReadU16Le(base);
  if (realField < 4 || realField > UZC_MAX_MP3_FRAME) {
    return false;
  }
  return uzcMp3Frame44kLooksValid(base + 2);
}

static bool uzcSlotHeaderLooksValidV2(const uint8_t* slot) {
  return uzcSlotMetaLooksValid(slot);
}

static bool uzcSlotHeaderLooksValidV1(const uint8_t* slot) {
  if (!uzcSlotV1LooksValidAt(slot)) {
    return false;
  }
  const uint16_t realField = uzcReadU16Le(slot);
  return (realField >= 80);
}

static bool uzcSlotHeaderLooksValid(const uint8_t* slot) {
  if (g_slot_fmt == UZC_SLOT_FMT_V1_REAL_FIRST) {
    return uzcSlotHeaderLooksValidV1(slot);
  }
  return uzcSlotHeaderLooksValidV2(slot);
}

static bool uzcSlotHeaderLooksValidAt(size_t offset) {
  if (g_slot_fmt == UZC_SLOT_FMT_V1_REAL_FIRST) {
    if (offset + 6 > g_slot_fill) {
      return false;
    }
    return uzcSlotHeaderLooksValidV1(g_slot_buf + offset);
  }
  if (offset + UZC_FRAME_DATA_OFF + 4 > g_slot_fill) {
    return false;
  }
  return uzcSlotHeaderLooksValidV2(g_slot_buf + offset);
}

static void uzcAlignSlotBufferPrefix(size_t offset) {
  if (offset == 0 || offset >= g_slot_fill) {
    return;
  }
  memmove(g_slot_buf, g_slot_buf + offset, g_slot_fill - offset);
  g_slot_fill -= offset;
}

static void uzcApplySlotLayout(size_t slotBytes, uint16_t maxFrame, UzcSlotFmt fmt, size_t alignOff) {
  if (alignOff > 0 && alignOff < g_slot_fill) {
    uzcAlignSlotBufferPrefix(alignOff);
  }
  g_slot_size = slotBytes;
  g_slot_max_frame = maxFrame;
  g_slot_fmt = fmt;
  g_slot_size_learned = true;
  uzcInitPaddingSkip(true);
}

static bool uzcTryDetectSlotLayoutScan() {
  size_t scanLen = g_slot_fill;
  if (scanLen > 512) {
    scanLen = 512;
  }

  size_t bestBase = (size_t)-1;
  size_t bestSlotBytes = 0;
  uint16_t bestMax = 0;
  uint16_t bestReal = 0;

  for (size_t base = 0; base + UZC_FRAME_DATA_OFF + 4 <= scanLen; base++) {
    if (!uzcMp3SyncAt(g_slot_buf + base, UZC_FRAME_DATA_OFF)) {
      continue;
    }
    const uint16_t maxField = uzcReadU16Le(g_slot_buf + base);
    const uint16_t realField = uzcReadU16Le(g_slot_buf + base + 2);
    if (maxField < 4 || maxField > UZC_MAX_MP3_FRAME) {
      continue;
    }
    if (realField < 4 || realField > maxField) {
      continue;
    }
    if (!uzcMp3Frame44kLooksValid(g_slot_buf + base + UZC_FRAME_DATA_OFF)) {
      continue;
    }
    const size_t slotBytes = (size_t)UZC_SLOT_META_BYTES + maxField;
    if (slotBytes > UZC_SLOT_BUF_MAX || base + slotBytes > scanLen) {
      continue;
    }
    bool hasSecond = false;
    if (base + slotBytes + UZC_FRAME_DATA_OFF + 2 <= scanLen) {
      hasSecond = uzcMp3SyncAt(g_slot_buf + base + slotBytes, UZC_FRAME_DATA_OFF);
    }
    if (bestBase == (size_t)-1 || base < bestBase || (base == bestBase && hasSecond)) {
      bestBase = base;
      bestSlotBytes = slotBytes;
      bestMax = maxField;
      bestReal = realField;
    }
  }

  if (bestBase != (size_t)-1) {
    uzcApplySlotLayout(bestSlotBytes, bestMax, UZC_SLOT_FMT_V2_META, bestBase);
    Serial.print(F("UZC slot V2 scan base="));
    Serial.print(bestBase);
    Serial.print(F(" size="));
    Serial.print(g_slot_size);
    Serial.print(F(" real="));
    Serial.println(bestReal);
    return true;
  }

  for (size_t mp3Off = 2; mp3Off + 2 < scanLen; mp3Off++) {
    if (!uzcMp3Frame44kLooksValid(g_slot_buf + mp3Off)) {
      continue;
    }
    const size_t base = mp3Off - 2;
    const uint16_t realField = uzcReadU16Le(g_slot_buf + base);
    if (realField < 4 || realField > UZC_MAX_MP3_FRAME) {
      continue;
    }
    static const uint16_t kV1Slots[] = {318, 316, 421, 424};
    for (uint16_t sz : kV1Slots) {
      if (realField <= (uint16_t)(sz - 2)) {
        if (base + sz <= scanLen && uzcSlotV1LooksValidAt(g_slot_buf + base + sz)) {
          uzcApplySlotLayout(sz, (uint16_t)(sz - 2), UZC_SLOT_FMT_V1_REAL_FIRST, base);
          Serial.print(F("UZC slot V1 scan base="));
          Serial.print(base);
          Serial.print(F(" size="));
          Serial.println(sz);
          return true;
        }
      }
    }
  }
  return false;
}

static bool uzcTryApplySlotSizeV1(uint16_t slotBytes, bool requireSecondHeader) {
  if (!uzcSlotV1LooksValidAt(g_slot_buf)) {
    return false;
  }
  if (requireSecondHeader) {
    if (g_slot_fill < (size_t)slotBytes * 2U) {
      return false;
    }
    if (!uzcSlotV1LooksValidAt(g_slot_buf + slotBytes)) {
      return false;
    }
  }
  g_slot_size = slotBytes;
  g_slot_max_frame = (uint16_t)(slotBytes - 2);
  g_slot_fmt = UZC_SLOT_FMT_V1_REAL_FIRST;
  g_slot_size_learned = true;
  uzcInitPaddingSkip(true);
  Serial.print(F("UZC slot V1: size="));
  Serial.println(g_slot_size);
  return true;
}

static void uzcDbgSlotPrefixOnce() {
  static bool done = false;
  if (done || g_slot_fill < 8) {
    return;
  }
  done = true;
  DBG("UZC raw[0..7]=%02X %02X %02X %02X %02X %02X %02X %02X",
      g_slot_buf[0], g_slot_buf[1], g_slot_buf[2], g_slot_buf[3],
      g_slot_buf[4], g_slot_buf[5], g_slot_buf[6], g_slot_buf[7]);
}

static void uzcTryDetectSlotLayout() {
  if (g_slot_size_learned) {
    return;
  }

  if (uzcTryDetectSlotLayoutScan()) {
    return;
  }

  if (g_slot_fill >= UZC_FRAME_DATA_OFF + 4 && uzcSlotMetaLooksValid(g_slot_buf)) {
    const uint16_t maxField = uzcReadU16Le(g_slot_buf + UZC_MAX_FRAME_SIZE_FIELD_OFF);
    const size_t slotBytes = (size_t)UZC_SLOT_META_BYTES + maxField;
    if (slotBytes <= UZC_SLOT_BUF_MAX) {
      g_slot_max_frame = maxField;
      g_slot_size = slotBytes;
      g_slot_fmt = UZC_SLOT_FMT_V2_META;
      g_slot_size_learned = true;
      uzcInitPaddingSkip(true);
      Serial.print(F("UZC slot V2 meta: size="));
      Serial.print(g_slot_size);
      Serial.print(F(" maxFrame="));
      Serial.println(g_slot_max_frame);
      return;
    }
  }

  if (g_slot_fill < 6) {
    return;
  }

  static const uint16_t kV1Prefer[] = {316, 318, 320, 422, 424, 426};
  if (uzcSlotV1LooksValidAt(g_slot_buf)) {
    for (uint16_t sz : kV1Prefer) {
      if (uzcTryApplySlotSizeV1(sz, false)) {
        return;
      }
    }
  }

  for (uint16_t sz = 300; sz <= 520; sz++) {
    if (uzcTryApplySlotSizeV1(sz, true)) {
      return;
    }
  }

  uzcDbgSlotPrefixOnce();
}

static void uzcSlideSlotBufferOneByte() {
  if (g_slot_fill == 0) {
    return;
  }
  memmove(g_slot_buf, g_slot_buf + 1, g_slot_fill - 1);
  g_slot_fill--;
}

static bool uzcSlotAlignedAtOffset(size_t offset) {
  if (g_slot_fmt == UZC_SLOT_FMT_V1_REAL_FIRST) {
    if (offset + 6 > g_slot_fill) {
      return false;
    }
    return uzcSlotV1LooksValidAt(g_slot_buf + offset);
  }
  if (offset + UZC_FRAME_DATA_OFF + 2 > g_slot_fill) {
    return false;
  }
  return uzcSlotMetaLooksValid(g_slot_buf + offset);
}

static void uzcTrimLeadingZeroPairs() {
  size_t off = 0;
  while (off + 1 < g_slot_fill && g_slot_buf[off] == 0 && g_slot_buf[off + 1] == 0) {
    off += 2;
  }
  if (off > 0) {
    uzcAlignSlotBufferPrefix(off);
  }
}

static void uzcUpdatePaddingSkipAfterSlot() {
  uzcInitPaddingSkip(false);
}

static bool uzcTryAlignSlotBuffer() {
  if (!g_slot_size_learned || g_slot_size == 0) {
    return false;
  }

  uzcTrimLeadingZeroPairs();

  if (uzcSlotAlignedAtOffset(0)) {
    return true;
  }

  size_t scanMax = g_slot_fill;
  if (scanMax > 512) {
    scanMax = 512;
  }
  for (size_t off = 1; off < scanMax; off++) {
    if (uzcSlotAlignedAtOffset(off)) {
      uzcAlignSlotBufferPrefix(off);
      return true;
    }
  }

  if (g_slot_fill > g_slot_size && g_slot_fill < g_slot_size + UZC_SYNC_SLIDE_MAX) {
    uzcSlideSlotBufferOneByte();
    return uzcSlotAlignedAtOffset(0);
  }

  static uint32_t s_resyncDbg = 0;
  if ((s_resyncDbg++ % 200U) == 0U) {
    DBG("UZC align wait fill=%u b0=%02X %02X %02X %02X", (unsigned)g_slot_fill,
        g_slot_buf[0], g_slot_buf[1], g_slot_buf[2], g_slot_buf[3]);
  }
  return false;
}

static uint32_t uzcConsumePaddingStereo(uint32_t stereoFrames) {
  if (g_uzc_skip_stereo_frames == 0 || stereoFrames == 0) {
    return 0;
  }
  const uint32_t skip = (g_uzc_skip_stereo_frames < stereoFrames) ? g_uzc_skip_stereo_frames : stereoFrames;
  g_uzc_skip_stereo_frames -= skip;
  return skip;
}

static void uzcConsumeOneSlotFromBuffer() {
  const size_t drop = g_slot_size;
  if (g_slot_fill > drop) {
    memmove(g_slot_buf, g_slot_buf + drop, g_slot_fill - drop);
    g_slot_fill -= drop;
  } else {
    g_slot_fill = 0;
  }
}

static int32_t mp3CacheSampleAt(uint32_t frameIdx) {
  return g_mp3_pcm_cache[frameIdx * 2];
}

static int16_t mp3SampleAbs(int16_t s) {
  return (s < 0) ? (int16_t)(-s) : s;
}

static uint32_t findMp3FirstEnergyOffset(int16_t threshold) {
  for (uint32_t off = 0; off + 8U <= g_mp3_pcm_frame_count; off++) {
    int16_t peak = 0;
    for (uint32_t j = 0; j < 8U; j++) {
      const int16_t a = mp3SampleAbs((int16_t)mp3CacheSampleAt(off + j));
      if (a > peak) {
        peak = a;
      }
    }
    if (peak >= threshold) {
      return off;
    }
  }
  return 0;
}

static int16_t mp3CachePeakInWindow(uint32_t start, uint32_t frames) {
  int16_t peak = 0;
  for (uint32_t i = 0; i < frames; i++) {
    if (start + i >= g_mp3_pcm_frame_count) {
      break;
    }
    const int16_t a = mp3SampleAbs((int16_t)mp3CacheSampleAt(start + i));
    if (a > peak) {
      peak = a;
    }
  }
  return peak;
}

static uint32_t findMp3PcmAlignOffset() {
  const uint32_t refLen = UZC_MP3_PCM_REF_SAMPLES;
  if (g_mp3_pcm_frame_count < refLen) {
    return 0;
  }

  const uint32_t scanMax = g_mp3_pcm_frame_count - refLen + 1U;
  uint32_t bestOff = 0;
  int64_t bestScore = INT64_MAX;

  for (uint32_t off = 0; off < scanMax; off++) {
    int64_t sumSq = 0;
    for (uint32_t i = 0; i < refLen; i++) {
      const int32_t err = mp3CacheSampleAt(off + i) - kUzcMp3PcmRef[i];
      sumSq += (int64_t)err * err;
    }
    if (sumSq < bestScore) {
      bestScore = sumSq;
      bestOff = off;
    }
  }
  return bestOff;
}

static void mp3UpdatePlayWindow() {
  const uint32_t energyOff = findMp3FirstEnergyOffset(3000);
  g_mp3_play_start = findMp3PcmAlignOffset();
  if (mp3CachePeakInWindow(g_mp3_play_start, 64U) < 3000) {
    g_mp3_play_start = energyOff;
  }
  if (g_mp3_play_start >= g_mp3_pcm_frame_count) {
    g_mp3_play_start = 0;
  }
  g_mp3_play_frames = UZC_MP3_PCM_REF_SAMPLES;
  if (g_mp3_play_start + g_mp3_play_frames > g_mp3_pcm_frame_count) {
    g_mp3_play_frames = g_mp3_pcm_frame_count - g_mp3_play_start;
  }
  if (g_mp3_play_frames < 256U) {
    g_mp3_play_start = 0;
    g_mp3_play_frames = g_mp3_pcm_frame_count;
  }

  static constexpr uint32_t TONE_CYCLE_SAMPLES = 100U;
  const uint32_t cycles = g_mp3_play_frames / TONE_CYCLE_SAMPLES;
  if (cycles >= 4U) {
    g_mp3_play_frames = cycles * TONE_CYCLE_SAMPLES;
  } else if (g_mp3_play_frames >= 400U) {
    g_mp3_play_frames = 400U;
  }
}

static bool decodeMp3SlotToCache(const uint8_t* slot, size_t dataOff, uint16_t realSize) {
  g_mp3_pcm_frame_count = 0;
  if (realSize < 4) {
    return false;
  }

  mp3dec_init(&g_mp3dec);
  g_mp3dec_ready = true;

  uint32_t offset = 0;
  while (offset + 4U <= realSize && g_mp3_pcm_frame_count < 1152U) {
    mp3dec_frame_info_t info;
    memset(&info, 0, sizeof(info));

    const int samplesPerCh =
        mp3dec_decode_frame(&g_mp3dec, slot + dataOff + offset, (int)(realSize - offset), g_mp3_pcm, &info);
    if (samplesPerCh <= 0 || info.frame_bytes <= 0) {
      break;
    }

    const uint32_t space = 1152U - g_mp3_pcm_frame_count;
    const int copyFrames = (samplesPerCh > (int)space) ? (int)space : samplesPerCh;
    const uint32_t base = g_mp3_pcm_frame_count;

    if (info.channels == 1) {
      for (int i = 0; i < copyFrames; i++) {
        g_mp3_pcm_cache[(base + (uint32_t)i) * 2 + 0] = g_mp3_pcm[i];
        g_mp3_pcm_cache[(base + (uint32_t)i) * 2 + 1] = g_mp3_pcm[i];
      }
    } else {
      for (int i = 0; i < copyFrames; i++) {
        g_mp3_pcm_cache[(base + (uint32_t)i) * 2 + 0] = g_mp3_pcm[i * 2];
        g_mp3_pcm_cache[(base + (uint32_t)i) * 2 + 1] = g_mp3_pcm[i * 2 + 1];
      }
    }

    g_mp3_pcm_frame_count += (uint32_t)copyFrames;
    offset += (uint32_t)info.frame_bytes;
  }

  return g_mp3_pcm_frame_count > 0;
}

static void decodeMp3SlotWorker() {
  if (!uzcSlotMetaLooksValid(g_decode_slot)) {
    g_tag_stat_hdr_reject++;
    DBG("MP3 decode hdr reject fill=%u b0=%02X %02X %02X %02X max=%u real=%u",
        (unsigned)g_slot_size, g_decode_slot[0], g_decode_slot[1], g_decode_slot[2], g_decode_slot[3],
        (unsigned)uzcReadU16Le(g_decode_slot), (unsigned)uzcReadU16Le(g_decode_slot + 2));
    return;
  }

  size_t dataOff = UZC_FRAME_DATA_OFF;
  uint16_t realSize = uzcReadU16Le(g_decode_slot + UZC_REAL_FRAME_SIZE_FIELD_OFF);
  if (g_slot_fmt == UZC_SLOT_FMT_V1_REAL_FIRST) {
    dataOff = 2;
    realSize = uzcReadU16Le(g_decode_slot);
  }

  if (!decodeMp3SlotToCache(g_decode_slot, dataOff, realSize)) {
    static uint32_t s_decodeFail = 0;
    if ((s_decodeFail++ % 50U) == 0U) {
      DBG("MP3 decode failed (realSize=%u)", (unsigned)realSize);
    }
    return;
  }

  mp3UpdatePlayWindow();
  if (g_mp3_pcm_frame_count == 0) {
    return;
  }

  g_tag_stat_decode_ok++;
  if (g_tag_stat_decode_ok <= 5U) {
    int16_t cachePeak = 0;
    for (uint32_t i = 0; i < g_mp3_pcm_frame_count; i++) {
      int16_t v = g_mp3_pcm_cache[i * 2 + 0];
      if (v < 0) {
        v = (int16_t)(-v);
      }
      if (v > cachePeak) {
        cachePeak = v;
      }
    }
    int16_t outPeak = 0;
    for (uint32_t i = 0; i < UZC_MP3_PERIOD_STEREO_441; i++) {
      const uint32_t idx = (g_mp3_stream_phase + i) % g_mp3_pcm_frame_count;
      int16_t v = g_mp3_pcm_cache[idx * 2 + 0];
      if (v < 0) {
        v = (int16_t)(-v);
      }
      if (v > outPeak) {
        outPeak = v;
      }
    }
    Serial.print(F("[MP3] decode ok #"));
    Serial.print(g_tag_stat_decode_ok);
    Serial.print(F(" pcm="));
    Serial.print(g_mp3_pcm_frame_count);
    Serial.print(F(" start="));
    Serial.print(g_mp3_play_start);
    Serial.print(F(" play="));
    Serial.print(g_mp3_play_frames);
    Serial.print(F(" phase="));
    Serial.print(g_mp3_stream_phase);
    Serial.print(F(" cachePeak="));
    Serial.print(cachePeak);
    Serial.print(F(" outPeak="));
    Serial.println(outPeak);
  }

  for (uint32_t i = 0; i < UZC_MP3_PERIOD_STEREO_441; i++) {
    const uint32_t idx = (g_mp3_stream_phase + i) % g_mp3_pcm_frame_count;
    const int16_t s = g_mp3_pcm_cache[idx * 2 + 0];
    g_mp3_stereo_out[i * 2 + 0] = s;
    g_mp3_stereo_out[i * 2 + 1] = s;
  }
  g_mp3_stream_phase = (g_mp3_stream_phase + UZC_MP3_PERIOD_STEREO_441) % g_mp3_pcm_frame_count;
  pcmHoldAppend(g_mp3_stereo_out, (uint32_t)UZC_MP3_PERIOD_STEREO_441);
}

static void uzcDecodeTask(void* /*param*/) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    waitForPcmPipelineSpace(UZC_MP3_PERIOD_STEREO_441);
    decodeMp3SlotWorker();
    g_uzc_decode_busy = false;
  }
}

static void uzcClearDecodeNotifications() {
  if (g_uzc_decode_task == nullptr) {
    return;
  }
  while (ulTaskNotifyTake(pdTRUE, 0) > 0) {
  }
  g_uzc_decode_busy = false;
}

static bool scheduleMp3SlotDecode() {
  if (g_uzc_decode_task == nullptr || g_uzc_decode_busy) {
    return false;
  }
  if (g_slot_fmt == UZC_SLOT_FMT_V2_META && !uzcSlotMetaLooksValid(g_slot_buf)) {
    return false;
  }
  if (g_slot_fmt == UZC_SLOT_FMT_V1_REAL_FIRST && !uzcSlotV1LooksValidAt(g_slot_buf)) {
    return false;
  }
  memcpy(g_decode_slot, g_slot_buf, g_slot_size);
  g_uzc_decode_busy = true;
  xTaskNotifyGive(g_uzc_decode_task);
  return true;
}

static bool scheduleMp3SlotDecodeLocked() {
  if (g_uzc_decode_task == nullptr || g_uzc_decode_busy) {
    return false;
  }
  memcpy(g_decode_slot, g_slot_buf, g_slot_size);
  g_uzc_decode_busy = true;
  xTaskNotifyGive(g_uzc_decode_task);
  return true;
}

static bool uzcWireSyncParamsLookValid(uint16_t slotSize, uint16_t maxFrame, uint16_t frameMs) {
  if (slotSize < UZC_SLOT_META_BYTES + 4 || slotSize > UZC_SLOT_BUF_MAX) {
    return false;
  }
  if (maxFrame < 4 || maxFrame > UZC_MAX_MP3_FRAME) {
    return false;
  }
  if (frameMs < 1 || frameMs > 200) {
    return false;
  }
  return true;
}

static uint32_t uzcLockedSlotStereoFrames() {
  return (uint32_t)((g_slot_size + 1U) / 2U);
}

static uint32_t uzcLockedPadStereoFrames() {
  const uint32_t periodStereoFrames = uzcPeriodStereoFrames();
  const uint32_t slotStereoFrames = uzcLockedSlotStereoFrames();
  return (periodStereoFrames > slotStereoFrames) ? (periodStereoFrames - slotStereoFrames) : 0;
}

static void uzcLockedTrimLeadingZeros() {
  size_t off = 0;
  while (off < g_slot_fill && g_slot_buf[off] == 0) {
    off++;
  }
  if (off > 0) {
    uzcAlignSlotBufferPrefix(off);
  }
}

static void uzcLockedDropWireSyncIfPresent() {
  const size_t scanMax = (g_slot_fill < 128) ? g_slot_fill : 128;
  for (size_t off = 0; off + 4 <= scanMax; off++) {
    if (memcmp(g_slot_buf + off, UZC_WIRE_SYNC_MAGIC, 4) != 0) {
      continue;
    }
    const size_t drop = off + g_slot_size;
    if (g_slot_fill > drop) {
      memmove(g_slot_buf, g_slot_buf + drop, g_slot_fill - drop);
      g_slot_fill -= drop;
    } else {
      g_slot_fill = 0;
    }
    DBG("UZC locked skip SYNC period off=%u", (unsigned)off);
    return;
  }
}

static bool uzcLockedAlignBufferToHeader() {
  uzcLockedTrimLeadingZeros();
  uzcLockedDropWireSyncIfPresent();
  uzcLockedTrimLeadingZeros();

  size_t scanMax = g_slot_fill;
  if (scanMax > 512) {
    scanMax = 512;
  }
  for (size_t off = 0; off + 6 <= scanMax; off++) {
    if (uzcLockedSlotHeaderLooksValid(g_slot_buf + off)) {
      if (off > 0) {
        uzcAlignSlotBufferPrefix(off);
      }
      return true;
    }
  }
  return false;
}

static bool uzcLockedSlotHeaderLooksValid(const uint8_t* slot) {
  const uint16_t maxField = uzcReadU16Le(slot + UZC_MAX_FRAME_SIZE_FIELD_OFF);
  const uint16_t realField = uzcReadU16Le(slot + UZC_REAL_FRAME_SIZE_FIELD_OFF);
  if (maxField != g_slot_max_frame) {
    return false;
  }
  if (realField < 4 || realField > maxField) {
    return false;
  }
  if (!uzcMp3SyncAt(slot, UZC_FRAME_DATA_OFF)) {
    return false;
  }
  if (!uzcMp3Frame44kLooksValid(slot + UZC_FRAME_DATA_OFF)) {
    return false;
  }
  const uint16_t hdrLen = uzcMp3FrameByteLength(slot + UZC_FRAME_DATA_OFF);
  return hdrLen >= 4 && hdrLen <= realField;
}

static void uzcApplyWireSyncStart(uint16_t slotSize, uint16_t maxFrame, uint16_t frameMs, size_t syncByteOff,
                                  uint32_t bufFramesAfterHdr) {
  g_slot_size = slotSize;
  g_slot_max_frame = maxFrame;
  g_slot_fmt = UZC_SLOT_FMT_V2_META;
  g_slot_size_learned = true;
  g_uzc_frame_ms = frameMs;
  g_uzc_wire_state = UZC_WIRE_LOCKED;
  uzcClearDecodeNotifications();
  g_mp3dec_ready = false;
  memset(&g_mp3dec, 0, sizeof(g_mp3dec));
  g_slot_fill = 0;
  g_uzc_slot_frames_left = 0;
  g_uzc_slot_pending_decode = false;
  const uint32_t period = uzcPeriodStereoFrames();
  const uint32_t syncStereoFrames = (uint32_t)((syncByteOff + UZC_WIRE_SYNC_BYTES + 1U) / 2U);
  const uint32_t remainSync = (period > syncStereoFrames) ? (period - syncStereoFrames) : 0U;
  const uint32_t totalSkip = remainSync + period;
  if (bufFramesAfterHdr < totalSkip) {
    g_uzc_pad_frames_skip = totalSkip - bufFramesAfterHdr;
    g_uzc_stream_frame = 0;
  } else {
    g_uzc_pad_frames_skip = 0;
    g_uzc_stream_frame = bufFramesAfterHdr;
  }
  g_uzc_skip_lock_batch = true;
  g_uzc_locked_hb_rx_bytes = 0;
  g_uzc_locked_last_fill = 0;
  g_uzc_locked_stale_ms = millis();
  DBG("UZC wire %lu Hz period=%lu slot=%lu pad=%lu skip=%lu bufF=%lu syncOff=%u (phase)",
      (unsigned long)UZC_I2S_RATE_HZ,
      (unsigned long)period,
      (unsigned long)uzcLockedSlotStereoFrames(),
      (unsigned long)uzcLockedPadStereoFrames(),
      (unsigned long)g_uzc_pad_frames_skip, (unsigned long)bufFramesAfterHdr, (unsigned)syncByteOff);
  Serial.print(F("UZC wire SYNC LOCKED slot="));
  Serial.print(slotSize);
  Serial.print(F(" max="));
  Serial.print(maxFrame);
  Serial.print(F(" frameMs="));
  Serial.println(frameMs);
}

static void uzcApplyWireSyncStop() {
  DBG("UZC wire SYNC STOP -> hunt");
  resetUzcSlotAssembler();
}

static bool uzcTryParseWireSyncAt(size_t off) {
  if (off + UZC_WIRE_SYNC_BYTES > g_slot_fill) {
    return false;
  }
  const uint8_t* p = g_slot_buf + off;
  if (memcmp(p, UZC_WIRE_SYNC_MAGIC, 4) != 0 || p[4] != UZC_WIRE_SYNC_VERSION) {
    return false;
  }
  const uint8_t cmd = p[5];
  const uint16_t slotSize = uzcReadU16Le(p + 6);
  const uint16_t maxFrame = uzcReadU16Le(p + 8);
  const uint16_t frameMs = uzcReadU16Le(p + 10);
  if (!uzcWireSyncParamsLookValid(slotSize, maxFrame, frameMs)) {
    return false;
  }

  const size_t drop = off + UZC_WIRE_SYNC_BYTES;
  if (g_slot_fill > drop) {
    memmove(g_slot_buf, g_slot_buf + drop, g_slot_fill - drop);
    g_slot_fill -= drop;
  } else {
    g_slot_fill = 0;
  }

  if (cmd == UZC_WIRE_SYNC_CMD_START) {
    const uint32_t bufFramesAfterHdr = (uint32_t)(g_slot_fill / 2U);
    uzcApplyWireSyncStart(slotSize, maxFrame, frameMs, off, bufFramesAfterHdr);
    return true;
  }
  if (cmd == UZC_WIRE_SYNC_CMD_STOP) {
    uzcApplyWireSyncStop();
    return true;
  }
  return false;
}

static void uzcTryHuntWireSync() {
  uzcTrimLeadingZeroPairs();
  for (size_t off = 0; off + UZC_WIRE_SYNC_BYTES <= g_slot_fill; off++) {
    if (memcmp(g_slot_buf + off, UZC_WIRE_SYNC_MAGIC, 4) == 0) {
      if (uzcTryParseWireSyncAt(off)) {
        return;
      }
    }
  }
  if (g_slot_fill > 512) {
    uzcAlignSlotBufferPrefix(g_slot_fill - 256);
  }
}

static void uzcLockedResetPhaseAfterBadHeader() {
  const uint32_t period = uzcPeriodStereoFrames();
  g_slot_fill = 0;
  g_uzc_slot_frames_left = 0;
  if (period > 0) {
    g_uzc_stream_frame = ((g_uzc_stream_frame / period) + 1U) * period;
  }
}

static bool uzcLockedAlignCollectedSlotInPlace() {
  if (g_slot_fill < g_slot_size) {
    return false;
  }
  if (uzcLockedSlotHeaderLooksValid(g_slot_buf)) {
    return true;
  }
  const size_t scanMax = (g_slot_fill > g_slot_size + 32U) ? 32U : (g_slot_fill - g_slot_size);
  for (size_t off = 2; off <= scanMax; off += 2U) {
    if (!uzcLockedSlotHeaderLooksValid(g_slot_buf + off)) {
      continue;
    }
    memmove(g_slot_buf, g_slot_buf + off, g_slot_size);
    g_slot_fill = g_slot_size;
    DBG("UZC slot hdr slide off=%u", (unsigned)off);
    return true;
  }
  return false;
}

static void uzcLockedDispatchCollectedSlot() {
  if (!g_slot_size_learned || g_slot_fill < g_slot_size) {
    return;
  }
  if (!uzcLockedAlignCollectedSlotInPlace()) {
    static uint32_t s_hdrFail = 0;
    if ((s_hdrFail++ % 20U) == 0U) {
      DBG("UZC bad hdr sf=%lu pos=%lu b0=%02X %02X %02X %02X",
          (unsigned long)g_uzc_stream_frame,
          (unsigned long)((g_uzc_stream_frame + 1U) % uzcPeriodStereoFrames()),
          g_slot_buf[0], g_slot_buf[1], g_slot_buf[2], g_slot_buf[3]);
    }
    uzcLockedResetPhaseAfterBadHeader();
    return;
  }
  memcpy(g_decode_slot, g_slot_buf, g_slot_size);
  decodeMp3SlotWorker();
  g_slot_fill = 0;
}

static void uzcLockedAppendChannelByte(int16_t w) {
  if (g_slot_fill < UZC_SLOT_BUF_MAX) {
    g_slot_buf[g_slot_fill++] = (uint8_t)(w & 0xFF);
  }
  if (g_slot_fill < UZC_SLOT_BUF_MAX) {
    g_slot_buf[g_slot_fill++] = (uint8_t)((w >> 8) & 0xFF);
  }
}

static void uzcLockedProcessStereoFrames(const int16_t* stereo, uint32_t stereoFrameCount, bool useRight) {
  const uint32_t period = uzcPeriodStereoFrames();
  const uint32_t slotFrames = uzcLockedSlotStereoFrames();

  for (uint32_t i = 0; i < stereoFrameCount; i++) {
    if (g_uzc_pad_frames_skip > 0) {
      g_uzc_pad_frames_skip--;
      g_uzc_stream_frame++;
      continue;
    }

    const int16_t w = useRight ? stereo[i * 2 + 1] : stereo[i * 2 + 0];
    const uint32_t pos = (g_uzc_stream_frame + 1U) % period;

    if (pos < slotFrames) {
      if (pos == 0) {
        g_slot_fill = 0;
      }
      uzcLockedAppendChannelByte(w);
      if (pos == slotFrames - 1U) {
        uzcLockedDispatchCollectedSlot();
      }
    }
    g_uzc_stream_frame++;
  }
}

static void uzcTryConsumeReadyLockedSlots() {
}

static void consumeFilledUzcSlot() {
  if (!g_slot_size_learned) {
    return;
  }
  if (!uzcTryAlignSlotBuffer()) {
    return;
  }

  if (!scheduleMp3SlotDecode()) {
    return;
  }
  uzcConsumeOneSlotFromBuffer();
  uzcUpdatePaddingSkipAfterSlot();
}

static bool uzcShouldProcessI2s() {
#if UZU_I2S_TEST_BYPASS_BT
  // sintest 等: BT 未接続でも I2S タグ処理を継続（CONNECTING 中に drain しない）
  return true;
#else
  return (g_state == ST_CONNECT && g_audio_enable);
#endif
}

static void drainI2sRx() {
  static int16_t trash[128 * 2];
  size_t rbytes = 0;
  while (i2s_read(I2S_NUM_1, trash, sizeof(trash), &rbytes, 0) == ESP_OK && rbytes > 0) {
  }
}

static void flushI2sRxFifo() {
  drainI2sRx();
}

static bool stereoChunkAllZero(const int16_t* stereo, uint32_t stereoFrames) {
  if (stereoFrames == 0) {
    return false;
  }
  for (uint32_t i = 0; i < stereoFrames * 2; i++) {
    if (stereo[i] != 0) {
      return false;
    }
  }
  return true;
}

static void i2sTagRxApplyRate(uint32_t hz) {
  if (g_i2s_input_rate == hz) {
    return;
  }
  g_i2s_input_rate = hz;
  i2s_rx_apply_nominal_rate(hz);
  g_mp3dec_ready = false;
  memset(&g_mp3dec, 0, sizeof(g_mp3dec));
  DBG("I2S tag rate -> %lu Hz", (unsigned long)hz);
}

static void i2sTagAppendL(int16_t l) {
  if (g_slot_fill < UZC_SLOT_BUF_MAX) {
    g_slot_buf[g_slot_fill++] = (uint8_t)(l & 0xFF);
  }
  if (g_slot_fill < UZC_SLOT_BUF_MAX) {
    g_slot_buf[g_slot_fill++] = (uint8_t)((l >> 8) & 0xFF);
  }
}

static uint16_t i2sTagExpectedSlotBytes() {
  if (g_slot_max_frame != 0) {
    return (uint16_t)(UZC_SLOT_META_BYTES + g_slot_max_frame);
  }
  return 0;
}

static uint16_t i2sTagExpectedSlotWords() {
  const uint16_t bytes = i2sTagExpectedSlotBytes();
  if (bytes == 0) {
    return 0;
  }
  return (uint16_t)(((uint32_t)bytes + 1U) / 2U);
}

static void i2sTagBeginPadSkip(uint16_t dataWords) {
  const uint32_t tagged = (uint32_t)dataWords + 2U;
  if (UZC_MP3_PERIOD_STEREO_441 > tagged) {
    g_i2s_tag_pad_skip = UZC_MP3_PERIOD_STEREO_441 - tagged;
  } else {
    g_i2s_tag_pad_skip = 0;
  }
}

static bool i2sTagHeaderLooksValidAt(const uint8_t* slot, uint16_t expectMax) {
  const uint16_t maxField = uzcReadU16Le(slot + UZC_MAX_FRAME_SIZE_FIELD_OFF);
  const uint16_t realField = uzcReadU16Le(slot + UZC_REAL_FRAME_SIZE_FIELD_OFF);
  if (expectMax != 0 && maxField != expectMax) {
    return false;
  }
  if (maxField < 4 || maxField > UZC_MAX_MP3_FRAME) {
    return false;
  }
  if (realField < 4 || realField > maxField) {
    return false;
  }
  if (!uzcMp3SyncAt(slot, UZC_FRAME_DATA_OFF)) {
    return false;
  }
  if (!uzcMp3Frame44kLooksValid(slot + UZC_FRAME_DATA_OFF)) {
    return false;
  }
  const uint16_t hdrLen = uzcMp3FrameByteLength(slot + UZC_FRAME_DATA_OFF);
  return hdrLen >= 4 && hdrLen <= realField;
}

static bool i2sTagPrepareDecodeSlot() {
  if (g_slot_fill < 8) {
    return false;
  }
  const uint16_t expectMax = g_slot_max_frame;
  const size_t scanMax = (g_slot_fill > 36U) ? 32U : (g_slot_fill - 8U);
  for (size_t off = 0; off <= scanMax; off += 2U) {
    if (off + 8U > g_slot_fill) {
      break;
    }
    if (!i2sTagHeaderLooksValidAt(g_slot_buf + off, expectMax)) {
      continue;
    }
    const uint16_t maxField = uzcReadU16Le(g_slot_buf + off + UZC_MAX_FRAME_SIZE_FIELD_OFF);
    const uint16_t realField = uzcReadU16Le(g_slot_buf + off + UZC_REAL_FRAME_SIZE_FIELD_OFF);
    size_t slotBytes = (size_t)UZC_SLOT_META_BYTES + (size_t)maxField;
    const size_t avail = g_slot_fill - off;
    if (avail < 8) {
      continue;
    }
    if (slotBytes > avail) {
      if (avail + 8 < slotBytes) {
        continue;
      }
      slotBytes = avail;
    }
    if (off > 0) {
      DBG("I2S tag slot hdr slide off=%u", (unsigned)off);
    }
    g_slot_size = slotBytes;
    g_slot_max_frame = maxField;
    g_slot_fmt = UZC_SLOT_FMT_V2_META;
    g_slot_size_learned = true;
    memcpy(g_decode_slot, g_slot_buf + off, slotBytes);
    return true;
  }
  return false;
}

static void i2sTagDispatchMp3Slot() {
  if (g_slot_fill == 0) {
    return;
  }
  g_tag_stat_slots_rx++;
  const uint16_t expWords = i2sTagExpectedSlotWords();
  const uint16_t expBytes = i2sTagExpectedSlotBytes();
  if (expWords != 0 &&
      (g_i2s_tag_data_words != expWords || g_slot_fill != expBytes)) {
    g_tag_stat_word_err++;
    if ((g_tag_stat_word_err <= 5U) || ((g_tag_stat_word_err % 100U) == 0U)) {
      DBG("I2S tag slot frame err words=%u/%u fill=%u/%u",
          (unsigned)g_i2s_tag_data_words, (unsigned)expWords,
          (unsigned)g_slot_fill, (unsigned)expBytes);
    }
    if (expBytes != 0 && g_slot_fill + 16U < expBytes) {
      g_slot_fill = 0;
      g_i2s_tag_data_words = 0;
      return;
    }
  }
  if (!i2sTagPrepareDecodeSlot()) {
    g_tag_stat_align_err++;
    DBG("I2S tag slot hdr fail fill=%u b0=%02X %02X %02X %02X max=%u real=%u",
        (unsigned)g_slot_fill, g_slot_buf[0], g_slot_buf[1], g_slot_buf[2], g_slot_buf[3],
        (unsigned)uzcReadU16Le(g_slot_buf), (unsigned)uzcReadU16Le(g_slot_buf + 2));
    g_slot_fill = 0;
    g_i2s_tag_data_words = 0;
    return;
  }
  decodeMp3SlotWorker();
  g_slot_fill = 0;
  g_i2s_tag_data_words = 0;
}

static void i2sTagProcessStereoFrames(const int16_t* stereo, uint32_t stereoFrameCount) {
  static uint32_t s_hbMs = 0;
  const uint32_t now = millis();

  for (uint32_t i = 0; i < stereoFrameCount; i++) {
    if (g_i2s_tag_pad_skip > 0) {
      g_i2s_tag_pad_skip--;
      continue;
    }

    const int16_t l = stereo[i * 2 + 0];
    const uint16_t tag = i2sRxTagValue(stereo[i * 2 + 1]);
    g_i2s_dbg_last_l = l;
    g_i2s_dbg_last_r = tag;

    switch (tag) {
      case 0xAAAA:
        g_pcm_input_armed = true;
        g_tag_stat_slots_rx++;
        i2sTagRxApplyRate(44100);
        g_i2s_tag_mp3_open = false;
        g_i2s_tag_pad_skip = 0;
        {
          int16_t pcm[2] = {l, l};
          pcmHoldAppend(pcm, 1);
        }
        break;
      case 0xAA00:
        g_pcm_input_armed = true;
        g_i2s_tag_mp3_open = true;
        g_i2s_tag_data_words = 0;
        g_i2s_tag_pad_skip = 0;
        g_slot_fill = 0;
        g_slot_fmt = UZC_SLOT_FMT_V2_META;
        g_slot_size = 0;
        g_slot_max_frame = 0;
        g_slot_size_learned = false;
        i2sTagRxApplyRate(44100);
        break;
      case 0xAA55:
        if (g_i2s_tag_mp3_open) {
          i2sTagAppendL(l);
          g_i2s_tag_data_words++;
        }
        break;
      case 0x5500:
        if (g_i2s_tag_mp3_open) {
          const uint16_t dataWords = g_i2s_tag_data_words;
          i2sTagDispatchMp3Slot();
          g_i2s_tag_mp3_open = false;
          i2sTagBeginPadSkip(dataWords);
        }
        break;
      case 0xFFFF:
        g_i2s_tag_pad_skip = 0;
        break;
      case 0x0000:
        break;
      default:
        break;
    }
  }

#if UZU_I2S_TEST_BYPASS_BT
  if ((uint32_t)(now - s_hbMs) >= 2000U) {
    s_hbMs = now;
    const uint32_t err = g_tag_stat_word_err + g_tag_stat_align_err + g_tag_stat_hdr_reject;
    const uint32_t pct = (g_tag_stat_slots_rx > 0)
                             ? (uint32_t)((err * 100UL) / g_tag_stat_slots_rx)
                             : 0U;
    uint32_t rbUsed = 0;
    uint32_t holdUsed = 0;
    portENTER_CRITICAL(&g_rb_mux);
    rbUsed = rb_used_nolock(g_rb_w, g_rb_r);
    holdUsed = pcmHoldUsedNolock();
    portEXIT_CRITICAL(&g_rb_mux);
    DBG("I2S tag hb rate=%lu rx=%lu ok=%lu err=%lu (%lu%%) hold=%lu rb=%lu u/d=%lu/%lu peek L=%d R=%04X armed=%d play=%d peak=%d",
        (unsigned long)g_i2s_input_rate,
        (unsigned long)g_tag_stat_slots_rx, (unsigned long)g_tag_stat_decode_ok,
        (unsigned long)err, (unsigned long)pct,
        (unsigned long)holdUsed, (unsigned long)rbUsed,
        (unsigned long)g_pcm_underrun_frames, (unsigned long)g_pcm_push_drop_frames,
        (int)g_i2s_dbg_last_l, (unsigned)g_i2s_dbg_last_r, (int)g_pcm_input_armed,
        (int)g_pcm_playback_active, (int)pcmHoldPeakAbs(256));
#if UZU_I2S_TEST_BYPASS_BT
    if (g_i2s_bench_playback) {
      Serial.printf("[I2S-BENCH] pcm_out=%lu peak=%d hold=%lu u/d=%lu/%lu\n",
                    (unsigned long)g_i2s_bench_pcm_out, (int)g_i2s_bench_peak,
                    (unsigned long)holdUsed,
                    (unsigned long)g_pcm_underrun_frames,
                    (unsigned long)g_pcm_push_drop_frames);
    }
#endif
  }
#endif
}

static void serviceI2SInput() {
  static bool rateInit = false;
  if (!rateInit) {
    rateInit = true;
    g_i2s_input_rate = 44100;
    i2s_rx_apply_nominal_rate(44100);
  }

  if (!uzcShouldProcessI2s()) {
    drainI2sRx();
    return;
  }

  for (int burst = 0; burst < 48; burst++) {
    static int16_t tmp[128 * 2];
    size_t rbytes = 0;
    esp_err_t e = i2s_read(I2S_NUM_1, tmp, sizeof(tmp), &rbytes, 0);
    if (e != ESP_OK || rbytes == 0) {
      return;
    }
    const uint32_t frames = (uint32_t)(rbytes / (sizeof(int16_t) * 2));
    if (frames == 0) {
      return;
    }
    i2sTagProcessStereoFrames(tmp, frames);
  }
}

static int32_t audio_cb(Frame *frames, int32_t frame_count) {
  if (frames == nullptr || frame_count <= 0) {
    return (frame_count > 0) ? frame_count : 0;
  }

  if (!g_audio_enable) {
    for (int i = 0; i < frame_count; i++) {
      frames[i].channel1 = 0;
      frames[i].channel2 = 0;
    }
    return frame_count;
  }

  if (!g_pcm_playback_active) {
    for (int i = 0; i < frame_count; i++) {
      frames[i].channel1 = 0;
      frames[i].channel2 = 0;
    }
    return frame_count;
  }

  uint32_t filled = 0;
  portENTER_CRITICAL(&g_rb_mux);
  uint32_t w = g_rb_w;
  uint32_t r = g_rb_r;
  uint32_t rbAvail = rb_used_nolock(w, r);
  uint32_t rbPop = ((uint32_t)frame_count <= rbAvail) ? (uint32_t)frame_count : rbAvail;
  for (uint32_t i = 0; i < rbPop; i++) {
    uint32_t idx = (r % RB_FRAMES) * 2;
    frames[i].channel1 = g_rb[idx + 0];
    frames[i].channel2 = g_rb[idx + 1];
    r++;
  }
  g_rb_r = r;
  filled = rbPop;
  portEXIT_CRITICAL(&g_rb_mux);

  if (filled < (uint32_t)frame_count) {
    g_pcm_underrun_frames += (uint32_t)frame_count - filled;
  }
  for (int32_t i = (int32_t)filled; i < frame_count; i++) {
    frames[i].channel1 = 0;
    frames[i].channel2 = 0;
  }
  return frame_count;
}

static void startMedia() {
  esp_err_t r = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
  uint32_t holdUsed = 0;
  portENTER_CRITICAL(&g_rb_mux);
  holdUsed = pcmHoldUsedNolock();
  portEXIT_CRITICAL(&g_rb_mux);
  DBG("MEDIA START -> %d (audioSt=%d hold=%lu rx=%lu decodeOk=%lu)",
      (int)r, (int)g_audio_state, (unsigned long)holdUsed,
      (unsigned long)g_tag_stat_slots_rx, (unsigned long)g_tag_stat_decode_ok);
}

// ============================================================
// State machine
// ============================================================
static PairingSubState g_pairing_sub = PSS_SCAN;
static uint32_t g_state_started_ms = 0;

static esp_bd_addr_t g_last_mac = {0};
static bool g_has_last_mac = false;

static esp_bd_addr_t g_pending_mac = {0};
static bool g_has_pending = false;

static bool g_reconnect_active = false;
static uint8_t g_reconnect_try_count = 0;
static uint32_t g_reconnect_started_ms = 0;

static const uint32_t TIMEOUT_PAIRING_MS    = 30000;
static const uint32_t TIMEOUT_CONNECTING_MS = 8000;
static const uint32_t TIMEOUT_ERROR_SHOW_MS = 3000;
static const uint32_t LONG_PRESS_MS         = 3000;
static const uint32_t VERY_LONG_PRESS_MS    = 8000;
static const uint32_t BLINK_FAST_MS         = 200;
static const uint32_t BLINK_SLOW_MS         = 800;
static const uint32_t BLINK_ERASE_MS        = 120;
static const uint32_t RECONNECT_WAIT_FIRST_MS = 5000;
static const uint32_t RECONNECT_WAIT_RETRY_MS = 5000;
static const uint8_t  RECONNECT_MAX_TRIES    = 5;
static const uint8_t  RECONNECT_EXTRA_CLEANUP_FROM_TRY = 3;
static const uint32_t RECONNECT_EXTRA_CLEANUP_DELAY_MS = 120;

// GAP discovery based pairing
static const int RSSI_THRESHOLD_DBM = -70;
static const uint8_t DISCOVERY_SECONDS = 6;
static const uint8_t DISCOVERY_MAX_RESP = 0;
static const uint32_t DISCOVERY_RESTART_DELAY_MS = 800;
static const int MAX_DISC_DEVICES = 16;

struct GapDevice {
  bool used;
  esp_bd_addr_t bda;
  char name[64];
  int rssi_last;
  int rssi_best;
  uint32_t cod;
  bool has_cod_rssi;
  uint32_t seen_count;
};

struct GapBest {
  bool valid;
  esp_bd_addr_t bda;
  char name[64];
  int rssi;
  uint32_t cod;
  int table_index;
};

static GapDevice g_disc_devices[MAX_DISC_DEVICES];
static GapBest g_gap_best;
static volatile bool g_gap_discovery_running = false;
static volatile bool g_gap_connect_request = false;
static volatile uint32_t g_gap_last_stop_ms = 0;
static volatile uint32_t g_gap_raw_events = 0;
static uint32_t g_gap_round = 0;
static bool g_connecting_to_known = false;
static bool g_gap_require_lastmac = false;

static const char* stateName(BtState s) {
  switch (s) {
    case ST_IDLE:       return "IDLE";
    case ST_PAIRING:    return "PAIRING";
    case ST_CONNECTING: return "CONNECTING";
    case ST_RECONNECT_WAIT: return "RECONNECT_WAIT";
    case ST_RECONNECT_SCAN: return "RECONNECT_SCAN";
    case ST_CONNECT:    return "CONNECT";
    case ST_ERASE:      return "ERASE";
    case ST_ERROR:      return "ERROR";
    case ST_TIMEOUT:    return "TIMEOUT";
    default:            return "UNKNOWN";
  }
}

static const char* pairingSubName(PairingSubState s) {
  switch (s) {
    case PSS_SCAN:        return "SCAN";
    case PSS_CONNECT_NEW: return "CONNECT_NEW";
    default:              return "UNKNOWN";
  }
}

static void setPairingSubState(PairingSubState s) {
  if (g_pairing_sub != s) {
    DBG("PAIRING_SUB %s -> %s", pairingSubName(g_pairing_sub), pairingSubName(s));
    g_pairing_sub = s;
  }
}

static void setState(BtState s) {
  BtState old = (BtState)g_state;
  g_state = s;
  g_state_started_ms = millis();
  DBG("STATE %s -> %s", stateName(old), stateName(s));
}

static bool macEquals(const esp_bd_addr_t a, const esp_bd_addr_t b) {
  return memcmp(a, b, 6) == 0;
}

static void startReconnectAttempt() {
  if (!g_has_last_mac) {
    DBG("startReconnectAttempt() skipped: no stored device");
    enterIdle();
    return;
  }

  uint8_t next_try = g_reconnect_try_count + 1;

  if (next_try >= RECONNECT_EXTRA_CLEANUP_FROM_TRY) {
    DBG("extra cleanup before retry #%u", (unsigned)next_try);
    disconnectNow();
    delay(RECONNECT_EXTRA_CLEANUP_DELAY_MS);
  }

  g_audio_enable = false;
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_reconnect_active = true;
  setPairingSubState(PSS_SCAN);

  g_reconnect_try_count = next_try;
  DBG("RECONNECT_WAIT done -> CONNECTING (try #%u)", (unsigned)g_reconnect_try_count);
  startConnectToKnown(g_last_mac);
  setState(ST_CONNECTING);
}

static void updateLed() {
  static uint32_t lastBlue = 0;
  static uint32_t lastRed  = 0;
  static bool blinkBlue = false;
  static bool blinkRed  = false;

  uint32_t now = millis();

  switch (g_state) {
    case ST_IDLE:
      setAllLedOff();
      return;

    case ST_CONNECT:
      setBlueLed(true);
      setRedLed(false);
      return;

    case ST_ERROR:
    case ST_TIMEOUT:
      setBlueLed(false);
      setRedLed(true);
      return;

    case ST_PAIRING:
      if (g_pairing_sub == PSS_SCAN) {
        if (now - lastRed >= BLINK_FAST_MS) {
          lastRed = now;
          blinkRed = !blinkRed;
        }
      } else {
        if (now - lastRed >= BLINK_SLOW_MS) {
          lastRed = now;
          blinkRed = !blinkRed;
        }
      }
      setBlueLed(false);
      setRedLed(blinkRed);
      return;

    case ST_CONNECTING:
    case ST_RECONNECT_WAIT:
    case ST_RECONNECT_SCAN:
      if (now - lastBlue >= BLINK_SLOW_MS) {
        lastBlue = now;
        blinkBlue = !blinkBlue;
      }
      setBlueLed(blinkBlue);
      setRedLed(false);
      return;

    case ST_ERASE:
      setBlueLed(false);
      setRedLed(true);
      return;
  }
}

// ============================================================
// NVS helpers
// ============================================================
static bool loadLastMac(esp_bd_addr_t out) {
  prefs.begin(NVS_NS, true);
  size_t n = prefs.getBytes(KEY_MAC, out, 6);
  prefs.end();
  return (n == 6);
}

static void saveLastMac(const esp_bd_addr_t mac) {
  prefs.begin(NVS_NS, false);
  prefs.putBytes(KEY_MAC, mac, 6);
  prefs.end();
}

static void clearLastMac() {
  prefs.begin(NVS_NS, false);
  prefs.remove(KEY_MAC);
  prefs.end();
}

// ============================================================
// BT bond clear
// ============================================================
static void clearBondedDevices() {
  int dev_num = esp_bt_gap_get_bond_device_num();
  DBG("bonded device count = %d", dev_num);
  if (dev_num <= 0) return;

  esp_bd_addr_t *dev_list = (esp_bd_addr_t *)malloc(dev_num * sizeof(esp_bd_addr_t));
  if (!dev_list) {
    DBG("malloc failed in clearBondedDevices");
    return;
  }

  if (esp_bt_gap_get_bond_device_list(&dev_num, dev_list) == ESP_OK) {
    for (int i = 0; i < dev_num; i++) {
      DBG("remove bond %02X:%02X:%02X:%02X:%02X:%02X",
          dev_list[i][0], dev_list[i][1], dev_list[i][2],
          dev_list[i][3], dev_list[i][4], dev_list[i][5]);
      esp_bt_gap_remove_bond_device(dev_list[i]);
      delay(30);
    }
  }
  free(dev_list);
}

// ============================================================
// I2C slave port (UzuCast_Master_Slave_I2C_Spec.md)
// ============================================================
#if UZU_ENABLE_I2C
static bool isMacIgnored(const esp_bd_addr_t mac) {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < I2C_IGNORE_MAX; i++) {
    if (!g_i2c_ignored[i].active) {
      continue;
    }
    if ((int32_t)(now - g_i2c_ignored[i].until_ms) >= 0) {
      g_i2c_ignored[i].active = false;
      continue;
    }
    if (macEquals(g_i2c_ignored[i].mac, mac)) {
      return true;
    }
  }
  return false;
}

static void i2cAddIgnoredMac(const esp_bd_addr_t mac) {
  const uint32_t until = millis() + I2C_IGNORE_MS;
  for (uint8_t i = 0; i < I2C_IGNORE_MAX; i++) {
    if (g_i2c_ignored[i].active && macEquals(g_i2c_ignored[i].mac, mac)) {
      g_i2c_ignored[i].until_ms = until;
      return;
    }
  }
  for (uint8_t i = 0; i < I2C_IGNORE_MAX; i++) {
    if (!g_i2c_ignored[i].active) {
      memcpy(g_i2c_ignored[i].mac, mac, ESP_BD_ADDR_LEN);
      g_i2c_ignored[i].until_ms = until;
      g_i2c_ignored[i].active = true;
      DBG("I2C ignore MAC %02X:%02X:%02X:%02X:%02X:%02X for %lus",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (unsigned long)(I2C_IGNORE_MS / 1000UL));
      return;
    }
  }
}

static void queueBtConnectPending(const esp_bd_addr_t mac) {
  memcpy(g_pending_mac, mac, ESP_BD_ADDR_LEN);
  g_has_pending = true;
  g_i2c_bt_pending = true;
  g_gap_connect_request = false;
  DBG("I2C BT pending %02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void executeBtConnectPending() {
  if (!g_has_pending) {
    return;
  }
  I2C_BT_PENDING_CLEAR();
  setPairingSubState(PSS_CONNECT_NEW);
  DBG("I2C BT connect allowed -> start");
  a2dp.connect_to(g_pending_mac);
}

static void denyBtConnectPending(const uint8_t mac[6]) {
  esp_bd_addr_t tmp;
  memcpy(tmp, mac, ESP_BD_ADDR_LEN);
  i2cAddIgnoredMac(tmp);
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  if (g_state == ST_CONNECTING) {
    disconnectNow();
    if (g_has_last_mac) {
      startReconnectWait(false);
    } else {
      enterIdle();
    }
    return;
  }
  if (g_state == ST_PAIRING) {
    setPairingSubState(PSS_SCAN);
  }
}

uint8_t i2cPortGetStatus() {
  uint8_t st = I2C_STATUS_READY;
  if (g_pcm_playback_active || g_audio_enable) {
    st |= I2C_STATUS_PLAYING;
  }
  if (g_state == ST_CONNECT) {
    st |= I2C_STATUS_BT_CONNECTED;
  }
  if (g_has_pending && g_i2c_bt_pending) {
    st |= I2C_STATUS_BT_CONNECT_PENDING;
  }
  uint32_t rbUsed = 0;
  portENTER_CRITICAL(&g_rb_mux);
  rbUsed = rb_used_nolock(g_rb_w, g_rb_r);
  portEXIT_CRITICAL(&g_rb_mux);
  if (rbUsed > 0) {
    st |= I2C_STATUS_BUFFER_ACTIVE;
  }
  if (g_state == ST_ERROR || g_state == ST_TIMEOUT) {
    st |= I2C_STATUS_ERROR;
  }
  if (g_i2c_slave_output_mode == I2C_MODE_MP3) {
    st |= I2C_STATUS_MODE_MP3;
  }
  return st;
}

void i2cPortGetConnectedTarget(uint8_t* connectedFlag, uint8_t mac[6]) {
  i2cZeroMac(mac);
  if (g_state == ST_CONNECT && g_connected_peer_valid) {
    *connectedFlag = 0x01;
    memcpy(mac, g_connected_peer_mac, 6);
    return;
  }
  *connectedFlag = 0x00;
}

void i2cPortGetPendingTarget(uint8_t* pendingFlag, uint8_t mac[6]) {
  i2cZeroMac(mac);
  if (g_has_pending && g_i2c_bt_pending) {
    *pendingFlag = 0x01;
    memcpy(mac, g_pending_mac, 6);
    return;
  }
  *pendingFlag = 0x00;
}

uint8_t i2cPortSetMode(uint8_t mode) {
  if (mode != I2C_MODE_WAV_PCM && mode != I2C_MODE_MP3) {
    return I2C_ACK_INVALID_PARAM;
  }
  g_i2c_slave_output_mode = mode;
  return I2C_ACK_OK;
}

uint8_t i2cPortNotifyConnectPermission(uint8_t result, const uint8_t mac[6]) {
  if (!g_has_pending || !g_i2c_bt_pending) {
    return I2C_ACK_DENIED;
  }
  if (memcmp(g_pending_mac, mac, 6) != 0) {
    return I2C_ACK_INVALID_PARAM;
  }
  if (result == 0x01) {
    executeBtConnectPending();
    return I2C_ACK_OK;
  }
  denyBtConnectPending(mac);
  return I2C_ACK_OK;
}

uint8_t i2cPortClearBuffer(uint8_t bufferType) {
  switch (bufferType) {
    case 0x00:
      resetAudioPipeline(true);
      resetUzcSlotAssembler();
      break;
    case 0x01:
      portENTER_CRITICAL(&g_rb_mux);
      g_rb_w = 0;
      g_rb_r = 0;
      portEXIT_CRITICAL(&g_rb_mux);
      break;
    case 0x02:
      resetUzcSlotAssembler();
      break;
    case 0x03:
      pcmHoldReset();
      break;
    default:
      return I2C_ACK_INVALID_PARAM;
  }
  return I2C_ACK_OK;
}

uint8_t i2cPortSetDelay(int16_t delayMs) {
  if (delayMs < -1000 || delayMs > 1000) {
    return I2C_ACK_INVALID_PARAM;
  }
  g_i2c_delay_ms = delayMs;
  return I2C_ACK_OK;
}

uint8_t i2cPortSoftwareReset(uint8_t resetCode) {
  if (resetCode != 0xA5) {
    return I2C_ACK_INVALID_PARAM;
  }
  return I2C_ACK_OK;
}

#endif  // UZU_ENABLE_I2C

// ============================================================
// connect / pairing control
// ============================================================
static void disconnectNow() {
  DBG("disconnectNow()");
  g_audio_enable = false;
  a2dp.clean_last_connection();
  a2dp.set_connected(false);
}

static void showIdleAckAndOff() {
  setRedLed(true);
  setBlueLed(true);
  delay(250);
  setAllLedOff();
}

static void enterIdle() {
  stopGapDiscovery();
  g_gap_connect_request = false;
  g_gap_require_lastmac = false;
  g_connecting_to_known = false;
  setDiscoverableConnectable(false);
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
  setPairingSubState(PSS_SCAN);
  setState(ST_IDLE);
}

static void enterError(const char* reason) {
  DBG("ERROR: %s", reason ? reason : "(null)");
  stopGapDiscovery();
  g_gap_connect_request = false;
  g_gap_require_lastmac = false;
  g_connecting_to_known = false;
  setDiscoverableConnectable(false);
  disconnectNow();
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
  setPairingSubState(PSS_SCAN);
  setState(ST_ERROR);
}

static void enterTimeout() {
  DBG("enter TIMEOUT (red on 3s)");
  stopGapDiscovery();
  g_gap_connect_request = false;
  g_gap_require_lastmac = false;
  g_connecting_to_known = false;
  setDiscoverableConnectable(false);
  disconnectNow();
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
  setPairingSubState(PSS_SCAN);
  setState(ST_TIMEOUT);
}

static void startConnectToKnown(const esp_bd_addr_t mac) {
  stopGapDiscovery();
  g_gap_connect_request = false;
  g_connecting_to_known = true;
  g_media_start_pending = false;
  setDiscoverableConnectable(false);
  g_audio_enable = false;
#if UZU_ENABLE_I2C
  DBG("queue KNOWN %02X:%02X:%02X:%02X:%02X:%02X (await Master I2C)",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  queueBtConnectPending(mac);
#else
  g_has_pending = false;
  DBG("connect_to KNOWN %02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  esp_bd_addr_t tmp = {0};
  memcpy(tmp, mac, 6);
  a2dp.connect_to(tmp);
#endif
}

static void startConnectToNewPending() {
#if UZU_ENABLE_I2C
  executeBtConnectPending();
#else
  stopGapDiscovery();
  g_gap_connect_request = false;
  setDiscoverableConnectable(false);
  if (!g_has_pending) {
    DBG("startConnectToNewPending() skipped: no pending mac");
    return;
  }
  g_audio_enable = false;
  setPairingSubState(PSS_CONNECT_NEW);
  DBG("connect_to NEW %02X:%02X:%02X:%02X:%02X:%02X",
      g_pending_mac[0], g_pending_mac[1], g_pending_mac[2],
      g_pending_mac[3], g_pending_mac[4], g_pending_mac[5]);
  a2dp.connect_to(g_pending_mac);
#endif
}

static void startPairing() {
  DBG("startPairing()");
  disconnectNow();
  clearGapDevices();
  g_gap_connect_request = false;
  g_gap_require_lastmac = false;
  g_connecting_to_known = false;
  setDiscoverableConnectable(true);
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
  g_gap_last_stop_ms = millis();
  setPairingSubState(PSS_SCAN);
  setState(ST_PAIRING);
}

static void startPairingForLastMac() {
  DBG("startPairingForLastMac()");
  disconnectNow();
  clearGapDevices();
  g_gap_connect_request = false;
  g_gap_require_lastmac = g_has_last_mac;
  setDiscoverableConnectable(true);
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
  g_gap_last_stop_ms = millis();
  setPairingSubState(PSS_SCAN);
  setState(ST_PAIRING);
}

static void startReconnectWait(bool resetTimerAndCount) {
  if (!g_has_last_mac) {
    DBG("startReconnectWait() skipped: no stored device");
    enterIdle();
    return;
  }

  if (resetTimerAndCount) {
    g_reconnect_started_ms = millis();
    g_reconnect_try_count = 0;
  }

  g_audio_enable = false;
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  g_reconnect_active = true;
  setPairingSubState(PSS_SCAN);
  setState(ST_RECONNECT_WAIT);
}

static void startConnectOrPairingFromIdle() {
  if (g_has_last_mac) {
    DBG("short press in IDLE -> CONNECTING (stored device exists)");
    g_reconnect_active = false;
    startConnectToKnown(g_last_mac);
    setState(ST_CONNECTING);
  } else {
    DBG("short press in IDLE -> PAIRING (no stored device)");
    startPairing();
  }
}

static void enterErase() {
  DBG("enter ERASE");
  stopGapDiscovery();
  g_gap_connect_request = false;
  setDiscoverableConnectable(false);
  disconnectNow();
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  setPairingSubState(PSS_SCAN);
  setState(ST_ERASE);
}

static void processErase() {
  DBG("erase: clear bonds + lastmac");
  clearBondedDevices();
  clearLastMac();

  g_has_last_mac = false;
  memset(g_last_mac, 0, sizeof(g_last_mac));
  g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
  memset(g_pending_mac, 0, sizeof(g_pending_mac));
  clearGapDevices();
  g_gap_require_lastmac = false;
  g_connecting_to_known = false;

  enterIdle();
}

// ============================================================
// GAP discovery helpers
// ============================================================
static bool hasCodBit21(uint32_t cod) {
  return ((cod >> 21) & 0x01) != 0;
}

static bool isTargetCandidate(uint32_t cod, int rssi) {
  if (!esp_bt_gap_is_valid_cod(cod)) return false;
  if (esp_bt_gap_get_cod_major_dev(cod) != 0x04) return false;
  if (!hasCodBit21(cod)) return false;
  if (rssi <= RSSI_THRESHOLD_DBM) return false;
  return true;
}

static void clearGapBest() {
  memset(&g_gap_best, 0, sizeof(g_gap_best));
  g_gap_best.valid = false;
  g_gap_best.rssi = -127;
  g_gap_best.table_index = -1;
}

static void clearGapDevices() {
  memset(g_disc_devices, 0, sizeof(g_disc_devices));
  clearGapBest();
  g_gap_raw_events = 0;
}

static int findGapDeviceIndexByBda(const esp_bd_addr_t bda) {
  for (int i = 0; i < MAX_DISC_DEVICES; i++) {
    if (g_disc_devices[i].used && macEquals(g_disc_devices[i].bda, bda)) {
      return i;
    }
  }
  return -1;
}

static int allocGapDeviceIndex() {
  for (int i = 0; i < MAX_DISC_DEVICES; i++) {
    if (!g_disc_devices[i].used) {
      g_disc_devices[i].used = true;
      return i;
    }
  }
  return -1;
}

static void updateGapBestFromEntry(int idx) {
  if (idx < 0 || idx >= MAX_DISC_DEVICES) return;
  if (!g_disc_devices[idx].used) return;
  if (!g_disc_devices[idx].has_cod_rssi) return;
  if (!isTargetCandidate(g_disc_devices[idx].cod, g_disc_devices[idx].rssi_best)) return;
#if UZU_ENABLE_I2C
  if (isMacIgnored(g_disc_devices[idx].bda)) return;
#endif
  if (g_gap_require_lastmac && g_has_last_mac && !macEquals(g_disc_devices[idx].bda, g_last_mac)) return;

  if (!g_gap_best.valid || g_disc_devices[idx].rssi_best > g_gap_best.rssi) {
    g_gap_best.valid = true;
    memcpy(g_gap_best.bda, g_disc_devices[idx].bda, ESP_BD_ADDR_LEN);
    strncpy(g_gap_best.name,
            g_disc_devices[idx].name[0] ? g_disc_devices[idx].name : "",
            sizeof(g_gap_best.name) - 1);
    g_gap_best.name[sizeof(g_gap_best.name) - 1] = '\0';
    g_gap_best.rssi = g_disc_devices[idx].rssi_best;
    g_gap_best.cod = g_disc_devices[idx].cod;
    g_gap_best.table_index = idx;
  }
}

static void registerOrUpdateGapDevice(const esp_bd_addr_t bda,
                                      const char *name,
                                      bool has_cod_rssi,
                                      int rssi,
                                      uint32_t cod) {
  int idx = findGapDeviceIndexByBda(bda);
  bool is_new = false;
  if (idx < 0) {
    idx = allocGapDeviceIndex();
    if (idx < 0) {
      DBG("GAP device table full");
      return;
    }
    is_new = true;
    memcpy(g_disc_devices[idx].bda, bda, ESP_BD_ADDR_LEN);
    g_disc_devices[idx].name[0] = '\0';
    g_disc_devices[idx].rssi_last = -127;
    g_disc_devices[idx].rssi_best = -127;
    g_disc_devices[idx].cod = 0;
    g_disc_devices[idx].has_cod_rssi = false;
    g_disc_devices[idx].seen_count = 0;
  }

  g_disc_devices[idx].seen_count++;

  if (name && *name) {
    strncpy(g_disc_devices[idx].name, name, sizeof(g_disc_devices[idx].name) - 1);
    g_disc_devices[idx].name[sizeof(g_disc_devices[idx].name) - 1] = '\0';
  }

  if (has_cod_rssi) {
    g_disc_devices[idx].has_cod_rssi = true;
    g_disc_devices[idx].rssi_last = rssi;
    if (rssi > g_disc_devices[idx].rssi_best) {
      g_disc_devices[idx].rssi_best = rssi;
    }
    g_disc_devices[idx].cod = cod;
  }

  updateGapBestFromEntry(idx);

  if (is_new) {
    DBG("GAP NEW: %s %02X:%02X:%02X:%02X:%02X:%02X %s%s",
        g_disc_devices[idx].name[0] ? g_disc_devices[idx].name : "(no name)",
        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
        has_cod_rssi ? "rssi=" : "",
        has_cod_rssi ? String(rssi).c_str() : "(no cod/rssi)");
  }
}

static bool extractNameFromDiscRes(const esp_bt_gap_cb_param_t::disc_res_param &disc_res,
                                   char *out,
                                   size_t out_size) {
  if (out_size == 0) return false;
  out[0] = '\0';

  for (int i = 0; i < disc_res.num_prop; i++) {
    const esp_bt_gap_dev_prop_t *p = &disc_res.prop[i];
    if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
      size_t n = min((size_t)p->len, out_size - 1);
      memcpy(out, p->val, n);
      out[n] = '\0';
      return true;
    }
    if (p->type == ESP_BT_GAP_DEV_PROP_EIR) {
      uint8_t len = 0;
      uint8_t *eir = (uint8_t*)p->val;
      uint8_t *name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
      if (!name) {
        name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
      }
      if (name && len > 0) {
        size_t n = min((size_t)len, out_size - 1);
        memcpy(out, name, n);
        out[n] = '\0';
        return true;
      }
    }
  }
  return false;
}

static bool extractCodRssiFromDiscRes(const esp_bt_gap_cb_param_t::disc_res_param &disc_res,
                                      uint32_t &cod_out,
                                      int &rssi_out) {
  bool cod_ok = false;
  bool rssi_ok = false;
  cod_out = 0;
  rssi_out = -127;

  for (int i = 0; i < disc_res.num_prop; i++) {
    const esp_bt_gap_dev_prop_t *p = &disc_res.prop[i];
    if (p->type == ESP_BT_GAP_DEV_PROP_COD && p->len >= 4) {
      cod_out = *(uint32_t*)p->val;
      cod_ok = true;
    } else if (p->type == ESP_BT_GAP_DEV_PROP_RSSI && p->len >= 1) {
      rssi_out = *(int8_t*)p->val;
      rssi_ok = true;
    }
  }
  return cod_ok && rssi_ok;
}

static void dumpGapSummary() {
  DBG("GAP summary: raw_events=%lu mode=%s", (unsigned long)g_gap_raw_events, g_gap_require_lastmac ? "LASTMAC" : "ANY");
  int used_count = 0;
  for (int i = 0; i < MAX_DISC_DEVICES; i++) {
    if (!g_disc_devices[i].used) continue;
    used_count++;
    DBG("  GAP[%d] %s %02X:%02X:%02X:%02X:%02X:%02X seen=%lu best=%d last=%d cod=0x%06lX %s",
        i,
        g_disc_devices[i].name[0] ? g_disc_devices[i].name : "(no name)",
        g_disc_devices[i].bda[0], g_disc_devices[i].bda[1], g_disc_devices[i].bda[2],
        g_disc_devices[i].bda[3], g_disc_devices[i].bda[4], g_disc_devices[i].bda[5],
        (unsigned long)g_disc_devices[i].seen_count,
        g_disc_devices[i].rssi_best,
        g_disc_devices[i].rssi_last,
        (unsigned long)(g_disc_devices[i].cod & 0xFFFFFF),
        (g_disc_devices[i].has_cod_rssi && isTargetCandidate(g_disc_devices[i].cod, g_disc_devices[i].rssi_best)) ? "ACCEPT" : "IGNORE");
  }
  DBG("GAP unique devices=%d", used_count);
  if (g_gap_best.valid) {
    DBG("GAP BEST idx=%d %s %02X:%02X:%02X:%02X:%02X:%02X rssi=%d cod=0x%06lX",
        g_gap_best.table_index,
        g_gap_best.name[0] ? g_gap_best.name : "(no name)",
        g_gap_best.bda[0], g_gap_best.bda[1], g_gap_best.bda[2],
        g_gap_best.bda[3], g_gap_best.bda[4], g_gap_best.bda[5],
        g_gap_best.rssi,
        (unsigned long)(g_gap_best.cod & 0xFFFFFF));
  } else {
    DBG("GAP BEST none");
  }
}

static void stopGapDiscovery() {
  esp_bt_gap_cancel_discovery();
  g_gap_discovery_running = false;
}

static void startGapDiscovery() {
  stopGapDiscovery();
  clearGapDevices();
  g_gap_round++;
  DBG("GAP start round=%lu", (unsigned long)g_gap_round);
  esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                             DISCOVERY_SECONDS,
                                             DISCOVERY_MAX_RESP);
  DBG("esp_bt_gap_start_discovery -> %d", (int)err);
  if (err == ESP_OK) {
    g_gap_discovery_running = true;
  } else {
    g_gap_last_stop_ms = millis();
  }
}

static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
      g_gap_raw_events++;
      if (g_state != ST_PAIRING) break;
      char name[64] = "";
      uint32_t cod = 0;
      int rssi = -127;
      bool has_name = extractNameFromDiscRes(param->disc_res, name, sizeof(name));
      bool has_cod_rssi = extractCodRssiFromDiscRes(param->disc_res, cod, rssi);
      if (!has_name) name[0] = '\0';
      registerOrUpdateGapDevice(param->disc_res.bda, name, has_cod_rssi, rssi, cod);
      break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        g_gap_discovery_running = false;
        g_gap_last_stop_ms = millis();
        if (g_state == ST_PAIRING) {
          dumpGapSummary();
          if (g_pairing_sub == PSS_SCAN && !g_has_pending && g_gap_best.valid) {
#if UZU_ENABLE_I2C
            if (!isMacIgnored(g_gap_best.bda)) {
              queueBtConnectPending(g_gap_best.bda);
              DBG("GAP pending BEST (await Master I2C)");
            } else {
              DBG("GAP BEST ignored by I2C deny list");
            }
#else
            memcpy(g_pending_mac, g_gap_best.bda, ESP_BD_ADDR_LEN);
            g_has_pending = true;
            g_gap_connect_request = true;
            DBG("GAP queue connect to BEST");
#endif
          }
        }
      } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        g_gap_discovery_running = true;
      }
      break;
    }
    default:
      break;
  }
}

// ============================================================
// Discovery callback (unused in Step1 GAP version)
// ============================================================
static bool ssid_cb(const char *ssid, esp_bd_addr_t address, int rssi) {
  (void)ssid; (void)address; (void)rssi;
  return false;
}

// ============================================================
// A2DP callbacks
// ============================================================
static void connection_state_callback(esp_a2d_connection_state_t state, void *ptr) {
  (void)ptr;
  g_conn_state = state;
  DBG("A2DP conn_state=%d state=%s", (int)state, stateName((BtState)g_state));

  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    stopGapDiscovery();
    g_gap_connect_request = false;
    g_audio_enable = true;
    resetAudioPipeline(false);
    resetUzcSlotAssembler();
    uzcClearDecodeNotifications();
    drainI2sRx();

    if (g_state == ST_PAIRING && g_has_pending) {
      saveLastMac(g_pending_mac);
      memcpy(g_last_mac, g_pending_mac, 6);
      g_has_last_mac = true;
      memcpy(g_connected_peer_mac, g_pending_mac, 6);
      g_connected_peer_valid = true;
      DBG("PAIR saved lastmac");
    } else if (g_has_last_mac) {
      memcpy(g_connected_peer_mac, g_last_mac, 6);
      g_connected_peer_valid = true;
    }

    g_has_pending = false;
    I2C_BT_PENDING_CLEAR();
    g_reconnect_active = false;
    g_reconnect_try_count = 0;
    g_reconnect_started_ms = 0;
    g_gap_require_lastmac = false;
    g_connecting_to_known = false;
    setPairingSubState(PSS_SCAN);
    setDiscoverableConnectable(false);
    setState(ST_CONNECT);

    // MEDIA START は PCM プリバッファ完了時 (pcmPlaybackBegin) に行う
    g_media_start_pending = false;
    return;
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    g_audio_enable = false;
    g_media_start_pending = false;
    g_connected_peer_valid = false;

    if (g_state == ST_CONNECT) {
      DBG("remote disconnected during CONNECT");
      if (g_has_last_mac) {
        startReconnectWait(true);
      } else {
        enterIdle();
      }
      return;
    }

    if (g_state == ST_CONNECTING) {
      if (g_reconnect_active) {
        DBG("disconnect callback while CONNECTING (reconnect mode -> RECONNECT_WAIT)");
        disconnectNow();
        startReconnectWait(false);
      } else if (g_connecting_to_known && g_has_last_mac) {
        DBG("disconnect callback while CONNECTING known device (wait for timeout -> GAP fallback)");
      } else {
        DBG("disconnect callback while CONNECTING (keep blue slow blink / wait for timeout)");
      }
      return;
    }

    if (g_state == ST_PAIRING) {
      if (g_pairing_sub == PSS_CONNECT_NEW) {
        DBG("new device connect failed -> back to PAIRING scan");
        g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
        setPairingSubState(PSS_SCAN);
      } else {
        DBG("disconnect callback while PAIRING scan");
      }
      return;
    }

  }
}

static void audio_state_callback(esp_a2d_audio_state_t state, void *ptr) {
  (void)ptr;
  g_audio_state = state;
  DBG("A2DP audio_state=%d", (int)state);
  if (state == ESP_A2D_AUDIO_STATE_SUSPEND || state == ESP_A2D_AUDIO_STATE_STOPPED) {
    g_media_ctrl_started = false;
    return;
  }
  if (state == ESP_A2D_AUDIO_STATE_STARTED && g_state == ST_CONNECT && g_audio_enable) {
    uint32_t holdUsed = 0;
    portENTER_CRITICAL(&g_rb_mux);
    holdUsed = pcmHoldUsedNolock();
    portEXIT_CRITICAL(&g_rb_mux);
    tryBeginPlaybackWhenReady(holdUsed);
  }
}

// ============================================================
// Button handling
// ============================================================
static void handleShortPress() {
  DBG("BUTTON short state=%s", stateName((BtState)g_state));
  if (g_state == ST_IDLE) {
    startConnectOrPairingFromIdle();
  } else if (g_state == ST_PAIRING) {
    DBG("short press in PAIRING -> IDLE");
    disconnectNow();
    enterIdle();
  } else if (g_state == ST_CONNECTING || g_state == ST_RECONNECT_WAIT || g_state == ST_RECONNECT_SCAN) {
    DBG("short press during reconnect/connect -> force PAIRING");
    disconnectNow();
    startPairing();
  }
}

static void handleLongPress() {
  DBG("BUTTON long state=%s", stateName((BtState)g_state));
  if (g_state != ST_ERASE) {
    stopGapDiscovery();
    g_gap_connect_request = false;
    disconnectNow();
    showIdleAckAndOff();
    enterIdle();
  }
}

static void handleVeryLongPress() {
  DBG("BUTTON very long state=%s", stateName((BtState)g_state));
  enterErase();
}

#if UZU_I2S_TEST_BYPASS_BT
static void pollSerialTestCommands() {
  static String line;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line.trim();
      if (line.equalsIgnoreCase(F("BT GO")) && g_state == ST_IDLE) {
        Serial.println(F("OK BT GO"));
        startConnectOrPairingFromIdle();
      } else if (line.length() > 0) {
        Serial.print(F("# unknown: "));
        Serial.println(line);
      }
      line = "";
      continue;
    }
    if (line.length() < 32) {
      line += c;
    }
  }
}
#endif

static void pollButton() {
  static bool lastPressed = false;
  static uint32_t tDown = 0;
  static bool firedLong = false;
  static bool firedVeryLong = false;

  bool pressed = isButtonPressed();

  if (!lastPressed && pressed) {
    tDown = millis();
    firedLong = false;
    firedVeryLong = false;
    DBG("BUTTON down");
  }
  else if (lastPressed && pressed) {
    uint32_t dt = millis() - tDown;

    if (!firedVeryLong && dt >= VERY_LONG_PRESS_MS) {
      firedVeryLong = true;
      handleVeryLongPress();
    }
    else if (!firedLong && dt >= LONG_PRESS_MS) {
      firedLong = true;
      handleLongPress();
    }
  }
  else if (lastPressed && !pressed) {
    uint32_t dt = millis() - tDown;
    DBG("BUTTON up dt=%lu", (unsigned long)dt);
    if (!firedLong && !firedVeryLong) {
      handleShortPress();
    }
  }

  lastPressed = pressed;
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  applyLowPowerPresetBeforeBtInit();

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== sintest_i2s_mp3 Slave start ==="));
  Serial.print(F("sintest_i2s_mp3 Slave ready  build "));
  Serial.println(BUILD_NUMBER);
  Serial.println(VERSION_STRING);
#if UZU_I2S_TEST_BYPASS_BT
  Serial.println(F("[I2S] test bypass BT enabled"));
#endif
  Serial.println(F("I2S RX: 44100Hz L=data R=AAAA/AA00/AA55/5500/FFFF (pad=Slave skip)"));

  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  setAllLedOff();
  pinMode(PIN_BUTTON, INPUT_PULLUP);

#if UZU_ENABLE_I2C
  i2cSlaveBegin();
  if (DEVICE_CHANNEL_NO >= 1 && DEVICE_CHANNEL_NO <= 5 &&
      (uint8_t)DEVICE_CHANNEL_NO != (uint8_t)(i2cSlaveSelValue() + 1)) {
    Serial.printf("[I2C] WARN: DEVICE_CHANNEL_NO=%d but SEL=%u (CH%u)\n",
                  DEVICE_CHANNEL_NO, (unsigned)i2cSlaveSelValue(),
                  (unsigned)(i2cSlaveSelValue() + 1));
  }
#else
  Serial.println(F("[I2C] disabled — Sub controls BT locally"));
#endif

  if (!i2s_rx_init_slave_stereo_16()) {
    enterError("I2S init failed");
  }

  if (xTaskCreate(uzcDecodeTask, "uzcMp3", 8192, nullptr, 3, &g_uzc_decode_task) != pdPASS) {
    DBG("UZC decode task create failed");
    g_uzc_decode_task = nullptr;
  }

  resetAudioPipeline(true);
  resetUzcSlotAssembler();

  g_has_last_mac = loadLastMac(g_last_mac);
  if (g_has_last_mac) {
    DBG("BOOT lastmac=%02X:%02X:%02X:%02X:%02X:%02X",
        g_last_mac[0], g_last_mac[1], g_last_mac[2],
        g_last_mac[3], g_last_mac[4], g_last_mac[5]);
  } else {
    DBG("BOOT no lastmac");
  }

  a2dp.set_on_connection_state_changed(connection_state_callback);
  a2dp.set_on_audio_state_changed(audio_state_callback);
  a2dp.set_data_callback_in_frames(audio_cb);
  a2dp.set_ssid_callback(ssid_cb);

  makeLocalBtName(g_local_bt_name, sizeof(g_local_bt_name));
  a2dp.set_local_name(g_local_bt_name);
  a2dp.set_auto_reconnect(false);

  DBG("A2DP start()");
  a2dp.start();
#if ESP_ARDUINO_VERSION_MAJOR >= 3 && __has_include("esp32-hal-bt.h")
  DBG("btStarted=%d", btStarted() ? 1 : 0);
  if (!btStarted()) {
    Serial.println(F("[ERROR] BT controller failed to start (see boot log above)"));
  }
#endif
  DBG("local BT name = %s", g_local_bt_name);

  enterIdle();
}

void loop() {
  if (g_state == ST_CONNECT && g_audio_enable) {
    for (int pass = 0; pass < 3; pass++) {
      serviceI2SInput();
    }
  } else {
    serviceI2SInput();
  }
#if UZU_I2S_TEST_BYPASS_BT
  pollSerialTestCommands();
  if (g_i2s_bench_playback) {
    i2sBenchDrainPcm();
  } else if (g_state == ST_CONNECT && g_audio_enable && g_pcm_playback_active) {
    servicePcmHoldToRing();
  }
#else
  if (g_state == ST_CONNECT && g_audio_enable && g_pcm_playback_active) {
    servicePcmHoldToRing();
  }
#endif
#if UZU_ENABLE_I2C
  i2cSlavePoll();
#endif

  pollButton();
  updateLed();

#if !UZU_ENABLE_I2C
  if (g_gap_connect_request && g_state == ST_PAIRING && g_has_pending && g_pairing_sub == PSS_SCAN) {
    DBG("GAP connect request -> connect pending BEST");
    startConnectToNewPending();
  }
#endif

  if (g_state == ST_PAIRING) {
    uint32_t now = millis();
    if (g_pairing_sub == PSS_SCAN && !g_gap_discovery_running && !g_gap_connect_request && !g_has_pending) {
      if ((now - g_gap_last_stop_ms) >= DISCOVERY_RESTART_DELAY_MS) {
        startGapDiscovery();
      }
    }

    if (now - g_state_started_ms >= TIMEOUT_PAIRING_MS) {
      DBG("PAIRING timeout -> TIMEOUT");
      g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
      setPairingSubState(PSS_SCAN);
      enterTimeout();
    }
  }

  if (g_media_start_pending && g_state == ST_CONNECT && millis() >= g_media_start_due_ms) {
    g_media_start_pending = false;
    startMedia();
  }

  if (g_state == ST_RECONNECT_WAIT) {
    uint32_t now = millis();

    if ((now - g_reconnect_started_ms) >= TIMEOUT_PAIRING_MS) {
      DBG("RECONNECT total timeout -> GAP fallback search for lastmac");
      startPairingForLastMac();
    } else {
      uint32_t wait_ms = (g_reconnect_try_count == 0) ? RECONNECT_WAIT_FIRST_MS : RECONNECT_WAIT_RETRY_MS;
      if ((now - g_state_started_ms) >= wait_ms) {
        if (g_reconnect_try_count >= RECONNECT_MAX_TRIES) {
          DBG("RECONNECT max tries -> GAP fallback search for lastmac");
          startPairingForLastMac();
        } else {
          startReconnectAttempt();
        }
      }
    }
  }

  if (g_state == ST_CONNECTING) {
    if (millis() - g_state_started_ms >= TIMEOUT_CONNECTING_MS) {
      if (g_reconnect_active) {
        DBG("CONNECTING timeout in reconnect mode -> RECONNECT_WAIT");
        disconnectNow();
        startReconnectWait(false);
      } else if (g_connecting_to_known && g_has_last_mac) {
        DBG("CONNECTING timeout for KNOWN -> GAP fallback search for lastmac");
        g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
        startPairingForLastMac();
      } else {
        DBG("CONNECTING timeout -> TIMEOUT");
        g_has_pending = false;
  I2C_BT_PENDING_CLEAR();
  g_connected_peer_valid = false;
        enterTimeout();
      }
    }
  }

  if (g_state == ST_TIMEOUT) {
    if (millis() - g_state_started_ms >= TIMEOUT_ERROR_SHOW_MS) {
      DBG("TIMEOUT display done -> IDLE");
      enterIdle();
    }
  }

  if (g_state == ST_ERASE) {
    // Enter once, blink red fast briefly so the user can notice it.
    if (millis() - g_state_started_ms >= 300) {
      processErase();
    }
  }

  delay(1);
}

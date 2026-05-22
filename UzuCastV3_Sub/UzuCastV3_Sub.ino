/*
============================================================
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

#include <Arduino.h>
#include <Preferences.h>
#include "BluetoothA2DPSource.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION_MAJOR != 2
  #error "This project requires ESP32 Arduino Core 2.x."
#endif

// ============================================================
// Debug
// ============================================================
#define DEBUG_BT_MODULE 1

#if DEBUG_BT_MODULE
  #define DBG(fmt, ...) Serial.printf("[DBG] " fmt "\n", ##__VA_ARGS__)
#else
  #define DBG(fmt, ...)
#endif

// ============================================================
// GPIO
// ============================================================
static const int PIN_BUTTON   = 4;
static const int PIN_LED_BLUE = 25;
static const int PIN_LED_RED  = 26;

// ============================================================
// Device identity / BT visibility
// ============================================================
static const int DEVICE_CHANNEL_NO = 1;   // change per board: 1,2,3...
static const char* DEVICE_NAME_PREFIX = "UZU";
static char g_local_bt_name[32] = {0};

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
  cfg.sample_rate = 48000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.dma_buf_count = 8;
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
// to the library and resample 48 kHz I2S input down to 44.1 kHz locally.
// We still log the negotiated rate reported by ESP_A2D_AUDIO_CFG_EVT when
// available, but playback processing stays fixed at 44.1 kHz.
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
};

static UzuBluetoothA2DPSource a2dp;
static volatile BtState g_state = ST_IDLE;

// ============================================================
// Ring Buffer (frames = L+R)
// ============================================================
static portMUX_TYPE g_rb_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint32_t RB_FRAMES = 8192;
static int16_t g_rb[RB_FRAMES * 2];
static volatile uint32_t g_rb_w = 0;
static volatile uint32_t g_rb_r = 0;

static volatile uint32_t g_rate_accum_frames = 0;
static volatile uint32_t g_rate_window_started_ms = 0;
static uint32_t g_resample_phase_q16 = 0;

static inline uint32_t rb_used_nolock(uint32_t w, uint32_t r) {
  return (w >= r) ? (w - r) : (RB_FRAMES - (r - w));
}

static inline uint32_t rb_free_nolock(uint32_t used) {
  return (RB_FRAMES - 1) - used;
}

static void resetAudioPipeline(bool clearRateWindow) {
  portENTER_CRITICAL(&g_rb_mux);
  g_rb_w = 0;
  g_rb_r = 0;
  portEXIT_CRITICAL(&g_rb_mux);
  g_resample_phase_q16 = 0;
  if (clearRateWindow) {
    g_rate_accum_frames = 0;
    g_rate_window_started_ms = millis();
  }
}

static void updateInputRateEstimate(uint32_t frames) {
  uint32_t now = millis();
  if (g_rate_window_started_ms == 0) {
    g_rate_window_started_ms = now;
  }
  g_rate_accum_frames += frames;
  uint32_t dt = now - g_rate_window_started_ms;
  if (dt < 200) return;

  uint32_t rate = (uint32_t)((1000ULL * g_rate_accum_frames + (dt / 2)) / dt);
  uint32_t prev = g_i2s_input_rate;
  uint32_t decided = prev;
  if (rate >= 46000) decided = 48000;
  else if (rate >= 42000) decided = 44100;

  if (decided != prev) {
    DBG("I2S input rate detected: raw=%lu -> %lu Hz", (unsigned long)rate, (unsigned long)decided);
    g_i2s_input_rate = decided;
    resetAudioPipeline(false);
  }

  g_rate_accum_frames = 0;
  g_rate_window_started_ms = now;
}

static void serviceI2SInput() {
  static int16_t tmp[256 * 2];
  size_t rbytes = 0;

  esp_err_t e = i2s_read(I2S_NUM_1, tmp, sizeof(tmp), &rbytes, 0);
  if (e != ESP_OK || rbytes == 0) return;

  uint32_t frames = (uint32_t)(rbytes / (sizeof(int16_t) * 2));
  if (frames == 0) return;

  updateInputRateEstimate(frames);

  if (!(g_state == ST_CONNECT || g_state == ST_CONNECTING || g_state == ST_PAIRING)) {
    return;
  }

  portENTER_CRITICAL(&g_rb_mux);
  uint32_t w = g_rb_w;
  uint32_t r = g_rb_r;
  uint32_t used = rb_used_nolock(w, r);
  uint32_t freef = rb_free_nolock(used);
  uint32_t push = (frames <= freef) ? frames : freef;

  for (uint32_t i = 0; i < push; i++) {
    uint32_t idx = (w % RB_FRAMES) * 2;
    g_rb[idx + 0] = tmp[i * 2 + 0];
    g_rb[idx + 1] = tmp[i * 2 + 1];
    w++;
  }
  g_rb_w = w;
  portEXIT_CRITICAL(&g_rb_mux);
}

static int16_t lerp16(int16_t a, int16_t b, uint16_t frac) {
  int32_t da = (int32_t)b - (int32_t)a;
  return (int16_t)((int32_t)a + ((da * (int32_t)frac) >> 16));
}

static int32_t audio_cb(Frame *frames, int32_t frame_count) {
  if (!g_audio_enable) {
    for (int i = 0; i < frame_count; i++) {
      frames[i].channel1 = 0;
      frames[i].channel2 = 0;
    }
    return frame_count;
  }

  // Bluetooth/A2DP side is intentionally fixed at 44.1 kHz.
  const bool direct_copy = (g_i2s_input_rate == 44100) || (g_i2s_input_rate == 0);

  portENTER_CRITICAL(&g_rb_mux);
  uint32_t w = g_rb_w;
  uint32_t r = g_rb_r;
  uint32_t avail = rb_used_nolock(w, r);

  if (direct_copy) {
    uint32_t pop = ((uint32_t)frame_count <= avail) ? (uint32_t)frame_count : avail;
    for (uint32_t i = 0; i < pop; i++) {
      uint32_t idx = (r % RB_FRAMES) * 2;
      frames[i].channel1 = g_rb[idx + 0];
      frames[i].channel2 = g_rb[idx + 1];
      r++;
    }
    g_rb_r = r;
    portEXIT_CRITICAL(&g_rb_mux);

    for (uint32_t i = pop; i < (uint32_t)frame_count; i++) {
      frames[i].channel1 = 0;
      frames[i].channel2 = 0;
    }
    return frame_count;
  }

  // Only 48 kHz -> 44.1 kHz conversion is expected/implemented here.
  const uint32_t step_q16 = (uint32_t)(((uint64_t)g_i2s_input_rate << 16) / 44100ULL);
  uint32_t phase = g_resample_phase_q16;
  uint32_t produced = 0;

  while (produced < (uint32_t)frame_count) {
    uint32_t base = phase >> 16;
    if ((base + 1) >= avail) break;

    uint16_t frac = (uint16_t)(phase & 0xFFFFu);
    uint32_t idx0 = ((r + base) % RB_FRAMES) * 2;
    uint32_t idx1 = ((r + base + 1) % RB_FRAMES) * 2;

    frames[produced].channel1 = lerp16(g_rb[idx0 + 0], g_rb[idx1 + 0], frac);
    frames[produced].channel2 = lerp16(g_rb[idx0 + 1], g_rb[idx1 + 1], frac);
    produced++;

    phase += step_q16;
    uint32_t advance = phase >> 16;
    phase &= 0xFFFFu;
    r += advance;
    avail -= advance;
  }

  g_rb_r = r;
  g_resample_phase_q16 = phase;
  portEXIT_CRITICAL(&g_rb_mux);

  for (uint32_t i = produced; i < (uint32_t)frame_count; i++) {
    frames[i].channel1 = 0;
    frames[i].channel2 = 0;
  }
  return frame_count;
}

static void startMedia() {
  esp_err_t r = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
  DBG("MEDIA START -> %d (input=%lu, bt_path=44100, negotiated=%lu)",
      (int)r,
      (unsigned long)g_i2s_input_rate,
      (unsigned long)g_a2dp_negotiated_rate);
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
  setDiscoverableConnectable(false);
  esp_bd_addr_t tmp = {0};
  memcpy(tmp, mac, 6);

  g_audio_enable = false;
  g_has_pending = false;
  DBG("connect_to KNOWN %02X:%02X:%02X:%02X:%02X:%02X",
      tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);

  a2dp.connect_to(tmp);
}

static void startConnectToNewPending() {
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
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
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
  g_media_start_pending = false;
  g_reconnect_active = false;
  g_reconnect_try_count = 0;
  g_reconnect_started_ms = 0;
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
  if (g_gap_discovery_running) {
    esp_bt_gap_cancel_discovery();
  }
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
            memcpy(g_pending_mac, g_gap_best.bda, ESP_BD_ADDR_LEN);
            g_has_pending = true;
            g_gap_connect_request = true;
            DBG("GAP queue connect to BEST");
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
    g_audio_enable = true;
    resetAudioPipeline(false);

    if (g_state == ST_PAIRING && g_has_pending) {
      saveLastMac(g_pending_mac);
      memcpy(g_last_mac, g_pending_mac, 6);
      g_has_last_mac = true;
      DBG("PAIR saved lastmac");
    }

    g_has_pending = false;
    g_reconnect_active = false;
    g_reconnect_try_count = 0;
    g_reconnect_started_ms = 0;
    g_gap_require_lastmac = false;
    g_connecting_to_known = false;
    setPairingSubState(PSS_SCAN);
    setDiscoverableConnectable(false);
    setState(ST_CONNECT);

    // Allow ESP-IDF / peer negotiation to settle before starting media.
    g_media_start_pending = true;
    g_media_start_due_ms = millis() + 300;
    return;
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    g_audio_enable = false;
    g_media_start_pending = false;

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
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== BTModule44_48K_StateRev3 start ===");

  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  setAllLedOff();
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  DBG("GPIO init done");

  if (!i2s_rx_init_slave_stereo_16()) {
    enterError("I2S init failed");
  }

  resetAudioPipeline(true);

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
  esp_err_t gapErr = esp_bt_gap_register_callback(gapCallback);
  DBG("esp_bt_gap_register_callback -> %d", (int)gapErr);
  esp_err_t nameErr = esp_bt_dev_set_device_name(g_local_bt_name);
  DBG("local BT name = %s (%d)", g_local_bt_name, (int)nameErr);

  enterIdle();
}

void loop() {
  serviceI2SInput();

  pollButton();
  updateLed();

  if (g_gap_connect_request && g_state == ST_PAIRING && g_has_pending && g_pairing_sub == PSS_SCAN) {
    DBG("GAP connect request -> connect pending BEST");
    startConnectToNewPending();
  }

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
        startPairingForLastMac();
      } else {
        DBG("CONNECTING timeout -> TIMEOUT");
        g_has_pending = false;
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

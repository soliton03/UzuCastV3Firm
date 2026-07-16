/*
Board:            ESP32S3 Dev Module
Flash Size:       16MB (128Mb)
PSRAM:            QSPI PSRAM
Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
CPU Frequency:    240MHz (WiFi)
Flash Mode:       QIO 80MHz
Upload Speed:     115200
Core Debug Level: None
USB Mode:         Hardware CDC and JTAG
USB CDC On Boot:  Disabled
Upload Mode:      UART0 / Hardware CDC
Arduino Runs On:  Core 1
Events Run On:    Core 1
*/
#include <Arduino.h>
#include "SD_MMC.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <WebSocketsServer.h>

#include "driver/i2s_tdm.h"
#include <esp_arduino_version.h>
#include <esp_heap_caps.h>
#include "i2c_mode.h"
#include "uzc_tdm.h"
#include "uzu_i2c_config.h"
#if UZU_ENABLE_I2C
#include "i2c_host.h"
#endif

#if ESP_ARDUINO_VERSION_MAJOR != 3
  #error "This project requires ESP32 Arduino Core 3.x (IDF5)."
#endif

#define VERSION_STRING   "UZU CAST Version 1.00"
#define FIRMWARE_BUILD   10045
#define FIRMWARE_BUILD_STR_HELPER(x) #x
#define FIRMWARE_BUILD_STR(x) FIRMWARE_BUILD_STR_HELPER(x)

// 1=起動時の電源ボタン長押し待ちをスキップ（デバッグ用）
#ifndef UZU_SKIP_POWER_BUTTON
#define UZU_SKIP_POWER_BUTTON 0
#endif
// ====================================================
// Forward declarations
// ====================================================
bool isCardInserted();

static void make_abs_path(const char* in, char* out, size_t out_len);
static void join_path(char* out, size_t out_len, const char* dir, const char* name);
static bool hasExtensionIgnoreCase(const char* name, const char* ext);

static uint32_t le32_from_buf(const uint8_t* p);
static int16_t le16_to_s16(const uint8_t* p);

static bool read_u32_at(File& f, uint32_t offset, uint32_t& value);
static bool read_string_at(File& f, uint32_t offset, uint32_t len, char* out, size_t outSize);
static bool check_uzu_magic(File& f);
static bool read_uzu_length_ms(const char* path, uint32_t& lengthMs);
struct WebTrackInfo;
static bool read_uzu_track_info(const char* path, WebTrackInfo& info);
static void stopAllPlayback();
static void wsSendTrackInfo(uint8_t clientId, int trackIndex0);
static void wsBroadcastStreamStatus();
static void wsBroadcastBufferInfo();
static void wsBroadcastUnderrun();
static void wsRequestStreamStatus();
static void wsRequestBufferInfo();
static void wsRequestUnderrun();
static void wsProcessNotifications();
static bool handleStreamTextCommand(uint8_t num, const String& msg);

static const char* getCardTypeString();
static bool mount_sdmmc_4bit(uint32_t freq_hz);
static void unmount_sdmmc();
static void mountAndScanSdCard(bool verbose);
static void processSdCardDetect();
static void clearFileList();
static int16_t apply_volume_127(int16_t sample, uint8_t vol);

static bool scanUzuFiles(const char* path);

static bool i2s_tdm_tx_init(uint32_t sampleRate);
static bool tdm_set_sample_rate(uint32_t sampleRate);
static bool tdm_enable_output();
static void tdm_disable_output();
static void i2sTestProcess();
static bool i2sTestStart(uint32_t seconds);
static void i2sTestStop();

static void wsSendTracks(uint8_t clientId);
static void wsBroadcastTracks();
static void wsSendState(uint8_t clientId);
static void wsBroadcastState();
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
static void json_append_escaped(String& s, const char* p);
static void performSystemInit(bool verbose);
static void shutdownNetworkServices();

// ====================================================
// Power control pins
// ====================================================
static const int POWER_PIN = 1;
static const int LED_PIN   = 2;
static const int SW_PIN    = 7;
static constexpr uint32_t POWER_LONGPRESS_MS = 3000;

// ====================================================
// SDMMC pins
// ====================================================
static const int PIN_SD_D2  = 14;
static const int PIN_SD_D3  = 13;
static const int PIN_SD_CMD = 12;
static const int PIN_SD_CLK = 11;
static const int PIN_SD_D0  = 10;
static const int PIN_SD_D1  = 9;
static const int PIN_SD_CD  = 21;   // LOW = inserted

static uint32_t g_sd_freq_hz = 20000000; // 20MHz
static bool g_mounted = false;
static bool g_sdCardPresent = false;
static bool g_sdRemountPending = false;
static uint32_t g_sdRemountAtMs = 0;
static uint32_t g_sdCdLastPollMs = 0;
static bool g_sdCdLastRead = false;
static uint32_t g_sdCdStableSinceMs = 0;

static constexpr uint32_t SD_CD_POLL_MS = 200;
static constexpr uint32_t SD_CD_DEBOUNCE_MS = 50;
static constexpr uint32_t SD_INSERT_SETTLE_MS = 300;

// ====================================================
// WiFi AP / DNS
// ====================================================
static const char* AP_SSID     = "UZU-CAST-HOST";
static const char* AP_PASSWORD = "12345678";
static const bool  AP_OPEN     = true;

static const IPAddress AP_IP  (192, 168, 4, 1);
static const IPAddress AP_GW  (192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);
static const byte DNS_PORT = 53;

static DNSServer dns;

// ====================================================
// TDM pins / format
// ====================================================
static constexpr gpio_num_t I2S_BCLK_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t I2S_WS_GPIO   = GPIO_NUM_16;
static constexpr gpio_num_t I2S_DOUT_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t I2S_MCLK_GPIO = I2S_GPIO_UNUSED;

static constexpr uint32_t TDM_DEFAULT_SAMPLE_RATE = 44100;
static constexpr uint32_t TDM_ALT_SAMPLE_RATE     = 48000;
static constexpr uint32_t TDM_OUT_SAMPLE_RATE     = TDM_DEFAULT_SAMPLE_RATE;
static constexpr uint32_t TDM_NUM_CH           = 8;
static constexpr uint32_t TDM_AUDIO_CH         = TDM_NUM_CH - 1;  // CH1-7 PCM, CH8=control tag
static constexpr uint32_t FRAMES_PER_WRITE     = 256;
static constexpr uint32_t BYTES_PER_TDM_FRAME  = TDM_NUM_CH * 2;
static constexpr int16_t I2S_TAG_RAW_TDM       = (int16_t)0xAAAA;

static constexpr i2s_tdm_slot_mask_t TDM_SLOT_MASK =
  (i2s_tdm_slot_mask_t)(
    I2S_TDM_SLOT0 | I2S_TDM_SLOT1 |
    I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
    I2S_TDM_SLOT4 | I2S_TDM_SLOT5 |
    I2S_TDM_SLOT6 | I2S_TDM_SLOT7
  );

static i2s_chan_handle_t g_tx_handle = nullptr;
static bool g_tdm_enabled = false;
static uint32_t g_tdm_current_rate = 0;

// ====================================================
// Player state
// ====================================================
enum class PlayerState {
  STOP = 0,
  PLAY,
  PAUSE,
  ERROR
};

// ====================================================
// File list cache
// ====================================================
struct FileEntry {
  char name[96];
  char path[160];
};

static constexpr int MAX_FILES = 128;
static FileEntry g_fileList[MAX_FILES];
static int g_fileCount = 0;
static int g_selectedTrack0 = 0;
static uint32_t g_selectedLenMs = 0;

struct WebTrackInfo {
  char name[96];
  char path[160];
  char title[257];
  uint32_t lengthMs = 0;
  uint32_t channels = 0;
  uint32_t sampleRate = 0;
  uint32_t bits = 0;
  uint32_t totalSamples = 0;
  char chName[8][257];
};

// ====================================================
// Global work buffers
// ====================================================
static uint8_t g_u32buf[4];
static char g_strbuf[257];
static constexpr uint32_t SRC_BUFFER_MAX_FRAMES =
    (uint32_t)((((uint64_t)FRAMES_PER_WRITE * TDM_ALT_SAMPLE_RATE) + (TDM_OUT_SAMPLE_RATE - 1)) /
               TDM_OUT_SAMPLE_RATE) +
    4;
static uint8_t g_pcmReadBuf[SRC_BUFFER_MAX_FRAMES * TDM_NUM_CH * 2];
static int16_t g_tdmTxBuf[FRAMES_PER_WRITE * TDM_NUM_CH];

static void tdmApplyRawTagCh8(uint32_t frameCount) {
  for (uint32_t f = 0; f < frameCount; ++f) {
    g_tdmTxBuf[f * TDM_NUM_CH + 7] = I2S_TAG_RAW_TDM;
  }
}

static uint8_t g_volume = 127;

// ====================================================
// Power manager
// ====================================================
// ====================================================
// Power manager
// ====================================================
class PowerManager {
public:
  enum class State {
    WAIT_ON_PRESS = 0,
    WAIT_ON_RELEASE,
    WAIT_STABLE_RELEASE_AFTER_ON,
    POWER_ON,
    WAIT_OFF_RELEASE,
    POWER_OFF
  };

  void begin() {
    pinMode(POWER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SW_PIN, INPUT_PULLDOWN);

    digitalWrite(POWER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

#if UZU_SKIP_POWER_BUTTON
    digitalWrite(POWER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    m_state = State::POWER_ON;
#else
    m_state = State::WAIT_ON_PRESS;
#endif
    m_pressStartMs = 0;
    m_releaseGuardStartMs = 0;
  }

  void waitUntilPowerOn() {
#if UZU_SKIP_POWER_BUTTON
    return;
#else
    while (m_state != State::POWER_ON &&
           m_state != State::WAIT_STABLE_RELEASE_AFTER_ON) {
      processBeforeOn();
      delay(1);
    }
#endif
  }

  void process() {
    switch (m_state) {
      case State::WAIT_STABLE_RELEASE_AFTER_ON:
        if (!isPressed()) {
          if (m_releaseGuardStartMs == 0) {
            m_releaseGuardStartMs = millis();
          } else if ((millis() - m_releaseGuardStartMs) >= RELEASE_GUARD_MS) {
            m_pressStartMs = 0;
            m_releaseGuardStartMs = 0;
            m_state = State::POWER_ON;
          }
        } else {
          m_releaseGuardStartMs = 0;
        }
        break;

      case State::POWER_ON:
        processPowerOn();
        break;

      case State::WAIT_OFF_RELEASE:
        if (!isPressed()) {
          digitalWrite(POWER_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          m_state = State::POWER_OFF;
        }
        break;

      case State::POWER_OFF:
        break;

      default:
        break;
    }
  }

  bool isPoweredOn() const {
    return m_state == State::POWER_ON;
  }

  bool isPoweredOff() const {
    return m_state == State::POWER_OFF;
  }

private:
  static constexpr uint32_t RELEASE_GUARD_MS = 50;

  State m_state = State::WAIT_ON_PRESS;
  uint32_t m_pressStartMs = 0;
  uint32_t m_releaseGuardStartMs = 0;

  bool isPressed() const {
    return digitalRead(SW_PIN) != 0;
  }

  void processBeforeOn() {
    switch (m_state) {
      case State::WAIT_ON_PRESS:
        if (isPressed()) {
          if (m_pressStartMs == 0) {
            m_pressStartMs = millis();
          } else if ((millis() - m_pressStartMs) >= POWER_LONGPRESS_MS) {
            // 長押し成立時点で自己保持を入れる
            digitalWrite(POWER_PIN, HIGH);
            digitalWrite(LED_PIN, HIGH);
            m_state = State::WAIT_ON_RELEASE;
          }
        } else {
          m_pressStartMs = 0;
        }
        break;

      case State::WAIT_ON_RELEASE:
        if (!isPressed()) {
          m_pressStartMs = 0;
          m_releaseGuardStartMs = 0;
          m_state = State::WAIT_STABLE_RELEASE_AFTER_ON;
        }
        break;

      default:
        break;
    }
  }

  void processPowerOn() {
    if (isPressed()) {
      if (m_pressStartMs == 0) {
        m_pressStartMs = millis();
      } else if ((millis() - m_pressStartMs) >= POWER_LONGPRESS_MS) {
        digitalWrite(LED_PIN, LOW);
        m_state = State::WAIT_OFF_RELEASE;
      }
    } else {
      m_pressStartMs = 0;
    }
  }
};

static PowerManager g_power;

// ====================================================
// HTML UI
// ====================================================
static const char kHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>UZU CAST HOST</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 16px; }
    h1 { margin: 0 0 12px 0; }
    .card { border: 1px solid #ccc; border-radius: 12px; padding: 12px; margin: 12px 0; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    button { padding: 10px 14px; border-radius: 10px; border: 1px solid #999; background: #f7f7f7; }
    select { padding: 10px; border-radius: 10px; border: 1px solid #999; min-width: 260px; }
    input[type="range"] { width: 100%; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    pre.mono { white-space: pre-wrap; margin: 8px 0 0 0; }
    .status { display: inline-block; padding: 6px 10px; border-radius: 999px; border: 1px solid #999; }
    select:disabled { opacity: 0.6; }
    input[type="file"] { max-width: 100%; }
    .hidden { display: none !important; }
  </style>
</head>
<body>
  <h1>UZU CAST HOST</h1>
  <div class="mono" id="fwBuild">Firmware Build: )HTML" FIRMWARE_BUILD_STR(FIRMWARE_BUILD) R"HTML(</div>

  <div class="card">
    <div class="row">
      <div class="status mono" id="wsState">WS: (connecting)</div>
    </div>
  </div>

  <div class="card">
    <h2>1) 曲の選択</h2>
    <div class="row">
      <label for="sourceModeSelect">ソース:</label>
      <select id="sourceModeSelect">
        <option value="SD">SDカード</option>
        <option value="FILE">ブラウザファイル</option>
      </select>
    </div>
    <div id="sdSourcePanel">
      <div class="row">
        <select id="trackSelect"></select>
      </div>
    </div>
    <div id="fileSourcePanel" class="hidden">
      <div class="row">
        <input id="fileInput" type="file" accept=".uzu,.UZU">
      </div>
    </div>
    <div class="mono" id="selectedTrack">Selected: -</div>
    <div class="row">
      <button id="btnToggleInfo">詳細表示</button>
    </div>
    <pre class="mono" id="trackInfo" style="display:none;">Info: -</pre>
  </div>

  <div class="card">
    <h2>2) 演奏</h2>
    <div class="row">
      <button id="btnPlay">再生</button>
      <button id="btnStop">停止</button>
      <button id="btnPause">ポーズ</button>
    </div>
    <div class="mono" id="playState">State: STOP</div>
  </div>

  <div class="card">
    <h2>3) シーク</h2>
    <div>0 ～ 曲長</div>
    <input id="seek" type="range" min="0" max="300000" step="1000" value="0">
    <div class="mono" id="seekLabel">00:00 / 00:00</div>
  </div>

  <div class="card">
    <h2>4) 音量</h2>
    <div>0 ～ 127</div>
    <input id="vol" type="range" min="0" max="127" step="1" value="127">
    <div class="mono" id="volLabel">VOL=127</div>
  </div>

  <div class="card">
    <h2>5) ストリームテスト</h2>
    <div class="row">
      <button id="btnTestTone">テスト音 440Hz</button>
      <label for="testChSelect">CH:</label>
      <select id="testChSelect">
        <option value="2">2ch</option>
        <option value="5" selected>5ch</option>
        <option value="8">8ch</option>
      </select>
    </div>
    <div class="mono" id="testToneState">Test: OFF</div>
    <div class="mono" id="testToneRate">Rate: -</div>
    <div class="mono" id="bufferMonitor">Buffer: - / - (0% used)</div>
    <div class="mono" id="underrunMonitor">Underrun: 0</div>
    <div class="mono" id="dropMonitor">Dropped: 0 bytes</div>
  </div>

<script>
(function(){
  const $ = (id)=>document.getElementById(id);

  const wsState = $("wsState");
  const trackSelect = $("trackSelect");
  const selectedTrack = $("selectedTrack");
  const trackInfo = $("trackInfo");
  const btnToggleInfo = $("btnToggleInfo");
  const playState = $("playState");
  const seek = $("seek");
  const seekLabel = $("seekLabel");
  const vol = $("vol");
  const volLabel = $("volLabel");
  const btnTestTone = $("btnTestTone");
  const testChSelect = $("testChSelect");
  const testToneState = $("testToneState");
  const testToneRate = $("testToneRate");
  const bufferMonitor = $("bufferMonitor");
  const underrunMonitor = $("underrunMonitor");
  const dropMonitor = $("dropMonitor");
  const sourceModeSelect = $("sourceModeSelect");
  const sdSourcePanel = $("sdSourcePanel");
  const fileSourcePanel = $("fileSourcePanel");
  const fileInput = $("fileInput");

  let ws;
  let sourceMode = "SD";
  let browserFile = null;
  let fileStreamActive = false;
  let fileStreamStartPending = false;
  let fileStreamPaused = false;
  let fileEofDrainPending = false;
  let fileSendPos = 0;
  let streamBufferSynced = false;
  let pcmSendTimer = null;
  let pcmPumpFn = null;
  let currentTrackIndex = 0;
  let currentLenMs = 0;
  let currentPlayState = "STOP";
  let detailVisible = false;
  let testToneActive = false;
  let testToneFreeBytes = 1048576;
  let testToneUsedBytes = 0;
  let testToneBufferSize = 1048576;
  let testToneUnderrunCount = 0;
  let testToneDroppedBytes = 0;
  let testToneLastDroppedBytes = 0;
  let testToneChunksSent = 0;
  let testToneBackoffUntil = 0;
  const TEST_CHANNEL_MODES = [2, 5, 8];
  let testToneChannels = 5;
  let testTonePhases = [0, 0, 0, 0, 0, 0, 0, 0];
  let testToneStartPending = false;
  let testToneFillPct = 0;
  let testToneResumeAfterReconnect = false;
  let testPrefillTarget = 131072;
  let testLowWater = 98304;
  let testHighWater = 216268;
  const TEST_SAMPLE_RATE = 44100;
  const TEST_AMPLITUDE = 6000;
  const TEST_SEND_MARGIN = 8192;
  const SIN_TABLE_SIZE = 1024;
  const SIN_TABLE = new Float32Array(SIN_TABLE_SIZE);
  for(let i = 0; i < SIN_TABLE_SIZE; i++){
    SIN_TABLE[i] = Math.sin(2 * Math.PI * i / SIN_TABLE_SIZE);
  }

  function fastSin01(phase){
    let idx = (phase * SIN_TABLE_SIZE) | 0;
    if(idx >= SIN_TABLE_SIZE) idx -= SIN_TABLE_SIZE;
    return SIN_TABLE[idx];
  }

  function testFramesPerChunk(){
    return 256;
  }

  function testChunkBytes(){
    return testFramesPerChunk() * testToneChannels * 2;
  }

  function testDataRate(){
    return TEST_SAMPLE_RATE * testToneChannels * 2;
  }

  function bandTierForChannels(ch){
    if(ch >= 8) return 2;
    if(ch >= 5) return 1;
    return 0;
  }

  function pumpIntervalMsForChannels(ch){
    const tier = bandTierForChannels(ch);
    if(tier >= 2) return 5;
    if(tier >= 1) return 8;
    return 10;
  }

  function wsBufferMaxForChannels(ch){
    const tier = bandTierForChannels(ch);
    if(tier >= 2) return 65536;
    if(tier >= 1) return 49152;
    return 32768;
  }

  function testBandTier(){
    return bandTierForChannels(testToneChannels);
  }

  function testPumpIntervalMs(){
    const tier = testBandTier();
    if(tier >= 2) return 5;
    if(tier >= 1) return 8;
    return 10;
  }

  function testWsBufferMax(){
    const tier = testBandTier();
    if(tier >= 2) return 65536;
    if(tier >= 1) return 49152;
    return 32768;
  }

  function setTestToneChannels(ch){
    const n = parseInt(ch, 10);
    testToneChannels = TEST_CHANNEL_MODES.includes(n) ? n : 5;
  }

  function updateTestChModeUi(){
    testChSelect.value = String(testToneChannels);
    testChSelect.disabled = testToneActive;
    updateTestRateLabel();
  }

  function updateTestRateLabel(){
    const kbps = Math.round((testDataRate() * 8) / 1000);
    testToneRate.textContent =
      "Rate: " + testDataRate() + " B/s (" + kbps + " kbps), chunk " + testChunkBytes() + " B";
  }

  function updateStreamWatermarks(channels){
    const cap = testToneBufferSize > 0 ? testToneBufferSize : 1048576;
    const chunk = 256 * channels * 2;
    testPrefillTarget = Math.max(chunk * 8, Math.floor(cap / 3));
    testLowWater = Math.max(chunk * 8, Math.floor(cap * 0.30));
    testHighWater = Math.floor(cap * 0.50);
  }

  function updateTestWatermarks(){
    updateStreamWatermarks(testToneChannels);
  }

  function maxPumpChunksForChannels(channels){
    if(testToneFillPct < 20 || testToneUsedBytes < testLowWater) return 16;
    if(testToneFillPct < 40) return 12;
    if(testToneFillPct >= 75) return 2;
    if(testToneFillPct >= 70) return 3;
    if(channels >= 8){
      if(testToneStartPending || fileStreamStartPending) return 6;
      if(testToneUsedBytes < testLowWater) return 4;
      return 3;
    }
    if(channels >= 5){
      if(testToneStartPending || fileStreamStartPending) return 6;
      if(testToneUsedBytes < testLowWater) return 4;
      return 3;
    }
    if(testToneStartPending || fileStreamStartPending) return 6;
    if(testToneUsedBytes < testLowWater) return 4;
    return 3;
  }

  function maxPumpChunks(){
    return maxPumpChunksForChannels(testToneChannels);
  }

  function mmss(ms){
    ms = Math.max(0, ms|0);
    const sec = Math.floor(ms/1000);
    const m = Math.floor(sec/60);
    const s = sec%60;
    return String(m).padStart(2,"0") + ":" + String(s).padStart(2,"0");
  }

  function updateSeekLabel(){
    seekLabel.textContent = mmss(parseInt(seek.value,10)||0) + " / " + mmss(currentLenMs);
  }

  function updateVolLabel(){
    volLabel.textContent = "VOL=" + (parseInt(vol.value,10)||0);
  }

  function updatePlaybackLock(){
    const sdBusy = sourceMode === "SD" && (currentPlayState === "PLAY" || currentPlayState === "PAUSE");
    const fileBusy = sourceMode === "FILE" && fileStreamActive;
    const busy = sdBusy || fileBusy || testToneActive;
    trackSelect.disabled = busy;
    fileInput.disabled = busy;
    sourceModeSelect.disabled = busy;
  }

  function updateTrackLock(){
    updatePlaybackLock();
  }

  function readFixedUzuString(view, offset, len){
    let end = 0;
    for(let i = 0; i < len; i++){
      if(view.getUint8(offset + i) === 0) break;
      end = i + 1;
    }
    while(end > 0){
      const c = view.getUint8(offset + end - 1);
      if(c === 0x20 || c === 0x09 || c === 0x0d || c === 0x0a) end--;
      else break;
    }
    if(end <= 0) return "";
    const slice = new Uint8Array(view.buffer, view.byteOffset + offset, end);
    return new TextDecoder("utf-8").decode(slice);
  }

  function parseUzuBuffer(buf, fileName){
    if(buf.byteLength < 32768) throw new Error("ファイルが小さすぎます");
    const view = new DataView(buf);
    let magic = "";
    for(let i = 0; i < 8; i++) magic += String.fromCharCode(view.getUint8(i));
    if(magic !== "UZUCPW1\u0000") throw new Error("UZUマジック不一致");

    const dataStart = view.getUint32(8, true);
    const channels = view.getUint32(12, true);
    const sampleRate = view.getUint32(16, true);
    const bits = view.getUint32(20, true);
    const totalSamples = view.getUint32(24, true);

    if(dataStart < 32768) throw new Error("dataStart が不正です");
    if(channels === 0 || channels > 8) throw new Error("チャンネル数が未対応です");
    if(bits !== 16) throw new Error("16bit のみ対応です");
    if(sampleRate !== 44100 && sampleRate !== 48000) throw new Error("サンプルレート未対応です");
    if(dataStart >= buf.byteLength) throw new Error("データ開始位置が不正です");

    const title = readFixedUzuString(view, 28, 256);
    const channelNames = [];
    for(let i = 0; i < channels && i < 8; i++){
      channelNames.push(readFixedUzuString(view, 284 + i * 256, 256));
    }
    const lengthMs = Math.floor((totalSamples * 1000) / sampleRate);

    return {
      valid: true,
      name: fileName || "local.UZU",
      title,
      channels,
      sampleRate,
      bits,
      totalSamples,
      lengthMs,
      dataStart,
      channelNames,
      buffer: buf
    };
  }

  function msToFilePos(ms){
    if(!browserFile) return 0;
    const sample = Math.floor((Math.max(0, ms|0) * browserFile.sampleRate) / 1000);
    const clamped = Math.min(sample, browserFile.totalSamples);
    return browserFile.dataStart + clamped * browserFile.channels * 2;
  }

  function filePosToMs(pos){
    if(!browserFile) return 0;
    const rel = Math.max(0, pos - browserFile.dataStart);
    const samples = Math.floor(rel / (browserFile.channels * 2));
    return Math.floor((samples * 1000) / browserFile.sampleRate);
  }

  function notifyEspFileHeader(meta){
    streamBufferSynced = false;
    send({cmd:"set_mode", mode:"STREAM"});
    send({cmd:"file_info", name:meta.name, sampleRate:meta.sampleRate, bitsPerSample:16, channels:meta.channels, durationMs:meta.lengthMs});
    send({cmd:"prepare"});
  }

  function releaseEspStreamIfPrepared(){
    send({cmd:"stop"});
    send({cmd:"reset_buffer"});
    send({cmd:"set_mode", mode:"SD"});
  }

  function updateSourceModeUi(){
    const isSd = sourceMode === "SD";
    sdSourcePanel.classList.toggle("hidden", !isSd);
    fileSourcePanel.classList.toggle("hidden", isSd);
    sourceModeSelect.value = sourceMode;
    updatePlaybackLock();
  }

  function showTrackInfo(msg){
    trackInfo.textContent = formatTrackInfo(msg);
  }

  function startPcmPump(fn, intervalMs){
    stopPcmPump();
    pcmPumpFn = fn;
    pcmSendTimer = setInterval(()=>{
      if(pcmPumpFn) pcmPumpFn();
    }, intervalMs);
  }

  function stopPcmPump(){
    if(!pcmSendTimer) return;
    clearInterval(pcmSendTimer);
    pcmSendTimer = null;
    pcmPumpFn = null;
  }

  function formatTrackInfo(msg){
    if(!msg || typeof msg !== "object") return "Info: -";
    const lines = [];
    lines.push("NAME: " + (msg.name || "-"));
    if(msg.title) lines.push("TITLE: " + msg.title);
    lines.push("RATE: " + (msg.sampleRate || 0) + " Hz");
    lines.push("CH: " + (msg.channels || 0));
    lines.push("BITS: " + (msg.bits || 0));
    lines.push("LEN: " + mmss(msg.lengthMs || 0));
    lines.push("SAMPLES: " + (msg.totalSamples || 0));
    if(Array.isArray(msg.channelNames)) {
      msg.channelNames.forEach((v, idx)=>{
        lines.push("CH" + (idx+1) + ": " + (v || ""));
      });
    }
    return lines.join("\n");
  }

  updateSeekLabel();
  updateVolLabel();
  updateTrackLock();
  updateBufferMonitor();
  updateTestChModeUi();

  function send(obj){
    const s = JSON.stringify(obj);
    if(ws && ws.readyState === WebSocket.OPEN){
      ws.send(s);
    }
  }

  function updateBufferMonitor(){
    const used = testToneUsedBytes|0;
    const total = testToneBufferSize|0;
    const free = testToneFreeBytes|0;
    const pct = total > 0 ? Math.round((used * 100) / total) : 0;
    const chLabel = fileStreamActive && browserFile ? (browserFile.channels + "ch file")
      : (testToneChannels + "ch test");
    bufferMonitor.textContent =
      "Buffer: used " + used + " / " + total + " (" + pct + "%), free " + free +
      ", " + chLabel + ", sent " + (testToneChunksSent|0);
    underrunMonitor.textContent = "Underrun: " + (testToneUnderrunCount|0);
    dropMonitor.textContent = "Dropped: " + (testToneDroppedBytes|0) + " bytes";
  }

  function buildTestPcmChunk(){
    const chCount = testToneChannels;
    const buf = new ArrayBuffer(testChunkBytes());
    const view = new DataView(buf);
    const step = 440 / TEST_SAMPLE_RATE;
    let off = 0;
    let phase = testTonePhases[0];
    const frames = testFramesPerChunk();
    for(let f = 0; f < frames; f++){
      const sample = Math.round(TEST_AMPLITUDE * fastSin01(phase));
      phase += step;
      if(phase >= 1) phase -= 1;
      for(let ch = 0; ch < chCount; ch++){
        view.setInt16(off, sample, true);
        off += 2;
      }
    }
    testTonePhases[0] = phase;
    return buf;
  }

  function canSendTestPcm(){
    if(!testToneActive || !ws || ws.readyState !== WebSocket.OPEN) return false;
    if(Date.now() < testToneBackoffUntil) return false;
    if(ws.bufferedAmount > testWsBufferMax()) return false;
    const chunk = testChunkBytes();
    if(testToneFreeBytes < chunk) return false;
    return true;
  }

  function sendTestChunks(maxCount){
    let sent = 0;
    const chunk = testChunkBytes();
    while(sent < maxCount){
      if(!canSendTestPcm()) break;
      ws.send(buildTestPcmChunk());
      testToneChunksSent++;
      sent++;
    }
    return sent;
  }

  function pumpTestPcm(){
    if(!testToneActive) return;
    if(ws.bufferedAmount > testWsBufferMax()) return;
    sendTestChunks(maxPumpChunks());
    maybeStartTestPlayback();
  }

  function maybeStartTestPlayback(){
    if(!testToneStartPending) return;
    if(testToneUsedBytes < testPrefillTarget) return;
    testToneStartPending = false;
    send({cmd:"start"});
    testToneState.textContent = "Test: ON (" + testToneChannels + "ch 440Hz) / PLAYING";
  }

  function onStreamBufferInfo(msg){
    streamBufferSynced = true;
    if(typeof msg.bufferSize === "number") testToneBufferSize = msg.bufferSize;
    const ch = fileStreamActive && browserFile ? browserFile.channels : testToneChannels;
    updateStreamWatermarks(ch);
    if(typeof msg.freeBytes === "number") testToneFreeBytes = msg.freeBytes;
    if(typeof msg.usedBytes === "number") testToneUsedBytes = msg.usedBytes;
    if(typeof msg.fillPct === "number") testToneFillPct = msg.fillPct;
    if(typeof msg.underrunCount === "number"){
      if(msg.underrunCount > testToneUnderrunCount){
        testToneBackoffUntil = 0;
        if(typeof msg.freeBytes === "number"){
          testToneFreeBytes = msg.freeBytes;
        }
        if(typeof msg.usedBytes === "number"){
          testToneUsedBytes = msg.usedBytes;
        }
      }
      testToneUnderrunCount = msg.underrunCount;
    }
    if(typeof msg.droppedBytes === "number"){
      if(msg.droppedBytes > testToneLastDroppedBytes){
        testToneBackoffUntil = Date.now() + 200;
      }
      testToneLastDroppedBytes = msg.droppedBytes;
      testToneDroppedBytes = msg.droppedBytes;
    }
    if(msg.sendHold === true){
      testToneBackoffUntil = Date.now() + 50;
    }
    updateBufferMonitor();
    maybeStartTestPlayback();
    maybeStartFilePlayback();
    if(fileEofDrainPending && testToneUsedBytes < fileChunkBytes()){
      fileEofDrainPending = false;
      send({cmd:"stop"});
    }
  }

  function restartTestToneSendTimer(){
    stopPcmPump();
    startPcmPump(pumpTestPcm, testPumpIntervalMs());
  }

  function startTestToneSendTimer(){
    if(pcmSendTimer && pcmPumpFn === pumpTestPcm) return;
    startPcmPump(pumpTestPcm, testPumpIntervalMs());
  }

  function stopTestToneSendTimer(){
    if(pcmPumpFn === pumpTestPcm) stopPcmPump();
  }

  function startTestTone(){
    if(fileStreamActive) stopFilePlayback();
    send({cmd:"set_mode", mode:"STREAM"});
    send({cmd:"file_info", name:"test.wav", sampleRate:TEST_SAMPLE_RATE, bitsPerSample:16, channels:testToneChannels});
    send({cmd:"prepare"});
    testToneActive = true;
    testToneStartPending = true;
    testTonePhases.fill(0);
    testToneUnderrunCount = 0;
    testToneDroppedBytes = 0;
    testToneLastDroppedBytes = 0;
    testToneBackoffUntil = 0;
    testToneChunksSent = 0;
    updateTestWatermarks();
    updateTestChModeUi();
    btnTestTone.textContent = "テスト音 停止";
    testToneState.textContent = "Test: ON (" + testToneChannels + "ch 440Hz) / BUFFERING";
    updateBufferMonitor();

    restartTestToneSendTimer();
  }

  function stopTestToneLocal(){
    testToneActive = false;
    testToneStartPending = false;
    stopTestToneSendTimer();
    testToneBackoffUntil = 0;
    btnTestTone.textContent = "テスト音 440Hz";
    testToneState.textContent = "Test: OFF";
    testToneFreeBytes = testToneBufferSize;
    testToneUsedBytes = 0;
    updateTestChModeUi();
    updateBufferMonitor();
  }

  function stopTestTone(){
    stopTestToneLocal();
    testToneResumeAfterReconnect = false;
    send({cmd:"stop"});
    send({cmd:"reset_buffer"});
    send({cmd:"set_mode", mode:"SD"});
  }

  function fileChunkBytes(){
    return browserFile ? (256 * browserFile.channels * 2) : 0;
  }

  function buildFilePcmChunk(){
    if(!browserFile) return null;
    const chunkBytes = fileChunkBytes();
    const remain = browserFile.buffer.byteLength - fileSendPos;
    if(remain <= 0) return null;
    const buf = new ArrayBuffer(chunkBytes);
    const out = new Uint8Array(buf);
    const copyLen = Math.min(remain, chunkBytes);
    out.set(new Uint8Array(browserFile.buffer, fileSendPos, copyLen));
    fileSendPos += copyLen;
    return buf;
  }

  function canSendFilePcm(){
    if(!fileStreamActive || fileStreamPaused || !browserFile || !ws || ws.readyState !== WebSocket.OPEN) return false;
    if(Date.now() < testToneBackoffUntil) return false;
    if(ws.bufferedAmount > wsBufferMaxForChannels(browserFile.channels)) return false;
    const chunk = fileChunkBytes();
    if(testToneFreeBytes < chunk) return false;
    if(fileSendPos >= browserFile.buffer.byteLength) return false;
    return true;
  }

  function sendFileChunks(maxCount){
    let sent = 0;
    const chunk = fileChunkBytes();
    while(sent < maxCount){
      if(!canSendFilePcm()) break;
      const pcm = buildFilePcmChunk();
      if(!pcm) break;
      ws.send(pcm);
      testToneChunksSent++;
      sent++;
    }
    return sent;
  }

  function pumpFilePcm(){
    if(!fileStreamActive || fileStreamPaused) return;
    if(!browserFile) return;
    if(ws.bufferedAmount > wsBufferMaxForChannels(browserFile.channels)) return;
    sendFileChunks(maxPumpChunksForChannels(browserFile.channels));
    maybeStartFilePlayback();
    if(fileStreamActive && !fileStreamStartPending){
      const posMs = filePosToMs(fileSendPos);
      seek.value = String(Math.min(posMs, currentLenMs));
      updateSeekLabel();
    }
    if(fileSendPos >= browserFile.buffer.byteLength && !fileStreamStartPending){
      stopPcmPump();
      fileStreamActive = false;
      fileStreamStartPending = false;
      fileStreamPaused = false;
      fileEofDrainPending = true;
      currentPlayState = "STOP";
      playState.textContent = "State: STOP (END)";
      updatePlaybackLock();
    }
  }

  function maybeStartFilePlayback(){
    if(!fileStreamStartPending) return;
    if(!streamBufferSynced) return;
    if(testToneUsedBytes < testPrefillTarget) return;
    fileStreamStartPending = false;
    send({cmd:"start"});
    currentPlayState = "PLAY";
    playState.textContent = "State: PLAY";
    updatePlaybackLock();
  }

  function startFilePlayback(){
    if(!browserFile) return;
    if(testToneActive) stopTestTone();
    notifyEspFileHeader(browserFile);
    fileStreamActive = true;
    fileStreamPaused = false;
    fileStreamStartPending = true;
    fileSendPos = msToFilePos(parseInt(seek.value, 10) || 0);
    testToneUnderrunCount = 0;
    testToneDroppedBytes = 0;
    testToneLastDroppedBytes = 0;
    testToneBackoffUntil = 0;
    testToneChunksSent = 0;
    testToneUsedBytes = 0;
    testToneFreeBytes = testToneBufferSize;
    streamBufferSynced = false;
    updateStreamWatermarks(browserFile.channels);
    currentPlayState = "PLAY";
    playState.textContent = "State: BUFFERING";
    updatePlaybackLock();
    startPcmPump(pumpFilePcm, pumpIntervalMsForChannels(browserFile.channels));
  }

  function stopFilePlaybackLocal(){
    stopPcmPump();
    fileStreamActive = false;
    fileStreamStartPending = false;
    fileStreamPaused = false;
    fileEofDrainPending = false;
    currentPlayState = "STOP";
    playState.textContent = "State: STOP";
    testToneFreeBytes = testToneBufferSize;
    testToneUsedBytes = 0;
    updatePlaybackLock();
  }

  function stopFilePlayback(){
    stopFilePlaybackLocal();
    releaseEspStreamIfPrepared();
  }

  function pauseFilePlayback(){
    if(!fileStreamActive || fileStreamPaused) return;
    fileStreamPaused = true;
    stopPcmPump();
    send({cmd:"pause"});
    currentPlayState = "PAUSE";
    playState.textContent = "State: PAUSE";
    updatePlaybackLock();
  }

  function resumeFilePlayback(){
    if(!fileStreamActive || !fileStreamPaused || !browserFile) return;
    fileStreamPaused = false;
    send({cmd:"resume"});
    currentPlayState = "PLAY";
    playState.textContent = "State: PLAY";
    startPcmPump(pumpFilePcm, pumpIntervalMsForChannels(browserFile.channels));
    updatePlaybackLock();
  }

  function endFilePlaybackAtEof(){
    stopPcmPump();
    fileStreamActive = false;
    fileStreamStartPending = false;
    fileStreamPaused = false;
    fileEofDrainPending = true;
    currentPlayState = "STOP";
    playState.textContent = "State: STOP (END)";
    updatePlaybackLock();
  }

  function applyFileSeek(posMs){
    if(!browserFile) return;
    fileSendPos = msToFilePos(posMs);
    seek.value = String(Math.max(0, Math.min(posMs, currentLenMs)));
    updateSeekLabel();
    if(!fileStreamActive) return;
    send({cmd:"reset_buffer"});
    testToneUsedBytes = 0;
    testToneFreeBytes = testToneBufferSize;
    testToneUnderrunCount = 0;
    testToneChunksSent = 0;
    if(!fileStreamPaused){
      startPcmPump(pumpFilePcm, pumpIntervalMsForChannels(browserFile.channels));
    }
  }

  function loadBrowserFile(file){
    return file.arrayBuffer().then((buf)=>{
      browserFile = parseUzuBuffer(buf, file.name);
      selectedTrack.textContent = "Selected: " + browserFile.name;
      showTrackInfo(browserFile);
      currentLenMs = browserFile.lengthMs;
      seek.max = String(currentLenMs > 0 ? currentLenMs : 1);
      seek.value = "0";
      fileSendPos = browserFile.dataStart;
      updateSeekLabel();
      if(!detailVisible){
        detailVisible = true;
        trackInfo.style.display = "block";
        btnToggleInfo.textContent = "詳細非表示";
      }
    });
  }

  function setSourceMode(mode){
    if(mode !== "SD" && mode !== "FILE") mode = "SD";
    if(mode === sourceMode) return;
    if(testToneActive) stopTestTone();
    if(fileStreamActive) stopFilePlayback();
    else if(sourceMode === "FILE" && browserFile) releaseEspStreamIfPrepared();
    if(sourceMode === "SD" && (currentPlayState === "PLAY" || currentPlayState === "PAUSE")){
      send({cmd:"stop"});
    }

    sourceMode = mode;
    browserFile = null;
    fileInput.value = "";
    selectedTrack.textContent = "Selected: -";
    trackInfo.textContent = "Info: -";
    currentLenMs = 0;
    seek.value = "0";
    seek.max = "300000";
    updateSeekLabel();

    if(sourceMode === "SD"){
      send({cmd:"get_tracks"});
      send({cmd:"get_state"});
    }
    updateSourceModeUi();
  }

  function connectWS(){
    const host = window.location.hostname || "192.168.4.1";
    const url = "ws://" + host + "/";
    wsState.textContent = "WS: CONNECTING";

    ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";

    ws.onopen = ()=>{
      wsState.textContent = "WS: CONNECTED";
      send({cmd:"hello", client:"web"});
      send({cmd:"get_tracks"});
      send({cmd:"get_state"});
      if(testToneResumeAfterReconnect){
        testToneResumeAfterReconnect = false;
        setTimeout(startTestTone, 300);
      }
    };

    ws.onclose = (ev)=>{
      testToneResumeAfterReconnect = testToneActive;
      stopTestToneLocal();
      stopFilePlaybackLocal();
      wsState.textContent = "WS: CLOSED code=" + ev.code;
      setTimeout(connectWS, 1000);
    };

    ws.onerror = ()=>{
      wsState.textContent = "WS: ERROR";
    };

    ws.onmessage = (ev)=>{
      if(typeof ev.data !== "string"){
        return;
      }
      try{
        const msg = JSON.parse(ev.data);

        if(msg.cmd === "buffer_info"){
          onStreamBufferInfo(msg);
          return;
        }

        if(msg.cmd === "status"){
          if(typeof msg.state === "string"){
            if(testToneActive){
              testToneState.textContent = "Test: ON (" + testToneChannels + "ch 440Hz) / " + msg.state;
            } else {
              testToneState.textContent = "Test: OFF / " + msg.state;
            }
            if(fileStreamActive){
              if(msg.state === "PLAYING"){
                currentPlayState = fileStreamPaused ? "PAUSE" : "PLAY";
              } else if(msg.state === "PAUSED"){
                currentPlayState = "PAUSE";
              } else if(msg.state === "BUFFERING"){
                currentPlayState = "PLAY";
                playState.textContent = "State: BUFFERING";
                return;
              } else if(msg.state === "STOPPED" || msg.state === "IDLE"){
                if(!fileStreamStartPending) currentPlayState = "STOP";
              }
              playState.textContent = "State: " + (fileStreamPaused ? "PAUSE" : currentPlayState);
              updatePlaybackLock();
            }
          }
          return;
        }

        if(msg.cmd === "underrun"){
          testToneBackoffUntil = 0;
          testToneUnderrunCount++;
          if(fileStreamActive && browserFile && fileSendPos < browserFile.buffer.byteLength){
            sendFileChunks(24);
          }
          updateBufferMonitor();
          return;
        }

        if(msg.type === "tracks" && Array.isArray(msg.items)){
          if(sourceMode !== "SD") return;
          trackSelect.innerHTML = "";
          msg.items.forEach((name, idx)=>{
            const opt = document.createElement("option");
            opt.value = String(idx);
            opt.textContent = name;
            trackSelect.appendChild(opt);
          });
          if (msg.items.length > 0) {
            if (currentTrackIndex >= msg.items.length) currentTrackIndex = 0;
            trackSelect.value = String(currentTrackIndex);
            selectedTrack.textContent = "Selected: " + (trackSelect.options[currentTrackIndex]?.textContent || "-");
            if (detailVisible) send({cmd:"get_info", trackIndex: currentTrackIndex});
          } else {
            selectedTrack.textContent = "Selected: -";
            trackInfo.textContent = "Info: -";
          }
          updateTrackLock();
          return;
        }

        if(msg.type === "track_info") {
          if(sourceMode !== "SD") return;
          trackInfo.textContent = formatTrackInfo(msg);
          return;
        }

        if(msg.type === "state"){
          if(sourceMode !== "SD" && fileStreamActive) return;
          if(typeof msg.play === "string") {
            currentPlayState = msg.play;
            playState.textContent = "State: " + msg.play;
            updateTrackLock();
          }

          if(typeof msg.trackIndex === "number") {
            currentTrackIndex = msg.trackIndex;
            trackSelect.value = String(currentTrackIndex);
            selectedTrack.textContent = "Selected: " + (trackSelect.options[currentTrackIndex]?.textContent || "-");
          }

          if(typeof msg.lenMs === "number"){
            currentLenMs = msg.lenMs;
            seek.max = String(currentLenMs > 0 ? currentLenMs : 300000);
          }

          if(typeof msg.posMs === "number"){
            let p = msg.posMs;
            if (p < 0) p = 0;
            if (currentLenMs > 0 && p > currentLenMs) p = currentLenMs;
            seek.value = String(p);
          }

          if(typeof msg.vol === "number"){
            vol.value = String(msg.vol);
          }

          updateSeekLabel();
          updateVolLabel();
          return;
        }
      }catch(e){
      }
    };
  }

  connectWS();

  updateSourceModeUi();

  trackSelect.onchange = ()=>{
    if(sourceMode !== "SD") return;
    if (trackSelect.disabled) return;

    currentTrackIndex = parseInt(trackSelect.value,10) || 0;
    const name = trackSelect.options[currentTrackIndex]?.textContent || "-";
    selectedTrack.textContent = "Selected: " + name;

    seek.value = "0";
    updateSeekLabel();

    send({cmd:"select", trackIndex: currentTrackIndex});
    if (detailVisible) send({cmd:"get_info", trackIndex: currentTrackIndex});
  };

  btnToggleInfo.onclick = ()=> {
    detailVisible = !detailVisible;
    trackInfo.style.display = detailVisible ? "block" : "none";
    btnToggleInfo.textContent = detailVisible ? "詳細非表示" : "詳細表示";
    if (detailVisible && sourceMode === "SD") {
      send({cmd:"get_info", trackIndex: currentTrackIndex});
    }
  };

  sourceModeSelect.onchange = ()=>{
    setSourceMode(sourceModeSelect.value);
  };

  fileInput.onchange = ()=>{
    const file = fileInput.files && fileInput.files[0];
    if(!file) return;
    if(fileStreamActive) stopFilePlayback();
    loadBrowserFile(file).catch((err)=>{
      browserFile = null;
      alert("UZU読込エラー: " + err.message);
    });
  };

  $("btnPlay").onclick = ()=>{
    if(sourceMode === "SD"){
      send({cmd:"play"});
      return;
    }
    if(!browserFile){
      alert("UZUファイルを選択してください");
      return;
    }
    if(fileStreamActive && fileStreamPaused) resumeFilePlayback();
    else if(!fileStreamActive) startFilePlayback();
  };

  $("btnStop").onclick = ()=>{
    if(sourceMode === "SD") send({cmd:"stop"});
    else stopFilePlayback();
  };

  $("btnPause").onclick = ()=>{
    if(sourceMode === "SD") send({cmd:"pause"});
    else pauseFilePlayback();
  };

  seek.oninput = ()=> updateSeekLabel();
  seek.onchange = ()=>{
    const posMs = parseInt(seek.value,10) || 0;
    if(sourceMode === "SD"){
      send({cmd:"seek", posMs});
      return;
    }
    applyFileSeek(posMs);
  };

  vol.oninput = ()=>{
    updateVolLabel();
    send({cmd:"vol", value: parseInt(vol.value,10) || 0});
  };

  btnTestTone.onclick = ()=>{
    if(testToneActive) stopTestTone();
    else startTestTone();
  };

  testChSelect.onchange = ()=>{
    if(testToneActive || fileStreamActive){
      testChSelect.value = String(testToneChannels);
      return;
    }
    setTestToneChannels(testChSelect.value);
    updateTestChModeUi();
    updateTestWatermarks();
  };

  updateTestChModeUi();
  updateTestWatermarks();
  updateSourceModeUi();

  setInterval(()=>{
    if(testToneActive || fileStreamActive || sourceMode === "FILE") return;
    if(ws && ws.readyState === WebSocket.OPEN){
      send({cmd:"get_state"});
    }
  }, 1000);
})();
</script>
</body>
</html>
)HTML";

static void sendHttpHtmlPage(WSclient_t* client, PGM_P html) {
  size_t len = strlen_P(html);
  client->tcp->printf(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Cache-Control: no-cache, no-store, must-revalidate\r\n"
    "Connection: close\r\n"
    "Content-Length: %u\r\n"
    "\r\n",
    (unsigned)len);

  const size_t chunk = 1024;
  for (size_t off = 0; off < len; off += chunk) {
    size_t n = (len - off > chunk) ? (chunk) : (len - off);
    char buf[1024];
    memcpy_P(buf, html + off, n);
    client->tcp->write((uint8_t*)buf, n);
  }
}

static void sendHttpRedirect(WSclient_t* client, const char* location) {
  client->tcp->printf(
    "HTTP/1.1 302 Found\r\n"
    "Location: %s\r\n"
    "Content-Type: text/plain\r\n"
    "Connection: close\r\n"
    "Content-Length: 12\r\n"
    "\r\n"
    "Redirecting",
    location);
}

class UzuWebSocketsServer : public WebSocketsServer {
public:
  UzuWebSocketsServer(uint16_t port) : WebSocketsServer(port) {}

protected:
  void handleNonWebsocketConnection(WSclient_t* client) override {
    String path = client->cUrl;
    int q = path.indexOf('?');
    if (q >= 0) path = path.substring(0, q);
    if (path.length() == 0) path = "/";

    if (path == "/" || path == "/index.html") {
      sendHttpHtmlPage(client, kHtml);
      clientDisconnect(client);
      return;
    }

    char loc[64];
    snprintf(loc, sizeof(loc), "http://%s/", AP_IP.toString().c_str());
    sendHttpRedirect(client, loc);
    clientDisconnect(client);
  }
};

static UzuWebSocketsServer ws(80);

// ====================================================
// Utility implementations
// ====================================================
bool isCardInserted() {
  return digitalRead(PIN_SD_CD) == LOW;
}

static void make_abs_path(const char* in, char* out, size_t out_len) {
  if (!in || !in[0]) {
    strncpy(out, "/", out_len);
    out[out_len - 1] = '\0';
    return;
  }
  if (in[0] == '/') {
    strncpy(out, in, out_len);
    out[out_len - 1] = '\0';
    return;
  }
  out[0] = '/';
  strncpy(out + 1, in, out_len - 1);
  out[out_len - 1] = '\0';
}

static void join_path(char* out, size_t out_len, const char* dir, const char* name) {
  if (!out || out_len == 0) return;

  if (!dir || !dir[0]) {
    snprintf(out, out_len, "/%s", name ? name : "");
    return;
  }

  if (strcmp(dir, "/") == 0) {
    snprintf(out, out_len, "/%s", name ? name : "");
    return;
  }

  size_t len = strlen(dir);
  if (len > 0 && dir[len - 1] == '/') {
    snprintf(out, out_len, "%s%s", dir, name ? name : "");
  } else {
    snprintf(out, out_len, "%s/%s", dir, name ? name : "");
  }
}

static bool hasExtensionIgnoreCase(const char* name, const char* ext) {
  if (!name || !ext) return false;

  size_t nameLen = strlen(name);
  size_t extLen  = strlen(ext);
  if (nameLen < extLen) return false;

  const char* p = name + (nameLen - extLen);
  for (size_t i = 0; i < extLen; ++i) {
    char a = p[i];
    char b = ext[i];
    if (a >= 'a' && a <= 'z') a = char(a - 'a' + 'A');
    if (b >= 'a' && b <= 'z') b = char(b - 'a' + 'A');
    if (a != b) return false;
  }
  return true;
}

static uint32_t le32_from_buf(const uint8_t* p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int16_t le16_to_s16(const uint8_t* p) {
  return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

static bool read_u32_at(File& f, uint32_t offset, uint32_t& value) {
  if (!f.seek(offset)) return false;
  size_t n = f.read(g_u32buf, 4);
  if (n != 4) return false;
  value = le32_from_buf(g_u32buf);
  return true;
}

static bool read_string_at(File& f, uint32_t offset, uint32_t len, char* out, size_t outSize) {
  if (!out || outSize == 0) return false;
  if (len + 1 > outSize) return false;
  if (!f.seek(offset)) return false;

  size_t n = f.read((uint8_t*)out, len);
  if (n != len) return false;

  out[len] = '\0';

  while (len > 0) {
    char c = out[len - 1];
    if (c == '\0' || c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      out[len - 1] = '\0';
      --len;
    } else {
      break;
    }
  }
  return true;
}

static bool check_uzu_magic(File& f) {
  static const uint8_t kMagic[8] = { 'U', 'Z', 'U', 'C', 'P', 'W', '1', 0 };
  uint8_t magic[8];
  if (!f.seek(0)) return false;
  size_t n = f.read(magic, sizeof(magic));
  if (n != sizeof(magic)) return false;
  return memcmp(magic, kMagic, sizeof(kMagic)) == 0;
}

static bool read_uzu_length_ms(const char* path, uint32_t& lengthMs) {
  lengthMs = 0;
  if (!path || !path[0] || !g_mounted) return false;

  if (isUzcMp3File(path)) {
    return readUzcLengthMs(path, lengthMs);
  }

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;

  if (!check_uzu_magic(f)) {
    f.close();
    return false;
  }

  uint32_t sampleRate = 0;
  uint32_t totalSamples = 0;

  bool ok = read_u32_at(f, 16, sampleRate) &&
            read_u32_at(f, 24, totalSamples);

  f.close();

  if (!ok) return false;
  if (sampleRate == 0) return false;

  lengthMs = (uint32_t)(((uint64_t)totalSamples * 1000ULL) / sampleRate);
  return true;
}

static const char* getCardTypeString() {
  uint8_t t = SD_MMC.cardType();
  if (t == CARD_MMC)  return "MMC";
  if (t == CARD_SD)   return "SDSC";
  if (t == CARD_SDHC) return "SDHC/SDXC";
  return "UNKNOWN";
}

static bool mount_sdmmc_4bit(uint32_t freq_hz) {
  if (g_mounted) {
    SD_MMC.end();
    g_mounted = false;
  }

  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);

  if (!isCardInserted()) {
    return false;
  }

  if (!SD_MMC.begin("/sdcard", false, false, freq_hz)) {
    g_mounted = false;
    return false;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    SD_MMC.end();
    g_mounted = false;
    return false;
  }

  g_mounted = true;
  return true;
}

static void unmount_sdmmc() {
  if (g_mounted) {
    SD_MMC.end();
    g_mounted = false;
  }
}

static void clearFileList() {
  g_fileCount = 0;
  for (int i = 0; i < MAX_FILES; ++i) {
    g_fileList[i].name[0] = '\0';
    g_fileList[i].path[0] = '\0';
  }
}

static int16_t apply_volume_127(int16_t sample, uint8_t vol) {
  if (vol == 0) return 0;
  if (vol >= 127) return sample;

  int32_t v = ((int32_t)sample * (int32_t)vol) / 127;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

static int16_t lerp16_q16(int16_t a, int16_t b, uint16_t frac_q16) {
  int64_t da = (int64_t)b - (int64_t)a;
  int64_t v = (int64_t)a + ((da * (int64_t)frac_q16) >> 16);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// ====================================================
// Shared scan function
// ====================================================
static bool scanUzuFiles(const char* path) {
  if (!g_mounted) return false;

  char dirPath[128];
  make_abs_path((path && path[0]) ? path : "/", dirPath, sizeof(dirPath));

  File root = SD_MMC.open(dirPath);
  if (!root) return false;
  if (!root.isDirectory()) {
    root.close();
    return false;
  }

  clearFileList();

  while (true) {
    File e = root.openNextFile();
    if (!e) break;

    if (!e.isDirectory()) {
      const char* nm = e.name();
      if (nm) {
        char entryName[96];
        strncpy(entryName, nm, sizeof(entryName));
        entryName[sizeof(entryName) - 1] = '\0';

        const char* base = strrchr(entryName, '/');
        const char* showName = base ? (base + 1) : entryName;

        if (hasExtensionIgnoreCase(showName, ".UZU") || hasExtensionIgnoreCase(showName, ".uzc")) {
          if (g_fileCount < MAX_FILES) {
            strncpy(g_fileList[g_fileCount].name, showName, sizeof(g_fileList[g_fileCount].name));
            g_fileList[g_fileCount].name[sizeof(g_fileList[g_fileCount].name) - 1] = '\0';

            if (nm[0] == '/') {
              strncpy(g_fileList[g_fileCount].path, nm, sizeof(g_fileList[g_fileCount].path));
              g_fileList[g_fileCount].path[sizeof(g_fileList[g_fileCount].path) - 1] = '\0';
            } else {
              join_path(g_fileList[g_fileCount].path,
                        sizeof(g_fileList[g_fileCount].path),
                        dirPath,
                        nm);
            }

            g_fileCount++;
          }
        }
      }
    }

    e.close();
  }

  root.close();

  if (g_selectedTrack0 >= g_fileCount) {
    g_selectedTrack0 = 0;
  }

  if (g_fileCount > 0) {
    uint32_t lenMs = 0;
    if (read_uzu_length_ms(g_fileList[g_selectedTrack0].path, lenMs)) {
      g_selectedLenMs = lenMs;
    } else {
      g_selectedLenMs = 0;
    }
  } else {
    g_selectedLenMs = 0;
  }

  return true;
}

// ====================================================
// TDM init / control
// ====================================================
static bool i2s_tdm_tx_init(uint32_t sampleRate) {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  esp_err_t err = i2s_new_channel(&chan_cfg, &g_tx_handle, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_new_channel error: %d\n", err);
    return false;
  }

  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg  = I2S_TDM_CLK_DEFAULT_CONFIG(sampleRate),
      .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(
                      I2S_DATA_BIT_WIDTH_16BIT,
                      I2S_SLOT_MODE_STEREO,
                      TDM_SLOT_MASK),
      .gpio_cfg = {
          .mclk = I2S_MCLK_GPIO,
          .bclk = I2S_BCLK_GPIO,
          .ws   = I2S_WS_GPIO,
          .dout = I2S_DOUT_GPIO,
          .din  = I2S_GPIO_UNUSED,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv   = false,
          },
      },
  };

  tdm_cfg.slot_cfg.total_slot     = 8;
  tdm_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;

  err = i2s_channel_init_tdm_mode(g_tx_handle, &tdm_cfg);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_init_tdm_mode error: %d\n", err);
    return false;
  }

  g_tdm_enabled = false;
  g_tdm_current_rate = sampleRate;
  Serial.printf("TDM init done (%lu Hz, 8ch, 16bit)\n", (unsigned long)sampleRate);
  return true;
}

static bool tdm_set_sample_rate(uint32_t sampleRate) {
  if (sampleRate == g_tdm_current_rate && g_tx_handle) {
    return true;
  }

  if (g_tx_handle) {
    if (g_tdm_enabled) {
      i2s_channel_disable(g_tx_handle);
      g_tdm_enabled = false;
    }
    i2s_del_channel(g_tx_handle);
    g_tx_handle = nullptr;
  }

  return i2s_tdm_tx_init(sampleRate);
}

static bool tdm_enable_output() {
  if (!g_tx_handle) return false;
  if (g_tdm_enabled) return true;

  esp_err_t err = i2s_channel_enable(g_tx_handle);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_enable error: %d\n", err);
    return false;
  }

  g_tdm_enabled = true;
  return true;
}

static void tdm_disable_output() {
  if (!g_tx_handle) return;
  if (!g_tdm_enabled) return;

  esp_err_t err = i2s_channel_disable(g_tx_handle);
  g_tdm_enabled = false;
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_disable error: %d\n", err);
    return;
  }
}

// ====================================================
// I2S/TDM bench test (CH1 tone + CH8 RAW tag 0xAAAA)
// ====================================================
static bool g_i2sTestActive = false;
static uint32_t g_i2sTestUntilMs = 0;
static uint32_t g_i2sTestPhase = 0;

static int16_t i2sTestWaveSample(uint32_t phase) {
  constexpr uint32_t kPeriod = 100;  // ~440 Hz @ 44100
  return ((phase % kPeriod) < (kPeriod / 2)) ? 6000 : -6000;
}

static bool i2sTestStart(uint32_t seconds) {
  if (seconds == 0) {
    seconds = 5;
  }
  stopAllPlayback();
  i2cModeSet(TdmDataMode::RAW);
  i2cModeProcess();
#if UZU_ENABLE_I2C
  i2cHostBroadcastSetMode(TdmDataMode::RAW);
#endif
  if (!tdm_set_sample_rate(TDM_DEFAULT_SAMPLE_RATE)) {
    Serial.println("[I2STEST] ERR TDM rate");
    return false;
  }
  if (!tdm_enable_output()) {
    Serial.println("[I2STEST] ERR TDM enable");
    return false;
  }
  g_i2sTestPhase = 0;
  g_i2sTestUntilMs = millis() + (seconds * 1000UL);
  g_i2sTestActive = true;
  Serial.printf("[I2STEST] started %lu sec (CH1=440Hz CH8=0xAAAA)\n",
                (unsigned long)seconds);
  return true;
}

static void i2sTestStop() {
  if (!g_i2sTestActive) {
    return;
  }
  g_i2sTestActive = false;
  tdm_disable_output();
  Serial.println("[I2STEST] stopped");
}

static void i2sTestDumpTdmReference() {
  constexpr uint32_t kDumpTdmFrames = 32;
  Serial.println("=== TDM TX DUMP BEGIN ===");
  Serial.println("tdm_idx,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8");
  for (uint32_t t = 0; t < kDumpTdmFrames; t++) {
    const int16_t ch1 = i2sTestWaveSample(t);
    Serial.printf("%lu,%d,0,0,0,0,0,0,%04X\n",
                  (unsigned long)t,
                  (int)ch1,
                  (unsigned)(uint16_t)I2S_TAG_RAW_TDM);
  }
  Serial.println("=== TDM AS-STEREO (4 pairs / TDM frame) BEGIN ===");
  Serial.println("st_idx,slot0,slot1");
  uint32_t st = 0;
  for (uint32_t t = 0; t < kDumpTdmFrames; t++) {
    const int16_t td[8] = {
        i2sTestWaveSample(t), 0, 0, 0, 0, 0, 0, I2S_TAG_RAW_TDM};
    for (uint32_t p = 0; p < 4; p++) {
      Serial.printf("%lu,%d,%d\n",
                    (unsigned long)st,
                    (int)td[p * 2],
                    (int)td[p * 2 + 1]);
      st++;
    }
  }
  Serial.println("=== TDM TX DUMP END ===");
}

static void i2sTestProcess() {
  if (!g_i2sTestActive) {
    return;
  }
  if ((int32_t)(millis() - g_i2sTestUntilMs) >= 0) {
    i2sTestStop();
    return;
  }
  if (!g_tx_handle) {
    i2sTestStop();
    return;
  }

  for (uint32_t f = 0; f < FRAMES_PER_WRITE; ++f) {
    const uint32_t dstBase = f * TDM_NUM_CH;
    for (uint32_t ch = 0; ch < TDM_NUM_CH; ++ch) {
      g_tdmTxBuf[dstBase + ch] = 0;
    }
    g_tdmTxBuf[dstBase + 0] = i2sTestWaveSample(g_i2sTestPhase + f);
    g_tdmTxBuf[dstBase + 7] = I2S_TAG_RAW_TDM;
  }
  g_i2sTestPhase += FRAMES_PER_WRITE;

  size_t bytesWritten = 0;
  const esp_err_t err = i2s_channel_write(
      g_tx_handle,
      g_tdmTxBuf,
      FRAMES_PER_WRITE * BYTES_PER_TDM_FRAME,
      &bytesWritten,
      portMAX_DELAY);
  if (err != ESP_OK) {
    Serial.printf("[I2STEST] write error: %d\n", (int)err);
    i2sTestStop();
  }
}

// ====================================================
// Stream mode (V3)
// ====================================================
enum class DeviceMode {
  SD = 0,
  STREAM
};

enum class StreamState {
  IDLE = 0,
  PREPARED,
  BUFFERING,
  PLAYING,
  PAUSED,
  STOPPED,
  ERROR
};

static DeviceMode g_deviceMode = DeviceMode::SD;

// Browser UZU stream debug (serial log). Paste [STREAM-DBG] lines when reporting issues.
#ifndef UZU_STREAM_DBG
#define UZU_STREAM_DBG 1
#endif

#if UZU_STREAM_DBG
static uint32_t g_streamDbgBinRx = 0;
static uint32_t g_streamDbgBinOk = 0;
static uint32_t g_streamDbgBinRejectMode = 0;
static uint32_t g_streamDbgBinRejectState = 0;
static uint32_t g_streamDbgBinRejectLen = 0;
static uint32_t g_streamDbgLastSummaryMs = 0;

static const char* deviceModeName(DeviceMode mode) {
  return (mode == DeviceMode::STREAM) ? "STREAM" : "SD";
}

static void streamDbgResetCounters() {
  g_streamDbgBinRx = 0;
  g_streamDbgBinOk = 0;
  g_streamDbgBinRejectMode = 0;
  g_streamDbgBinRejectState = 0;
  g_streamDbgBinRejectLen = 0;
  g_streamDbgLastSummaryMs = 0;
}
#endif

class PcmRingBuffer {
public:
  static constexpr size_t PSRAM_TARGET_BYTES = 1048576;
  static constexpr size_t PSRAM_FALLBACK_BYTES = 393216;
  static constexpr size_t INTERNAL_FALLBACK_BYTES = 49152;

  bool init() {
    if (m_buf) return true;

    m_capacity = PSRAM_TARGET_BYTES;
    m_buf = (uint8_t*)heap_caps_malloc(m_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (m_buf) {
      Serial.printf("[STREAM] ring %u bytes (PSRAM)\n", (unsigned)m_capacity);
      reset();
      return true;
    }

    m_capacity = PSRAM_FALLBACK_BYTES;
    m_buf = (uint8_t*)heap_caps_malloc(m_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (m_buf) {
      Serial.printf("[STREAM] ring %u bytes (PSRAM fallback)\n", (unsigned)m_capacity);
      reset();
      return true;
    }

    m_capacity = INTERNAL_FALLBACK_BYTES;
    m_buf = (uint8_t*)heap_caps_malloc(m_capacity, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (m_buf) {
      Serial.printf("[STREAM] ring %u bytes (internal fallback)\n", (unsigned)m_capacity);
      reset();
      return true;
    }

    Serial.println("[STREAM] ring buffer alloc FAILED");
    m_capacity = 0;
    return false;
  }

  size_t capacity() const { return m_capacity; }

  void reset() {
    m_head = 0;
    m_tail = 0;
  }

  size_t usedBytes() const {
    if (!m_buf || m_capacity <= 1) return 0;
    if (m_head >= m_tail) return m_head - m_tail;
    return m_capacity - m_tail + m_head;
  }

  size_t freeBytes() const {
    if (!m_buf || m_capacity <= 1) return 0;
    return m_capacity - usedBytes() - 1;
  }

  size_t write(const uint8_t* data, size_t len) {
    if (!m_buf) return 0;
    if (len > freeBytes()) return 0;

    for (size_t i = 0; i < len; ++i) {
      m_buf[m_head] = data[i];
      m_head = (m_head + 1) % m_capacity;
    }
    return len;
  }

  size_t read(uint8_t* out, size_t len) {
    if (!m_buf) return 0;
    size_t avail = usedBytes();
    size_t n = (len < avail) ? len : avail;
    for (size_t i = 0; i < n; ++i) {
      out[i] = m_buf[m_tail];
      m_tail = (m_tail + 1) % m_capacity;
    }
    return n;
  }

private:
  uint8_t* m_buf = nullptr;
  size_t m_capacity = 0;
  size_t m_head = 0;
  size_t m_tail = 0;
};

static PcmRingBuffer g_streamRing;
static uint8_t g_streamPcmBuf[FRAMES_PER_WRITE * TDM_NUM_CH * 2];
static uint32_t g_streamPendingSampleRate = TDM_DEFAULT_SAMPLE_RATE;
static uint32_t g_streamPendingChannels = 2;
static uint32_t g_streamPendingBits = 16;
static uint32_t g_streamDroppedBytes = 0;
static uint32_t g_streamRxBytes = 0;
static uint32_t g_lastDropLogMs = 0;

static uint32_t streamMinStartBytes() {
  const size_t cap = g_streamRing.capacity();
  if (cap == 0) return 16384;
  return (uint32_t)(cap / 3);
}

static const char* streamStateToString(StreamState st) {
  switch (st) {
    case StreamState::IDLE:      return "IDLE";
    case StreamState::PREPARED:  return "PREPARED";
    case StreamState::BUFFERING: return "BUFFERING";
    case StreamState::PLAYING:   return "PLAYING";
    case StreamState::PAUSED:    return "PAUSED";
    case StreamState::STOPPED:   return "STOPPED";
    case StreamState::ERROR:     return "ERROR";
    default:                     return "UNKNOWN";
  }
}

class StreamEngine {
public:
  StreamState state() const { return m_state; }
  uint32_t channelCount() const { return m_channels; }
  uint32_t underrunCount() const { return m_underrunCount; }
  uint32_t droppedBytes() const { return g_streamDroppedBytes; }
  uint32_t rxBytes() const { return g_streamRxBytes; }

  void reset() {
    stopInternal(true);
    m_state = StreamState::IDLE;
    g_streamRxBytes = 0;
  }

  bool prepare(uint32_t sampleRate, uint32_t channels, uint32_t bits) {
    if (bits != 16 || channels == 0 || channels > TDM_NUM_CH) {
      m_state = StreamState::ERROR;
      return false;
    }
    if (sampleRate != TDM_DEFAULT_SAMPLE_RATE && sampleRate != TDM_ALT_SAMPLE_RATE) {
      m_state = StreamState::ERROR;
      return false;
    }

    if (!g_streamRing.init()) {
      m_state = StreamState::ERROR;
      return false;
    }

    g_streamRing.reset();
    g_streamDroppedBytes = 0;
    g_streamRxBytes = 0;
    m_sampleRate = sampleRate;
    m_channels = channels;
    m_bits = bits;
    m_underrunSent = false;
    m_underrunCount = 0;

    if (!tdm_set_sample_rate(sampleRate)) {
      m_state = StreamState::ERROR;
      return false;
    }

    m_state = StreamState::PREPARED;
    m_playRequested = false;
    return true;
  }

  bool start() {
    if (m_state != StreamState::PREPARED &&
        m_state != StreamState::BUFFERING &&
        m_state != StreamState::PAUSED) {
      return false;
    }
    m_playRequested = true;
    if (m_state == StreamState::PAUSED) {
      if (!tdm_enable_output()) {
        m_state = StreamState::ERROR;
        return false;
      }
      m_state = StreamState::PLAYING;
      m_underrunSent = false;
      m_streamNextBlockUs = micros();
      return true;
    }
    m_state = StreamState::BUFFERING;
    m_underrunSent = false;
    return true;
  }

  void stop() {
    stopInternal(true);
  }

  void pause() {
    if (m_state != StreamState::PLAYING) return;
    tdm_disable_output();
    m_state = StreamState::PAUSED;
  }

  void resume() {
    if (m_state != StreamState::PAUSED) return;
    if (!tdm_enable_output()) {
      m_state = StreamState::ERROR;
      return;
    }
    m_state = StreamState::PLAYING;
    m_underrunSent = false;
    m_streamNextBlockUs = micros();
  }

  void resetBuffer() {
    g_streamRing.reset();
    g_streamDroppedBytes = 0;
    g_streamRxBytes = 0;
    m_underrunSent = false;
    m_underrunCount = 0;
  }

  size_t pushPcm(const uint8_t* data, size_t len) {
    if (g_deviceMode != DeviceMode::STREAM) {
#if UZU_STREAM_DBG
      g_streamDbgBinRejectMode++;
      if (g_streamDbgBinRejectMode <= 3) {
        Serial.printf("[STREAM-DBG] pushPcm reject: devMode=%s (need STREAM)\n",
                      deviceModeName(g_deviceMode));
      }
#endif
      return 0;
    }
    if (m_state != StreamState::PREPARED &&
        m_state != StreamState::BUFFERING &&
        m_state != StreamState::PLAYING &&
        m_state != StreamState::PAUSED) {
#if UZU_STREAM_DBG
      g_streamDbgBinRejectState++;
      if (g_streamDbgBinRejectState <= 3) {
        Serial.printf("[STREAM-DBG] pushPcm reject: state=%s\n",
                      streamStateToString(m_state));
      }
#endif
      return 0;
    }

    if (m_state == StreamState::PREPARED) {
      m_state = StreamState::BUFFERING;
    }

    const size_t fullChunk = (size_t)FRAMES_PER_WRITE * m_channels * 2;
    if (len != fullChunk) {
#if UZU_STREAM_DBG
      g_streamDbgBinRejectLen++;
      if (g_streamDbgBinRejectLen <= 5) {
        Serial.printf("[STREAM-DBG] pushPcm reject: len=%u expect=%u (ch=%u)\n",
                      (unsigned)len, (unsigned)fullChunk, (unsigned)m_channels);
      }
#endif
      return 0;
    }

    size_t written = g_streamRing.write(data, len);
    if (written > 0) {
      g_streamRxBytes += (uint32_t)written;
    }
    if (written == 0 && len > 0) {
      g_streamDroppedBytes += (uint32_t)len;
      uint32_t nowDrop = millis();
      if ((nowDrop - g_lastDropLogMs) >= 1000) {
        g_lastDropLogMs = nowDrop;
        Serial.printf("[STREAM] drop %u bytes (free=%u total=%lu)\n",
                      (unsigned)len, (unsigned)g_streamRing.freeBytes(),
                      (unsigned long)g_streamDroppedBytes);
      }
      wsBroadcastBufferInfo();
    }
    return written;
  }

  void process() {
    if (m_state == StreamState::BUFFERING && m_playRequested) {
      if (g_streamRing.usedBytes() < streamMinStartBytes()) {
        return;
      }
      if (!tdm_enable_output()) {
        m_state = StreamState::ERROR;
        wsRequestStreamStatus();
        return;
      }
      m_state = StreamState::PLAYING;
      m_streamNextBlockUs = micros();
      wsRequestStreamStatus();
      Serial.printf("[STREAM] PLAYING (buf=%u bytes, %uch)\n",
                    (unsigned)g_streamRing.usedBytes(),
                    (unsigned)m_channels);
#if UZU_STREAM_DBG
      streamDbgSummary(true);
#endif
    }

    if (m_state != StreamState::PLAYING) return;

    const uint32_t burstMax = 4U;
    for (uint32_t i = 0; i < burstMax; i++) {
      if (!processOneAudioBlock()) break;
    }

    const uint32_t now = millis();
    if ((now - m_lastStreamLogMs) >= 2000) {
      m_lastStreamLogMs = now;
      Serial.printf("[STREAM] buf used=%u free=%u rx=%lu underrun=%lu drop=%lu\n",
                    (unsigned)g_streamRing.usedBytes(),
                    (unsigned)g_streamRing.freeBytes(),
                    (unsigned long)g_streamRxBytes,
                    (unsigned long)m_underrunCount,
                    (unsigned long)g_streamDroppedBytes);
    }
  }

private:
  bool processOneAudioBlock() {
    const uint32_t bytesPerFrame = m_channels * 2;
    const uint32_t wantBytes = FRAMES_PER_WRITE * bytesPerFrame;
    const uint32_t blockUs = (uint32_t)((uint64_t)FRAMES_PER_WRITE * 1000000ULL / m_sampleRate);
    const uint32_t nowUs = micros();

    if ((int32_t)(nowUs - (uint32_t)m_streamNextBlockUs) < 0) {
      return false;
    }

    const uint32_t avail = (uint32_t)g_streamRing.usedBytes();
    const uint32_t now = millis();

    if ((int32_t)(nowUs - (uint32_t)m_streamNextBlockUs) < 0) {
      return false;
    }

    if ((uint32_t)(nowUs - (uint32_t)m_streamNextBlockUs) > blockUs * 4) {
      m_streamNextBlockUs = nowUs;
    }
    m_streamNextBlockUs += blockUs;

    for (uint32_t i = 0; i < FRAMES_PER_WRITE * TDM_NUM_CH; ++i) {
      g_tdmTxBuf[i] = 0;
    }

    uint32_t framesFilled = 0;
    bool hadUnderrun = false;

    if (avail >= wantBytes) {
      size_t got = g_streamRing.read(g_streamPcmBuf, wantBytes);
      if (got >= wantBytes) {
        framesFilled = FRAMES_PER_WRITE;
      } else {
        hadUnderrun = true;
        if (got >= bytesPerFrame) {
          framesFilled = (uint32_t)(got / bytesPerFrame);
        }
        if (!m_underrunSent && got < wantBytes) {
          Serial.printf("[STREAM] underrun (short read got=%u)\n", (unsigned)got);
        }
      }
    } else if (avail >= bytesPerFrame) {
      const uint32_t aligned = (avail / bytesPerFrame) * bytesPerFrame;
      size_t got = g_streamRing.read(g_streamPcmBuf, aligned);
      framesFilled = (uint32_t)(got / bytesPerFrame);
      hadUnderrun = true;
    } else {
      if (avail > 0) {
        uint8_t trash[1024];
        g_streamRing.read(trash, avail);
      }
      hadUnderrun = true;
    }

    if (framesFilled > 0) {
      for (uint32_t f = 0; f < framesFilled; ++f) {
        uint32_t srcBase = f * bytesPerFrame;
        uint32_t dstBase = f * TDM_NUM_CH;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
          int16_t s = le16_to_s16(&g_streamPcmBuf[srcBase + ch * 2]);
          g_tdmTxBuf[dstBase + ch] = apply_volume_127(s, g_volume);
        }
      }
    }

    if (hadUnderrun) {
      m_underrunCount++;
      if (!m_underrunSent) {
        wsRequestUnderrun();
        m_underrunSent = true;
      }
      if ((now - m_lastUnderrunLogMs) >= 1000) {
        m_lastUnderrunLogMs = now;
        Serial.printf("[STREAM] underrun (avail=%u need=%u total=%lu)\n",
                      (unsigned)avail,
                      (unsigned)wantBytes,
                      (unsigned long)m_underrunCount);
      }
    } else {
      m_underrunSent = false;
    }

    if (i2cModeGet() == TdmDataMode::RAW) {
      tdmApplyRawTagCh8(FRAMES_PER_WRITE);
    }

    size_t bytesWritten = 0;
    esp_err_t err = i2s_channel_write(
        g_tx_handle,
        g_tdmTxBuf,
        FRAMES_PER_WRITE * BYTES_PER_TDM_FRAME,
        &bytesWritten,
        portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("[STREAM] i2s write error: %d\n", err);
      stopInternal(true);
      m_state = StreamState::ERROR;
      wsRequestStreamStatus();
      return false;
    }
    return true;
  }

  StreamState m_state = StreamState::IDLE;
  uint32_t m_sampleRate = TDM_DEFAULT_SAMPLE_RATE;
  uint32_t m_channels = 2;
  uint32_t m_bits = 16;
  bool m_underrunSent = false;
  uint32_t m_underrunCount = 0;
  uint32_t m_lastStreamLogMs = 0;
  uint32_t m_lastUnderrunLogMs = 0;
  bool m_playRequested = false;
  uint64_t m_streamNextBlockUs = 0;

  void stopInternal(bool clearBuffer) {
    tdm_disable_output();
    if (clearBuffer) {
      g_streamRing.reset();
      g_streamDroppedBytes = 0;
    }
    m_underrunSent = false;
    m_underrunCount = 0;
    m_playRequested = false;
    m_state = StreamState::STOPPED;
    m_state = StreamState::IDLE;
  }
};

static StreamEngine g_streamEngine;
static uint32_t g_lastBinBufferInfoMs = 0;
static TaskHandle_t g_streamTaskHandle = nullptr;
static TaskHandle_t g_playerTaskHandle = nullptr;

#if UZU_STREAM_DBG
static void streamDbgSummary(bool force) {
  const uint32_t now = millis();
  if (!force && (uint32_t)(now - g_streamDbgLastSummaryMs) < 5000U) {
    return;
  }
  g_streamDbgLastSummaryMs = now;
  Serial.printf(
      "[STREAM-DBG] devMode=%s engine=%s tdmEn=%d i2c=%s "
      "pending=%luHz/%uch/%ubit ring=%u/%u rxB=%lu drop=%lu underrun=%lu "
      "bin(rx=%lu ok=%lu rejMode=%lu rejState=%lu rejLen=%lu)\n",
      deviceModeName(g_deviceMode),
      streamStateToString(g_streamEngine.state()),
      (int)g_tdm_enabled,
      tdmDataModeName(i2cModeGet()),
      (unsigned long)g_streamPendingSampleRate,
      (unsigned long)g_streamPendingChannels,
      (unsigned long)g_streamPendingBits,
      (unsigned)g_streamRing.usedBytes(),
      (unsigned)g_streamRing.capacity(),
      (unsigned long)g_streamEngine.rxBytes(),
      (unsigned long)g_streamEngine.droppedBytes(),
      (unsigned long)g_streamEngine.underrunCount(),
      (unsigned long)g_streamDbgBinRx,
      (unsigned long)g_streamDbgBinOk,
      (unsigned long)g_streamDbgBinRejectMode,
      (unsigned long)g_streamDbgBinRejectState,
      (unsigned long)g_streamDbgBinRejectLen);
}
#endif

static void streamPlaybackTask(void* param) {
  (void)param;
  for (;;) {
    if (g_deviceMode == DeviceMode::STREAM) {
      g_streamEngine.process();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void ensureStreamPlaybackTask() {
  if (g_streamTaskHandle) return;
  xTaskCreatePinnedToCore(
      streamPlaybackTask,
      "streamAudio",
      4096,
      nullptr,
      5,
      &g_streamTaskHandle,
      1);
}

static int jsonExtractInt(const String& msg, const char* key, int defVal) {
  String needle = String("\"") + key + "\":";
  int p = msg.indexOf(needle);
  if (p < 0) return defVal;
  return msg.substring(p + needle.length()).toInt();
}

// ====================================================
// Player
// ====================================================
class UzuTdmPlayer {
public:
  bool begin() {
    return i2s_tdm_tx_init(TDM_DEFAULT_SAMPLE_RATE);
  }

  void process() {
    if (m_state != PlayerState::PLAY) return;
    if (!m_file || m_channels == 0) {
      stop();
      return;
    }

    // TDM output is always 44.1 kHz. Resample 48 kHz UZU files on Main before transmit.
    if (m_sampleRate == TDM_OUT_SAMPLE_RATE) {
      const uint32_t bytesPerSourceFrame = m_channels * 2;
      const uint32_t wantBytes = FRAMES_PER_WRITE * bytesPerSourceFrame;

      size_t n = m_file.read(g_pcmReadBuf, wantBytes);
      if (n == 0) {
        stop();
        return;
      }

      uint32_t framesRead = (uint32_t)(n / bytesPerSourceFrame);
      if (framesRead == 0) {
        stop();
        return;
      }

      for (uint32_t i = 0; i < FRAMES_PER_WRITE * TDM_NUM_CH; ++i) {
        g_tdmTxBuf[i] = 0;
      }

      for (uint32_t f = 0; f < framesRead; ++f) {
        uint32_t srcBase = f * bytesPerSourceFrame;
        uint32_t dstBase = f * TDM_NUM_CH;
        const uint32_t mapCh = (m_channels > TDM_AUDIO_CH) ? TDM_AUDIO_CH : m_channels;

        for (uint32_t ch = 0; ch < mapCh; ++ch) {
          int16_t s = le16_to_s16(&g_pcmReadBuf[srcBase + ch * 2]);
          g_tdmTxBuf[dstBase + ch] = apply_volume_127(s, g_volume);
        }
      }
      tdmApplyRawTagCh8(FRAMES_PER_WRITE);

      size_t bytesToWrite = FRAMES_PER_WRITE * BYTES_PER_TDM_FRAME;
      size_t bytesWritten = 0;

      esp_err_t err = i2s_channel_write(
          g_tx_handle,
          g_tdmTxBuf,
          bytesToWrite,
          &bytesWritten,
          portMAX_DELAY);

      if (err != ESP_OK) {
        Serial.printf("write error: %d\n", err);
        stop();
        return;
      }

      m_currentSample += framesRead;

      if (n < wantBytes || m_currentSample >= m_totalSamples) {
        stop();
      }
      return;
    }

    if (m_sampleRate != TDM_ALT_SAMPLE_RATE) {
      stop();
      return;
    }

    const uint32_t bytesPerSourceFrame = m_channels * 2;
    const uint32_t requiredFrames = SRC_BUFFER_MAX_FRAMES;

    while (m_srcBufFramesAvail < requiredFrames) {
      if (m_srcBufAbsStartSample + m_srcBufFramesAvail >= m_totalSamples) break;

      uint32_t remainingTrackFrames =
          m_totalSamples - (m_srcBufAbsStartSample + m_srcBufFramesAvail);
      uint32_t wantFrames = requiredFrames - m_srcBufFramesAvail;
      uint32_t freeFrames = SRC_BUFFER_MAX_FRAMES - m_srcBufFramesAvail;
      wantFrames = (wantFrames < freeFrames) ? wantFrames : freeFrames;
      if (wantFrames == 0) break;
      if (wantFrames > remainingTrackFrames) wantFrames = remainingTrackFrames;
      if (wantFrames == 0) break;

      size_t wantBytes = (size_t)wantFrames * bytesPerSourceFrame;
      uint8_t* dst = g_pcmReadBuf + (size_t)m_srcBufFramesAvail * bytesPerSourceFrame;
      size_t n = m_file.read(dst, wantBytes);
      if (n == 0) break;

      uint32_t framesRead = (uint32_t)(n / bytesPerSourceFrame);
      if (framesRead == 0) break;
      m_srcBufFramesAvail += framesRead;
    }

    for (uint32_t i = 0; i < FRAMES_PER_WRITE * TDM_NUM_CH; ++i) {
      g_tdmTxBuf[i] = 0;
    }

    uint32_t produced = 0;
    uint32_t srcPos = 0;
    uint32_t phase = m_resample_phase_q16;

    while (produced < FRAMES_PER_WRITE) {
      if (srcPos + 1 >= m_srcBufFramesAvail) break;

      uint16_t frac = (uint16_t)(phase & 0xFFFFu);

      uint32_t dstBase = produced * TDM_NUM_CH;
      uint32_t src0Base = srcPos * bytesPerSourceFrame;
      uint32_t src1Base = (srcPos + 1) * bytesPerSourceFrame;

      for (uint32_t ch = 0; ch < m_channels; ++ch) {
        int16_t s0 = le16_to_s16(&g_pcmReadBuf[src0Base + ch * 2]);
        int16_t s1 = le16_to_s16(&g_pcmReadBuf[src1Base + ch * 2]);
        int16_t sm = lerp16_q16(s0, s1, frac);
        if (ch < TDM_AUDIO_CH) {
          g_tdmTxBuf[dstBase + ch] = apply_volume_127(sm, g_volume);
        }
      }

      produced++;

      phase += m_resample_step_q16;
      uint32_t advance = (phase >> 16);
      phase &= 0xFFFFu;
      srcPos += advance;
    }

    const uint32_t consumedFrames = srcPos;
    if (consumedFrames > 0) {
      uint32_t remaining = m_srcBufFramesAvail - consumedFrames;
      if (remaining > 0) {
        memmove(
            g_pcmReadBuf,
            g_pcmReadBuf + (size_t)consumedFrames * bytesPerSourceFrame,
            (size_t)remaining * bytesPerSourceFrame);
      }
      m_srcBufFramesAvail = remaining;
      m_srcBufAbsStartSample += consumedFrames;
      m_currentSample = m_srcBufAbsStartSample;
    }
    m_resample_phase_q16 = phase;

    if (produced == 0) {
      stop();
      return;
    }

    tdmApplyRawTagCh8(FRAMES_PER_WRITE);

    size_t bytesToWrite = FRAMES_PER_WRITE * BYTES_PER_TDM_FRAME;
    size_t bytesWritten = 0;

    esp_err_t err = i2s_channel_write(
        g_tx_handle,
        g_tdmTxBuf,
        bytesToWrite,
        &bytesWritten,
        portMAX_DELAY);

    if (err != ESP_OK) {
      Serial.printf("write error: %d\n", err);
      stop();
      return;
    }

    if (produced < FRAMES_PER_WRITE || m_currentSample >= m_totalSamples) {
      stop();
    }
  }

  bool playIndex(int index) {
    if (index <= 0 || index > g_fileCount) {
      m_lastError = "ERR BAD_INDEX";
      return false;
    }

    g_streamEngine.stop();
    g_deviceMode = DeviceMode::SD;
    stop();

    File f = SD_MMC.open(g_fileList[index - 1].path, FILE_READ);
    if (!f) {
      m_lastError = "ERR FILE_OPEN";
      return false;
    }

    if (!isUzuPcmFile(f)) {
      f.close();
      m_lastError = "ERR BAD_FORMAT";
      return false;
    }

    uint32_t dataStart = 0;
    uint32_t channels = 0;
    uint32_t sampleRate = 0;
    uint32_t bits = 0;
    uint32_t totalSamples = 0;

    if (!read_u32_at(f, 8, dataStart) ||
        !read_u32_at(f, 12, channels) ||
        !read_u32_at(f, 16, sampleRate) ||
        !read_u32_at(f, 20, bits) ||
        !read_u32_at(f, 24, totalSamples)) {
      f.close();
      m_lastError = "ERR BAD_FORMAT";
      return false;
    }

    if (dataStart < 32768 || channels == 0 || channels > 8 || (sampleRate != TDM_DEFAULT_SAMPLE_RATE && sampleRate != TDM_ALT_SAMPLE_RATE) || bits != 16) {
      f.close();
      if (channels > 8) m_lastError = "ERR UNSUPPORTED_CH";
      else if (sampleRate != TDM_DEFAULT_SAMPLE_RATE && sampleRate != TDM_ALT_SAMPLE_RATE) m_lastError = "ERR UNSUPPORTED_RATE";
      else m_lastError = "ERR BAD_FORMAT";
      return false;
    }

    if (!f.seek(dataStart)) {
      f.close();
      m_lastError = "ERR FILE_SEEK";
      return false;
    }

    if (!tdm_set_sample_rate(TDM_OUT_SAMPLE_RATE)) {
      f.close();
      m_lastError = "ERR TDM_RATE";
      return false;
    }

    if (!tdm_enable_output()) {
      f.close();
      m_lastError = "ERR TDM_ENABLE";
      return false;
    }

    m_file = f;
    m_state = PlayerState::PLAY;
    m_currentIndex = index;
    m_dataStart = dataStart;
    m_channels = channels;
    m_sampleRate = sampleRate;
    m_bits = bits;
    m_totalSamples = totalSamples;
    m_lengthMs = (uint32_t)(((uint64_t)totalSamples * 1000ULL) / sampleRate);
    m_currentSample = 0;
    m_lastError = nullptr;

    m_srcBufFramesAvail = 0;
    m_srcBufAbsStartSample = 0;
    m_resample_phase_q16 = 0;
    m_resample_step_q16 = 0;
    if (m_sampleRate != TDM_OUT_SAMPLE_RATE) {
      m_resample_step_q16 = (uint32_t)(((uint64_t)m_sampleRate << 16) / TDM_OUT_SAMPLE_RATE);
      Serial.printf("PLAY resample %lu -> %lu Hz\n",
                    (unsigned long)m_sampleRate,
                    (unsigned long)TDM_OUT_SAMPLE_RATE);
    }

    g_selectedTrack0 = index - 1;
    g_selectedLenMs = m_lengthMs;

    Serial.printf("PLAY open: ch=%lu rate=%lu bits=%lu samples=%lu dataStart=%lu vol=%u (TDM CH8=RAW)\n",
                  (unsigned long)m_channels,
                  (unsigned long)m_sampleRate,
                  (unsigned long)m_bits,
                  (unsigned long)m_totalSamples,
                  (unsigned long)m_dataStart,
                  (unsigned)g_volume);
    if (m_channels > TDM_AUDIO_CH) {
      Serial.printf("PLAY note: file %lu ch -> TDM CH1-%lu (CH8=0xAAAA tag)\n",
                    (unsigned long)m_channels, (unsigned long)TDM_AUDIO_CH);
    }

    return true;
  }

  bool resume() {
    if (m_state == PlayerState::PAUSE && m_file) {
      if (!tdm_enable_output()) {
        m_lastError = "ERR TDM_ENABLE";
        return false;
      }
      m_state = PlayerState::PLAY;
      return true;
    }
    m_lastError = "ERR BAD_PARAM";
    return false;
  }

  void pause() {
    if (m_state == PlayerState::PLAY) {
      m_state = PlayerState::PAUSE;
      tdm_disable_output();
    }
  }

  void stop() {
    if (m_file) m_file.close();

    m_state = PlayerState::STOP;
    m_currentIndex = 0;
    m_dataStart = 0;
    m_channels = 0;
    m_sampleRate = 0;
    m_bits = 0;
    m_totalSamples = 0;
    m_lengthMs = 0;
    m_currentSample = 0;
    m_lastError = nullptr;

    m_srcBufFramesAvail = 0;
    m_srcBufAbsStartSample = 0;
    m_resample_phase_q16 = 0;
    m_resample_step_q16 = 0;

    tdm_disable_output();
  }

  bool seekMs(uint32_t ms) {
    if (!m_file || m_state == PlayerState::STOP || m_sampleRate == 0 || m_channels == 0) {
      m_lastError = "ERR BAD_PARAM";
      return false;
    }

    uint64_t targetSample64 = ((uint64_t)ms * (uint64_t)m_sampleRate) / 1000ULL;
    if (targetSample64 > m_totalSamples) targetSample64 = m_totalSamples;

    uint32_t targetSample = (uint32_t)targetSample64;
    uint64_t pos = (uint64_t)m_dataStart + ((uint64_t)targetSample * (uint64_t)m_channels * 2ULL);

    if (!m_file.seek((uint32_t)pos)) {
      m_lastError = "ERR FILE_SEEK";
      return false;
    }

    m_currentSample = targetSample;

    m_srcBufFramesAvail = 0;
    m_srcBufAbsStartSample = targetSample;
    m_resample_phase_q16 = 0;
    return true;
  }

  PlayerState state() const { return m_state; }
  int currentIndex() const { return m_currentIndex; }
  uint32_t positionMs() const {
    if (m_sampleRate == 0) return 0;
    return (uint32_t)(((uint64_t)m_currentSample * 1000ULL) / m_sampleRate);
  }
  uint32_t lengthMs() const { return m_lengthMs; }
  const char* lastError() const { return m_lastError ? m_lastError : "ERR INTERNAL"; }

private:
  File m_file;
  PlayerState m_state = PlayerState::STOP;
  int m_currentIndex = 0;
  uint32_t m_dataStart = 0;
  uint32_t m_channels = 0;
  uint32_t m_sampleRate = 0;
  uint32_t m_bits = 0;
  uint32_t m_totalSamples = 0;
  uint32_t m_lengthMs = 0;
  uint32_t m_currentSample = 0;
  const char* m_lastError = nullptr;

  uint32_t m_srcBufFramesAvail = 0;
  uint32_t m_srcBufAbsStartSample = 0;
  uint32_t m_resample_phase_q16 = 0;
  uint32_t m_resample_step_q16 = 0;
};

static UzuTdmPlayer g_player;
static UzcTdmPlayer g_uzcPlayer;
static bool g_uzcPlaying = false;

static void playerPlaybackTask(void* param) {
  (void)param;
  for (;;) {
    if (g_deviceMode != DeviceMode::STREAM && !g_uzcPlaying && !g_i2sTestActive) {
      if (g_player.state() == PlayerState::PLAY) {
        g_player.process();
      } else {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

static void ensurePlayerPlaybackTask() {
  if (g_playerTaskHandle) return;
  xTaskCreatePinnedToCore(
      playerPlaybackTask,
      "playerAudio",
      4096,
      nullptr,
      5,
      &g_playerTaskHandle,
      1);
}

static void stopAllPlayback() {
  i2sTestStop();
  g_uzcPlayer.stop();
  g_uzcPlaying = false;
  g_player.stop();
  g_streamEngine.stop();
}

static void streamBeginSession() {
#if UZU_STREAM_DBG
  Serial.printf("[STREAM-DBG] beginSession (before) devMode=%s i2c=%s engine=%s\n",
                deviceModeName(g_deviceMode),
                tdmDataModeName(i2cModeGet()),
                streamStateToString(g_streamEngine.state()));
#endif
  stopAllPlayback();
  tdm_disable_output();
  i2cModeSet(TdmDataMode::RAW);
  i2cModeProcess();
#if UZU_ENABLE_I2C
  i2cHostBroadcastSetMode(TdmDataMode::RAW);
  i2cHostBroadcastClearBuffer(0x00);
  Serial.println("[STREAM] I2C slaves -> RAW + buffer clear");
#endif
#if UZU_STREAM_DBG
  streamDbgResetCounters();
  Serial.printf("[STREAM-DBG] beginSession (after) i2c=%s tdmEn=%d\n",
                tdmDataModeName(i2cModeGet()),
                (int)g_tdm_enabled);
#endif
}

static bool playTrackByIndex(int index) {
  if (index <= 0 || index > g_fileCount) {
    return false;
  }

  g_streamEngine.stop();
  g_deviceMode = DeviceMode::SD;
  stopAllPlayback();

  const char* path = g_fileList[index - 1].path;

  if (isUzuPcmFile(path)) {
    i2cModeSet(TdmDataMode::RAW);
    i2cModeProcess();
#if UZU_ENABLE_I2C
    i2cHostBroadcastSetMode(TdmDataMode::RAW);
#endif
    Serial.printf("[PLAY] PCM .UZU TDM (RAW): %s\n", g_fileList[index - 1].name);
    return g_player.playIndex(index);
  }

  if (isUzcMp3File(path)) {
    i2cModeSet(TdmDataMode::MP3);
    i2cModeProcess();
#if UZU_ENABLE_I2C
    i2cHostBroadcastSetMode(TdmDataMode::MP3);
#endif
    if (!tdm_set_sample_rate(TDM_DEFAULT_SAMPLE_RATE)) {
      return false;
    }
    g_tdm_enabled = false;
    if (!tdm_enable_output()) {
      return false;
    }
    if (!g_uzcPlayer.begin(g_tx_handle)) {
      return false;
    }
    if (!g_uzcPlayer.playPath(path)) {
      Serial.println(g_uzcPlayer.lastError());
      tdm_disable_output();
      return false;
    }
    g_uzcPlaying = true;
    g_selectedTrack0 = index - 1;
    if (readUzcLengthMs(path, g_selectedLenMs)) {
      // track length for UI
    } else {
      g_selectedLenMs = 0;
    }
    Serial.printf("[PLAY] UZC MP3 TDM: %s\n", g_fileList[index - 1].name);
    return true;
  }

  Serial.println("[PLAY] ERR: unknown format (expected PCM .UZU or MP3 .uzc)");
  return false;
}

static void mountAndScanSdCard(bool verbose) {
  stopAllPlayback();
  clearFileList();
  g_selectedTrack0 = 0;
  g_selectedLenMs = 0;

  if (!isCardInserted()) {
    unmount_sdmmc();
    if (verbose) Serial.println("[SD] no card");
    wsBroadcastState();
    wsBroadcastTracks();
    return;
  }

  if (mount_sdmmc_4bit(g_sd_freq_hz)) {
    scanUzuFiles("/");
    if (verbose) {
      Serial.printf("[SD] mounted, %d file(s)\n", g_fileCount);
    }
  } else {
    clearFileList();
    if (verbose) Serial.println("[SD] mount failed");
  }

  wsBroadcastState();
  wsBroadcastTracks();
}

static void processSdCardDetect() {
  const uint32_t now = millis();

  if ((now - g_sdCdLastPollMs) < SD_CD_POLL_MS) return;
  g_sdCdLastPollMs = now;

  const bool read = isCardInserted();
  if (read != g_sdCdLastRead) {
    g_sdCdLastRead = read;
    g_sdCdStableSinceMs = now;
    return;
  }
  if ((now - g_sdCdStableSinceMs) < SD_CD_DEBOUNCE_MS) return;

  if (g_sdRemountPending && now >= g_sdRemountAtMs) {
    g_sdRemountPending = false;
    if (isCardInserted() && !g_mounted) {
      Serial.println("[SD] card inserted");
      mountAndScanSdCard(true);
      g_sdCardPresent = true;
    }
    return;
  }

  if (read == g_sdCardPresent) return;

  g_sdCardPresent = read;
  if (!read) {
    g_sdRemountPending = false;
    Serial.println("[SD] card removed");
    stopAllPlayback();
    unmount_sdmmc();
    clearFileList();
    g_selectedTrack0 = 0;
    g_selectedLenMs = 0;
    wsBroadcastState();
    wsBroadcastTracks();
    return;
  }

  g_sdRemountPending = true;
  g_sdRemountAtMs = now + SD_INSERT_SETTLE_MS;
}

// ====================================================
// Web helpers
// ====================================================
static bool webSdPlaybackBusy() {
  return g_uzcPlaying || g_player.state() != PlayerState::STOP;
}

static const char* webSdPlayStateString() {
  if (g_uzcPlaying) {
    if (!g_uzcPlayer.isActive()) {
      return "STOP";
    }
    return g_uzcPlayer.isPaused() ? "PAUSE" : "PLAY";
  }
  switch (g_player.state()) {
    case PlayerState::PLAY:  return "PLAY";
    case PlayerState::PAUSE: return "PAUSE";
    default:                 return "STOP";
  }
}

static void json_append_escaped(String& s, const char* p) {
  if (!p) return;
  while (*p) {
    char c = *p++;
    if (c == '\"') s += "\\\"";
    else if (c == '\\') s += "\\\\";
    else if ((uint8_t)c >= 0x20) s += c;
  }
}

static bool read_uzu_track_info(const char* path, WebTrackInfo& info) {
  memset(&info, 0, sizeof(info));
  if (!path || !path[0] || !g_mounted) return false;

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;

  bool ok = false;
  do {
    if (isUzcMp3File(path)) {
      uint32_t uzcCh = 0;
      uint32_t uzcRate = 0;
      if (!readUzcTrackInfo(path, uzcCh, uzcRate, info.lengthMs)) {
        break;
      }
      info.channels = uzcCh;
      info.sampleRate = uzcRate;
      info.bits = 16;
      info.totalSamples = 0;
      info.title[0] = '\0';
      const char* base = strrchr(path, '/');
      base = base ? (base + 1) : path;
      strncpy(info.path, path, sizeof(info.path) - 1);
      strncpy(info.name, base, sizeof(info.name) - 1);
      ok = true;
      break;
    }

    if (!check_uzu_magic(f)) break;

    uint32_t dataStart = 0;
    if (!read_u32_at(f, 8, dataStart) ||
        !read_u32_at(f, 12, info.channels) ||
        !read_u32_at(f, 16, info.sampleRate) ||
        !read_u32_at(f, 20, info.bits) ||
        !read_u32_at(f, 24, info.totalSamples)) {
      break;
    }

    if (dataStart < 32768 || info.channels == 0 || info.channels > 8 ||
        info.sampleRate == 0 || info.bits != 16) {
      break;
    }

    info.lengthMs = (uint32_t)(((uint64_t)info.totalSamples * 1000ULL) / info.sampleRate);

    const char* base = strrchr(path, '/');
    base = base ? (base + 1) : path;
    strncpy(info.path, path, sizeof(info.path) - 1);
    strncpy(info.name, base, sizeof(info.name) - 1);

    if (!read_string_at(f, 28, 256, info.title, sizeof(info.title))) {
      info.title[0] = '\0';
    }

    uint32_t count = info.channels;
    if (count > 8) count = 8;
    for (uint32_t i = 0; i < count; ++i) {
      uint32_t ofs = 284 + i * 256;
      if (!read_string_at(f, ofs, 256, info.chName[i], sizeof(info.chName[i]))) {
        info.chName[i][0] = '\0';
      }
    }

    ok = true;
  } while (0);

  f.close();
  return ok;
}

static void wsSendTracks(uint8_t clientId) {
  if (!webSdPlaybackBusy()) {
    scanUzuFiles("/");
  }

  String s = "{\"type\":\"tracks\",\"items\":[";
  for (int i = 0; i < g_fileCount; i++) {
    if (i) s += ",";
    s += "\"";
    json_append_escaped(s, g_fileList[i].name);
    s += "\"";
  }
  s += "]}";
  ws.sendTXT(clientId, s);
}

static void wsBroadcastTracks() {
  if (!webSdPlaybackBusy() && g_mounted) {
    scanUzuFiles("/");
  }

  String s = "{\"type\":\"tracks\",\"items\":[";
  for (int i = 0; i < g_fileCount; i++) {
    if (i) s += ",";
    s += "\"";
    json_append_escaped(s, g_fileList[i].name);
    s += "\"";
  }
  s += "]}";
  ws.broadcastTXT(s);
}

static void wsSendTrackInfo(uint8_t clientId, int trackIndex0) {
  if (!webSdPlaybackBusy()) {
    scanUzuFiles("/");
  }

  if (trackIndex0 < 0) trackIndex0 = 0;
  if (trackIndex0 >= g_fileCount) trackIndex0 = (g_fileCount > 0) ? (g_fileCount - 1) : 0;

  if (g_fileCount <= 0) {
    ws.sendTXT(clientId, "{\"type\":\"track_info\",\"valid\":false}");
    return;
  }

  WebTrackInfo info;
  strncpy(info.name, g_fileList[trackIndex0].name, sizeof(info.name) - 1);
  strncpy(info.path, g_fileList[trackIndex0].path, sizeof(info.path) - 1);

  String s = "{\"type\":\"track_info\",\"valid\":";
  if (!read_uzu_track_info(g_fileList[trackIndex0].path, info)) {
    s += "false,\"trackIndex\":";
    s += trackIndex0;
    s += ",\"name\":\"";
    json_append_escaped(s, g_fileList[trackIndex0].name);
    s += "\"}";
    ws.sendTXT(clientId, s);
    return;
  }

  s += "true,\"trackIndex\":";
  s += trackIndex0;
  s += ",\"name\":\"";
  json_append_escaped(s, g_fileList[trackIndex0].name);
  s += "\",\"title\":\"";
  json_append_escaped(s, info.title);
  s += "\",\"lengthMs\":";
  s += info.lengthMs;
  s += ",\"channels\":";
  s += info.channels;
  s += ",\"sampleRate\":";
  s += info.sampleRate;
  s += ",\"bits\":";
  s += info.bits;
  s += ",\"totalSamples\":";
  s += info.totalSamples;
  s += ",\"channelNames\":[";
  for (uint32_t i = 0; i < info.channels && i < 8; ++i) {
    if (i) s += ",";
    s += "\"";
    json_append_escaped(s, info.chName[i]);
    s += "\"";
  }
  s += "]}";
  ws.sendTXT(clientId, s);
}

static void wsSendState(uint8_t clientId) {
  uint32_t lenMs = g_selectedLenMs;
  uint32_t posMs = 0;
  if (g_uzcPlaying && g_uzcPlayer.isActive()) {
    lenMs = g_selectedLenMs;
  } else if (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE) {
    lenMs = g_player.lengthMs();
    posMs = g_player.positionMs();
  }

  String st = "{\"type\":\"state\",\"play\":\"";
  st += webSdPlayStateString();
  st += "\",\"trackIndex\":";
  st += g_selectedTrack0;
  st += ",\"posMs\":";
  st += posMs;
  st += ",\"lenMs\":";
  st += lenMs;
  st += ",\"vol\":";
  st += g_volume;
  st += "}";
  ws.sendTXT(clientId, st);
}

static void wsBroadcastState() {
  uint32_t lenMs = g_selectedLenMs;
  uint32_t posMs = 0;
  if (g_uzcPlaying && g_uzcPlayer.isActive()) {
    lenMs = g_selectedLenMs;
  } else if (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE) {
    lenMs = g_player.lengthMs();
    posMs = g_player.positionMs();
  }

  String st = "{\"type\":\"state\",\"play\":\"";
  st += webSdPlayStateString();
  st += "\",\"trackIndex\":";
  st += g_selectedTrack0;
  st += ",\"posMs\":";
  st += posMs;
  st += ",\"lenMs\":";
  st += lenMs;
  st += ",\"vol\":";
  st += g_volume;
  st += "}";
  ws.broadcastTXT(st);
}

static void wsBroadcastStreamStatus() {
  String st = "{\"cmd\":\"status\",\"state\":\"";
  st += streamStateToString(g_streamEngine.state());
  st += "\"}";
  ws.broadcastTXT(st);
}

static void wsBroadcastBufferInfo() {
  const size_t used = g_streamRing.usedBytes();
  const size_t freeb = g_streamRing.freeBytes();
  const size_t cap = g_streamRing.capacity();
  const unsigned fillPct = (cap > 0) ? (unsigned)((used * 100ULL) / cap) : 0;

  String st = "{\"cmd\":\"buffer_info\",\"freeBytes\":";
  st += (unsigned long)freeb;
  st += ",\"usedBytes\":";
  st += (unsigned long)used;
  st += ",\"bufferSize\":";
  st += (unsigned long)cap;
  st += ",\"fillPct\":";
  st += fillPct;
  st += ",\"underrunCount\":";
  st += (unsigned long)g_streamEngine.underrunCount();
  st += ",\"droppedBytes\":";
  st += (unsigned long)g_streamEngine.droppedBytes();
  const bool sendHold = (freeb < 4096) || (fillPct >= 88);
  st += ",\"sendHold\":";
  st += sendHold ? "true" : "false";
  st += "}";
  ws.broadcastTXT(st);
}

static void wsBroadcastUnderrun() {
  ws.broadcastTXT("{\"cmd\":\"underrun\"}");
}

static volatile bool g_wsNotifyBufferInfo = false;
static volatile bool g_wsNotifyStreamStatus = false;
static volatile bool g_wsNotifyUnderrun = false;

static void wsRequestStreamStatus() {
  g_wsNotifyStreamStatus = true;
}

static void wsRequestBufferInfo() {
  g_wsNotifyBufferInfo = true;
}

static void wsRequestUnderrun() {
  g_wsNotifyUnderrun = true;
  g_wsNotifyBufferInfo = true;
}

static void wsProcessNotifications() {
  static uint32_t s_lastBufferInfoMs = 0;
  const uint32_t now = millis();

  if (g_wsNotifyStreamStatus) {
    g_wsNotifyStreamStatus = false;
    wsBroadcastStreamStatus();
  }
  if (g_wsNotifyUnderrun) {
    g_wsNotifyUnderrun = false;
    wsBroadcastUnderrun();
    wsBroadcastBufferInfo();
    s_lastBufferInfoMs = now;
  }
  if (g_wsNotifyBufferInfo && (now - s_lastBufferInfoMs) >= 30) {
    g_wsNotifyBufferInfo = false;
    s_lastBufferInfoMs = now;
    wsBroadcastBufferInfo();
  }
}

static bool handleStreamTextCommand(uint8_t num, const String& msg) {
  if (msg.indexOf("\"cmd\":\"vol\"") >= 0) {
    int p = msg.indexOf("value");
    if (p >= 0) {
      int c = msg.indexOf(":", p);
      if (c >= 0) {
        int v = msg.substring(c + 1).toInt();
        if (v < 0) v = 0;
        if (v > 127) v = 127;
        g_volume = (uint8_t)v;
      }
    }
    wsBroadcastState();
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"set_mode\"") >= 0) {
    if (msg.indexOf("\"STREAM\"") >= 0) {
#if UZU_STREAM_DBG
      Serial.println("[STREAM-DBG] cmd set_mode STREAM");
#endif
      g_player.stop();
      g_uzcPlayer.stop();
      g_uzcPlaying = false;
      i2sTestStop();
      g_deviceMode = DeviceMode::STREAM;
      g_streamEngine.reset();
      wsBroadcastBufferInfo();
#if UZU_STREAM_DBG
      streamDbgSummary(true);
#endif
    } else {
#if UZU_STREAM_DBG
      Serial.println("[STREAM-DBG] cmd set_mode SD");
#endif
      g_streamEngine.stop();
      g_deviceMode = DeviceMode::SD;
    }
    wsBroadcastStreamStatus();
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"file_info\"") >= 0) {
    if (msg.indexOf("\"sampleRate\"") >= 0) {
      g_streamPendingSampleRate = (uint32_t)jsonExtractInt(msg, "sampleRate", TDM_DEFAULT_SAMPLE_RATE);
    }
    if (msg.indexOf("\"channels\"") >= 0) {
      g_streamPendingChannels = (uint32_t)jsonExtractInt(msg, "channels", 2);
    }
    if (msg.indexOf("\"bitsPerSample\"") >= 0) {
      g_streamPendingBits = (uint32_t)jsonExtractInt(msg, "bitsPerSample", 16);
    }
#if UZU_STREAM_DBG
    Serial.printf("[STREAM-DBG] cmd file_info %luHz %uch %ubit\n",
                  (unsigned long)g_streamPendingSampleRate,
                  (unsigned long)g_streamPendingChannels,
                  (unsigned long)g_streamPendingBits);
#endif
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"prepare\"") >= 0) {
#if UZU_STREAM_DBG
    Serial.printf("[STREAM-DBG] cmd prepare %luHz %uch %ubit i2c=%s\n",
                  (unsigned long)g_streamPendingSampleRate,
                  (unsigned long)g_streamPendingChannels,
                  (unsigned long)g_streamPendingBits,
                  tdmDataModeName(i2cModeGet()));
#endif
    streamBeginSession();
    if (!g_streamEngine.prepare(
            g_streamPendingSampleRate,
            g_streamPendingChannels,
            g_streamPendingBits)) {
#if UZU_STREAM_DBG
      Serial.printf("[STREAM-DBG] prepare FAILED -> engine=%s\n",
                    streamStateToString(g_streamEngine.state()));
      streamDbgSummary(true);
#endif
      ws.sendTXT(num, "{\"cmd\":\"error\",\"message\":\"Invalid format\"}");
      wsBroadcastStreamStatus();
      return true;
    }

    g_deviceMode = DeviceMode::STREAM;
#if UZU_STREAM_DBG
    Serial.printf("[STREAM-DBG] prepare OK engine=%s chunk=%uB\n",
                  streamStateToString(g_streamEngine.state()),
                  (unsigned)(FRAMES_PER_WRITE * g_streamPendingChannels * 2));
    streamDbgSummary(true);
#endif
    wsBroadcastStreamStatus();
    wsBroadcastBufferInfo();
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"start\"") >= 0) {
#if UZU_STREAM_DBG
    Serial.printf("[STREAM-DBG] cmd start (before) engine=%s ring=%u/%u\n",
                  streamStateToString(g_streamEngine.state()),
                  (unsigned)g_streamRing.usedBytes(),
                  (unsigned)g_streamRing.capacity());
#endif
    if (!g_streamEngine.start()) {
#if UZU_STREAM_DBG
      Serial.printf("[STREAM-DBG] start FAILED engine=%s\n",
                    streamStateToString(g_streamEngine.state()));
      streamDbgSummary(true);
#endif
      ws.sendTXT(num, "{\"cmd\":\"error\",\"message\":\"Start failed\"}");
      wsBroadcastStreamStatus();
      return true;
    }
#if UZU_STREAM_DBG
    Serial.printf("[STREAM-DBG] start OK engine=%s minStart=%lu\n",
                  streamStateToString(g_streamEngine.state()),
                  (unsigned long)streamMinStartBytes());
    streamDbgSummary(true);
#endif
    wsBroadcastStreamStatus();
    wsBroadcastBufferInfo();
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"stop\"") >= 0) {
    if (g_deviceMode == DeviceMode::STREAM) {
      g_streamEngine.stop();
      wsBroadcastStreamStatus();
      ws.sendTXT(num, "{\"type\":\"ack\"}");
      return true;
    }
    return false;
  }

  if (msg.indexOf("\"cmd\":\"pause\"") >= 0) {
    if (g_deviceMode == DeviceMode::STREAM) {
      g_streamEngine.pause();
      wsBroadcastStreamStatus();
      ws.sendTXT(num, "{\"type\":\"ack\"}");
      return true;
    }
    return false;
  }

  if (msg.indexOf("\"cmd\":\"resume\"") >= 0) {
    if (g_deviceMode == DeviceMode::STREAM) {
      g_streamEngine.resume();
      wsBroadcastStreamStatus();
      ws.sendTXT(num, "{\"type\":\"ack\"}");
      return true;
    }
    return false;
  }

  if (msg.indexOf("\"cmd\":\"reset_buffer\"") >= 0) {
    g_streamEngine.resetBuffer();
    wsBroadcastBufferInfo();
    ws.sendTXT(num, "{\"type\":\"ack\"}");
    return true;
  }

  if (msg.indexOf("\"cmd\":\"ping\"") >= 0) {
    ws.sendTXT(num, "{\"cmd\":\"pong\"}");
    return true;
  }

  return false;
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected\n", num);
      wsSendTracks(num);
      wsSendState(num);
      wsSendTrackInfo(num, g_selectedTrack0);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected", num);
      if (length >= 2) {
        const uint16_t reason = ((uint16_t)payload[0] << 8) | payload[1];
        Serial.printf(" code=%u", (unsigned)reason);
      }
      Serial.println();
      if (g_deviceMode == DeviceMode::STREAM && ws.connectedClients() == 0) {
        g_streamEngine.stop();
        g_deviceMode = DeviceMode::SD;
        Serial.println("[STREAM] stopped (client disconnected)");
      }
      break;

    case WStype_TEXT: {
      String msg;
      msg.reserve(length + 1);
      for (size_t i = 0; i < length; i++) msg += (char)payload[i];

      if (handleStreamTextCommand(num, msg)) return;

      if (msg.indexOf("\"cmd\":\"get_tracks\"") >= 0) {
        wsSendTracks(num);
        ws.sendTXT(num, "{\"type\":\"ack\"}");
        return;
      }

      if (msg.indexOf("\"cmd\":\"get_state\"") >= 0) {
        wsSendState(num);
        ws.sendTXT(num, "{\"type\":\"ack\"}");
        return;
      }

      if (msg.indexOf("\"cmd\":\"get_info\"") >= 0) {
        int trackIndex0 = g_selectedTrack0;
        int p = msg.indexOf("trackIndex");
        if (p >= 0) {
          int c = msg.indexOf(":", p);
          if (c >= 0) {
            trackIndex0 = msg.substring(c + 1).toInt();
          }
        }
        wsSendTrackInfo(num, trackIndex0);
        ws.sendTXT(num, "{\"type\":\"ack\"}");
        return;
      }

      if (msg.indexOf("\"cmd\":\"play\"") >= 0) {
        if (g_deviceMode != DeviceMode::STREAM && g_fileCount > 0) {
          if (g_uzcPlaying && g_uzcPlayer.isPaused()) {
            g_uzcPlayer.resume();
          } else if (g_player.state() == PlayerState::PAUSE) {
            g_player.resume();
          } else {
            playTrackByIndex(g_selectedTrack0 + 1);
          }
        }
      }
      else if (msg.indexOf("\"cmd\":\"stop\"") >= 0) {
        stopAllPlayback();
      }
      else if (msg.indexOf("\"cmd\":\"pause\"") >= 0) {
        if (g_uzcPlaying) {
          g_uzcPlayer.pause();
        } else {
          g_player.pause();
        }
      }
      else if (msg.indexOf("\"cmd\":\"select\"") >= 0) {
        if (!webSdPlaybackBusy()) {
          int p = msg.indexOf("trackIndex");
          if (p >= 0) {
            int c = msg.indexOf(":", p);
            if (c >= 0) {
              g_selectedTrack0 = msg.substring(c + 1).toInt();
              if (g_selectedTrack0 < 0) g_selectedTrack0 = 0;
              if (g_selectedTrack0 >= g_fileCount) g_selectedTrack0 = (g_fileCount > 0) ? (g_fileCount - 1) : 0;
            }
          }

          uint32_t lenMs = 0;
          if (g_fileCount > 0 && read_uzu_length_ms(g_fileList[g_selectedTrack0].path, lenMs)) {
            g_selectedLenMs = lenMs;
          } else {
            g_selectedLenMs = 0;
          }
        }

        wsSendState(num);
        ws.sendTXT(num, "{\"type\":\"ack\"}");
        return;
      }
      else if (msg.indexOf("\"cmd\":\"seek\"") >= 0) {
        int p = msg.indexOf("posMs");
        if (p >= 0) {
          int c = msg.indexOf(":", p);
          if (c >= 0) {
            uint32_t posMs = (uint32_t)msg.substring(c + 1).toInt();
            g_player.seekMs(posMs);
          }
        }
      }
      else if (msg.indexOf("\"cmd\":\"vol\"") >= 0) {
        int p = msg.indexOf("value");
        if (p >= 0) {
          int c = msg.indexOf(":", p);
          if (c >= 0) {
            int v = msg.substring(c + 1).toInt();
            if (v < 0) v = 0;
            if (v > 127) v = 127;
            g_volume = (uint8_t)v;
          }
        }
      }

      wsBroadcastState();
      ws.sendTXT(num, "{\"type\":\"ack\"}");
      break;
    }

    case WStype_BIN: {
#if UZU_STREAM_DBG
      g_streamDbgBinRx++;
#endif
      size_t written = g_streamEngine.pushPcm(payload, length);
#if UZU_STREAM_DBG
      if (written > 0) {
        g_streamDbgBinOk++;
      }
      if (written == 0 && g_streamDbgBinRx <= 8) {
        Serial.printf("[STREAM-DBG] BIN #%lu len=%u written=0 devMode=%s engine=%s\n",
                      (unsigned long)g_streamDbgBinRx,
                      (unsigned)length,
                      deviceModeName(g_deviceMode),
                      streamStateToString(g_streamEngine.state()));
      }
      streamDbgSummary(false);
#endif
      if (written > 0) {
        uint32_t now = millis();
        const uint32_t infoInterval = 30u;
        if ((now - g_lastBinBufferInfoMs) >= infoInterval) {
          g_lastBinBufferInfoMs = now;
          wsBroadcastBufferInfo();
        }
      }
      break;
    }

    default:
      break;
  }
}

// ====================================================
// Command parser
// ====================================================
class CommandParser {
public:
  static constexpr int BUF_SIZE = 128;

  void begin(HardwareSerial& serial) {
    m_serial = &serial;
    clearBuffer();
    printPrompt();
  }

  void process() {
    if (!m_serial) return;
    while (m_serial->available() > 0) {
      int ch = m_serial->read();
      if (ch < 0) break;
      handleChar((char)ch);
    }
  }

private:
  HardwareSerial* m_serial = nullptr;
  char m_buf[BUF_SIZE];
  int m_len = 0;
  bool m_lineOverflow = false;

  void clearBuffer() {
    m_len = 0;
    m_buf[0] = '\0';
    m_lineOverflow = false;
  }

  void printPrompt() {
    m_serial->print("> ");
  }

  void handleChar(char ch) {
    if (ch == '\r') return;

    if (ch == '\b' || (uint8_t)ch == 0x7F) {
      handleBackspace();
      return;
    }

    if (ch == '\n') {
      m_serial->println();
      finalizeLine();
      printPrompt();
      return;
    }

    if (ch < 0x20 || ch > 0x7E) return;
    if (m_lineOverflow) return;

    if (m_len >= (BUF_SIZE - 1)) {
      m_lineOverflow = true;
      return;
    }

    m_buf[m_len++] = ch;
    m_serial->write((uint8_t)ch);
  }

  void handleBackspace() {
    if (m_len <= 0) return;
    m_len--;
    m_buf[m_len] = '\0';
    m_serial->print("\b \b");
  }

  void finalizeLine() {
    if (m_lineOverflow) {
      println("ERR LINE_TOO_LONG");
      clearBuffer();
      return;
    }

    m_buf[m_len] = '\0';
    trimInPlace(m_buf);

    if (m_buf[0] != '\0') handleLine(m_buf);
    clearBuffer();
  }

  void handleLine(char* line) {
    toUpperInPlace(line);

    char* argv[4] = { nullptr, nullptr, nullptr, nullptr };
    int argc = splitTokens(line, argv, 4);
    if (argc <= 0) return;

    if (strcmp(argv[0], "HELP") == 0 || strcmp(argv[0], "?") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdHelp(); return;
    }

    if (strcmp(argv[0], "CHECKDISK") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdCheckDisk(); return;
    }

    if (strcmp(argv[0], "MOUNT") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdMount(); return;
    }

    if (strcmp(argv[0], "FREQ") == 0) {
      if (argc != 2) { println("ERR BAD_PARAM"); return; }
      cmdFreq(argv[1]); return;
    }

    if (strcmp(argv[0], "DIR") == 0) {
      if (argc > 2) { println("ERR BAD_PARAM"); return; }
      cmdDir(argc == 2 ? argv[1] : "/"); return;
    }

    if (strcmp(argv[0], "FSTAT") == 0) {
      if (argc != 2) { println("ERR BAD_PARAM"); return; }
      int index = 0;
      if (!parsePositiveInt(argv[1], index)) { println("ERR BAD_PARAM"); return; }
      cmdFstat(index); return;
    }

    if (strcmp(argv[0], "PLAY") == 0) {
      if (argc == 1) { cmdPlayResume(); return; }
      if (argc == 2) {
        int index = 0;
        if (!parsePositiveInt(argv[1], index)) { println("ERR BAD_PARAM"); return; }
        cmdPlayIndex(index); return;
      }
      println("ERR BAD_PARAM"); return;
    }

    if (strcmp(argv[0], "PAUSE") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdPause(); return;
    }

    if (strcmp(argv[0], "STOP") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdStop(); return;
    }

    if (strcmp(argv[0], "SEEK") == 0) {
      if (argc != 2) { println("ERR BAD_PARAM"); return; }
      uint32_t ms = 0;
      if (!parseUint32(argv[1], ms)) { println("ERR BAD_PARAM"); return; }
      cmdSeek(ms); return;
    }

    if (strcmp(argv[0], "VOL") == 0) {
      if (argc != 2) { println("ERR BAD_PARAM"); return; }
      uint32_t v = 0;
      if (!parseUint32(argv[1], v) || v > 127) { println("ERR BAD_PARAM"); return; }
      cmdVol((uint8_t)v); return;
    }

    if (strcmp(argv[0], "STAT") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdStat(); return;
    }

    if (strcmp(argv[0], "MODE") == 0) {
      if (argc != 2) { println("ERR BAD_PARAM"); return; }
      if (strcmp(argv[1], "RAW") == 0) {
        stopAllPlayback();
        i2cModeSet(TdmDataMode::RAW);
        i2cModeProcess();
#if UZU_ENABLE_I2C
        i2cHostBroadcastSetMode(TdmDataMode::RAW);
#endif
        println("OK MODE RAW");
        return;
      }
      if (strcmp(argv[1], "MP3") == 0) {
        stopAllPlayback();
        i2cModeSet(TdmDataMode::MP3);
        i2cModeProcess();
#if UZU_ENABLE_I2C
        i2cHostBroadcastSetMode(TdmDataMode::MP3);
#endif
        println("OK MODE MP3");
        return;
      }
      println("ERR BAD_PARAM"); return;
    }

#if UZU_ENABLE_I2C
    if (strcmp(argv[0], "I2CSCAN") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      i2cHostScanChannels();
      println("OK I2CSCAN");
      return;
    }
#endif

    if (strcmp(argv[0], "I2STEST") == 0) {
      if (argc == 2 && strcmp(argv[1], "DUMP") == 0) {
        i2sTestDumpTdmReference();
        println("OK I2STEST DUMP");
        return;
      }
      if (argc == 1) {
        if (i2sTestStart(5)) {
          println("OK I2STEST");
        } else {
          println("ERR I2STEST");
        }
        return;
      }
      if (argc == 2 && strcmp(argv[1], "STOP") == 0) {
        i2sTestStop();
        println("OK I2STEST STOP");
        return;
      }
      if (argc == 2) {
        uint32_t sec = 0;
        if (!parseUint32(argv[1], sec)) { println("ERR BAD_PARAM"); return; }
        if (i2sTestStart(sec)) {
          println("OK I2STEST");
        } else {
          println("ERR I2STEST");
        }
        return;
      }
      println("ERR BAD_PARAM"); return;
    }

    if (strcmp(argv[0], "RESET") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdReset(); return;
    }

    println("ERR UNKNOWN_CMD");
  }

  void cmdHelp() {
    println("OK HELP");
    m_serial->print("  BUILD         : ");
    m_serial->println(FIRMWARE_BUILD);
    m_serial->print("  AP SSID       : ");
    m_serial->println(AP_SSID);
    m_serial->print("  AP PASSWORD   : ");
    m_serial->println(AP_OPEN ? "(open)" : AP_PASSWORD);
    m_serial->print("  AP IP         : ");
    m_serial->println(WiFi.softAPIP());
    println("  HELP / ?      : show help");
    println("  CHECKDISK     : show SD card status");
    println("  MOUNT         : mount SD card");
    println("  FREQ <hz>     : set SD clock");
    println("  DIR [path]    : list *.UZU / *.uzc files");
    println("  MODE RAW|MP3  : TDM output mode");
#if UZU_ENABLE_I2C
    println("  I2CSCAN       : ping Sub CH1-CH5 on I2C bus");
#endif
    println("  I2STEST [sec] : TDM test tone CH1 + RAW tag CH8 (default 5s)");
    println("  I2STEST STOP  : stop TDM test tone");
    println("  FSTAT <n>     : show UZU header details");
    println("  PLAY <n>      : start TDM playback");
    println("  PLAY          : resume from pause");
    println("  PAUSE         : pause playback");
    println("  STOP          : stop playback");
    println("  SEEK <ms>     : seek position in ms");
    println("  VOL <0-127>   : set output volume");
    println("  STAT          : show current status");
    println("  RESET         : reinitialize from setup");
  }

  void cmdReset() {
    shutdownNetworkServices();
    performSystemInit(false);
    println("OK RESET");
  }

  void cmdCheckDisk() {
    println("OK CHECKDISK");
    m_serial->print("CD=");
    m_serial->println(digitalRead(PIN_SD_CD) == LOW ? "LOW" : "HIGH");

    m_serial->print("INSERTED=");
    m_serial->println(isCardInserted() ? "YES" : "NO");

    if (!isCardInserted()) g_mounted = false;

    m_serial->print("MOUNTED=");
    m_serial->println(g_mounted ? "YES" : "NO");

    m_serial->print("FREQ=");
    m_serial->println((unsigned long)g_sd_freq_hz);

    if (g_mounted) {
      m_serial->print("CARDTYPE=");
      m_serial->println(getCardTypeString());

      uint64_t total = SD_MMC.totalBytes();
      uint64_t used  = SD_MMC.usedBytes();
      uint64_t freeb = (total > used) ? (total - used) : 0;

      m_serial->print("TOTAL=");
      m_serial->println((unsigned long long)total);
      m_serial->print("USED=");
      m_serial->println((unsigned long long)used);
      m_serial->print("FREE=");
      m_serial->println((unsigned long long)freeb);
    }
  }

  void cmdMount() {
    m_serial->print("Mounting SDMMC 4-bit @");
    m_serial->print((unsigned long)g_sd_freq_hz);
    m_serial->println(" Hz ...");

    bool ok = mount_sdmmc_4bit(g_sd_freq_hz);
    println(ok ? "OK MOUNT" : "ERR NO_SD");
  }

  void cmdFreq(const char* arg) {
    uint32_t hz = 0;
    if (!parseUint32(arg, hz)) { println("ERR BAD_PARAM"); return; }
    if (hz < 100000) { println("ERR BAD_PARAM"); return; }

    g_sd_freq_hz = hz;
    m_serial->print("OK FREQ ");
    m_serial->println((unsigned long)g_sd_freq_hz);
  }

  void cmdDir(const char* path) {
    if (g_player.state() == PlayerState::PLAY) {
      println("ERR BUSY");
      return;
    }

    if (!g_mounted) {
      println("ERR NO_SD");
      return;
    }

    if (!scanUzuFiles(path)) {
      println("ERR FILE_OPEN");
      return;
    }

    m_serial->print("OK DIR ");
    m_serial->println(g_fileCount);

    for (int i = 0; i < g_fileCount; ++i) {
      m_serial->print(i + 1);
      m_serial->print(": ");
      m_serial->println(g_fileList[i].name);
    }
  }

  void cmdFstat(int index) {
    if (index <= 0 || index > g_fileCount) { println("ERR BAD_INDEX"); return; }
    if (!g_mounted) { println("ERR NO_SD"); return; }
    if (g_player.state() == PlayerState::PLAY) { println("ERR BUSY"); return; }

    File f = SD_MMC.open(g_fileList[index - 1].path, FILE_READ);
    if (!f) { println("ERR FILE_OPEN"); return; }

    if (!check_uzu_magic(f)) {
      f.close();
      println("ERR BAD_FORMAT");
      return;
    }

    uint32_t dataStart = 0;
    uint32_t channels = 0;
    uint32_t sampleRate = 0;
    uint32_t bits = 0;
    uint32_t totalSamples = 0;

    if (!read_u32_at(f, 8, dataStart) ||
        !read_u32_at(f, 12, channels) ||
        !read_u32_at(f, 16, sampleRate) ||
        !read_u32_at(f, 20, bits) ||
        !read_u32_at(f, 24, totalSamples)) {
      f.close();
      println("ERR BAD_FORMAT");
      return;
    }

    if (dataStart < 32768 || channels == 0 || channels > 32 || sampleRate == 0 || bits != 16) {
      f.close();
      println("ERR BAD_FORMAT");
      return;
    }

    uint32_t lengthMs = (uint32_t)(((uint64_t)totalSamples * 1000ULL) / sampleRate);

    m_serial->print("OK FSTAT ");
    m_serial->println(index);

    m_serial->print("NAME=");
    m_serial->println(g_fileList[index - 1].name);

    m_serial->print("PATH=");
    m_serial->println(g_fileList[index - 1].path);

    if (read_string_at(f, 28, 256, g_strbuf, sizeof(g_strbuf))) {
      m_serial->print("TITLE=");
      m_serial->println(g_strbuf);
    } else {
      m_serial->println("TITLE=");
    }

    m_serial->print("LEN=");
    m_serial->println(lengthMs);

    m_serial->print("CH=");
    m_serial->println(channels);

    m_serial->print("RATE=");
    m_serial->println(sampleRate);

    m_serial->print("BITS=");
    m_serial->println(bits);

    m_serial->print("SAMPLES=");
    m_serial->println(totalSamples);

    for (uint32_t i = 0; i < channels; ++i) {
      uint32_t ofs = 284 + i * 256;
      if (read_string_at(f, ofs, 256, g_strbuf, sizeof(g_strbuf))) {
        m_serial->print("CH");
        m_serial->print(i + 1);
        m_serial->print("=");
        m_serial->println(g_strbuf);
      } else {
        m_serial->print("CH");
        m_serial->print(i + 1);
        m_serial->println("=");
      }
    }

    f.close();
  }

  void cmdPlayIndex(int index) {
    if (!playTrackByIndex(index)) {
      if (g_uzcPlaying) {
        println(g_uzcPlayer.lastError());
      } else {
        println(g_player.lastError());
      }
      return;
    }

    m_serial->print("OK PLAY ");
    m_serial->print(index);
    m_serial->print(" ");
    m_serial->println(g_fileList[index - 1].name);
  }

  void cmdPlayResume() {
    if (g_uzcPlaying) {
      if (!g_uzcPlayer.resume()) {
        println(g_uzcPlayer.lastError());
        return;
      }
      println("OK RESUME");
      return;
    }
    if (!g_player.resume()) {
      println(g_player.lastError());
      return;
    }
    println("OK RESUME");
  }

  void cmdPause() {
    if (g_uzcPlaying) {
      g_uzcPlayer.pause();
      println("OK PAUSE");
      return;
    }
    g_player.pause();
    println("OK PAUSE");
  }

  void cmdStop() {
    stopAllPlayback();
    println("OK STOP");
  }

  void cmdSeek(uint32_t ms) {
    if (!g_player.seekMs(ms)) {
      println(g_player.lastError());
      return;
    }
    m_serial->print("OK SEEK ");
    m_serial->println(ms);
  }

  void cmdVol(uint8_t v) {
    g_volume = v;
    m_serial->print("OK VOL ");
    m_serial->println((unsigned long)g_volume);
  }

  void cmdStat() {
    m_serial->print("OK STAT MODE=");
    m_serial->print(tdmDataModeName(i2cModeGet()));
    m_serial->print(" STATE=");
    if (g_uzcPlaying) {
      m_serial->print(g_uzcPlayer.isPaused() ? "PAUSE" : "PLAY");
    } else {
      m_serial->print(stateToString(g_player.state()));
    }
    m_serial->print(" FILE=");
    m_serial->print(g_player.currentIndex());
    m_serial->print(" POS=");
    m_serial->print(g_player.positionMs());
    m_serial->print(" LEN=");
    m_serial->print((g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE) ? g_player.lengthMs() : g_selectedLenMs);
    m_serial->print(" VOL=");
    m_serial->println((unsigned long)g_volume);
  }

  void println(const char* s) {
    m_serial->println(s);
  }

  const char* stateToString(PlayerState st) {
    switch (st) {
      case PlayerState::STOP:  return "STOP";
      case PlayerState::PLAY:  return "PLAY";
      case PlayerState::PAUSE: return "PAUSE";
      case PlayerState::ERROR: return "ERROR";
      default:                 return "UNKNOWN";
    }
  }

  static void toUpperInPlace(char* s) {
    while (*s != '\0') {
      if (*s >= 'a' && *s <= 'z') *s = char(*s - 'a' + 'A');
      ++s;
    }
  }

  static void trimInPlace(char* s) {
    if (!s || s[0] == '\0') return;

    char* start = s;
    while (*start == ' ' || *start == '\t') ++start;

    if (start != s) {
      memmove(s, start, strlen(start) + 1);
    }

    int len = (int)strlen(s);
    while (len > 0) {
      char c = s[len - 1];
      if (c == ' ' || c == '\t') {
        s[len - 1] = '\0';
        --len;
      } else {
        break;
      }
    }
  }

  static int splitTokens(char* s, char* argv[], int maxArgc) {
    int argc = 0;
    char* p = s;

    while (*p != '\0' && argc < maxArgc) {
      while (*p == ' ' || *p == '\t') ++p;
      if (*p == '\0') break;

      argv[argc++] = p;

      while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
      if (*p == '\0') break;

      *p = '\0';
      ++p;
    }

    return argc;
  }

  static bool parsePositiveInt(const char* s, int& out) {
    if (!s || *s == '\0') return false;

    int value = 0;
    const char* p = s;
    while (*p != '\0') {
      if (*p < '0' || *p > '9') return false;
      value = value * 10 + (*p - '0');
      ++p;
    }

    if (value <= 0) return false;
    out = value;
    return true;
  }

  static bool parseUint32(const char* s, uint32_t& out) {
    if (!s || *s == '\0') return false;

    uint32_t value = 0;
    const char* p = s;
    while (*p != '\0') {
      if (*p < '0' || *p > '9') return false;
      uint32_t digit = (uint32_t)(*p - '0');
      uint32_t nextValue = value * 10u + digit;
      if (nextValue < value) return false;
      value = nextValue;
      ++p;
    }

    out = value;
    return true;
  }
};

static CommandParser g_parser;

static void performSystemInit(bool verbose) {
  pinMode(PIN_SD_CD, INPUT);

  g_player.stop();
  g_uzcPlayer.stop();
  g_uzcPlaying = false;
  clearFileList();
  g_selectedTrack0 = 0;
  g_selectedLenMs = 0;
  g_deviceMode = DeviceMode::SD;
  g_streamEngine.reset();
  g_streamPendingSampleRate = TDM_DEFAULT_SAMPLE_RATE;
  g_streamPendingChannels = 2;
  g_streamPendingBits = 16;
  g_streamDroppedBytes = 0;
  g_streamRxBytes = 0;

  g_sdCardPresent = isCardInserted();
  g_sdCdLastRead = g_sdCardPresent;
  g_sdCdStableSinceMs = millis();
  g_sdRemountPending = false;

  g_mounted = mount_sdmmc_4bit(g_sd_freq_hz);
  if (verbose) {
    Serial.println(g_mounted ? "Auto mount OK" : "Auto mount FAILED");
  }

  if (!g_player.begin()) {
    if (verbose) Serial.println("TDM init FAILED");
  }
  g_uzcPlayer.begin(g_tx_handle);
  i2cModeBegin();
#if UZU_ENABLE_I2C
  i2cHostBegin();
  i2cHostBroadcastSetMode(i2cModeGet());
#endif

  if (g_mounted) {
    scanUzuFiles("/");
  }

  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

  bool ok;
  if (AP_OPEN) ok = WiFi.softAP(AP_SSID);
  else         ok = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (verbose) {
    Serial.println(ok ? "SoftAP started." : "SoftAP start FAILED.");
    Serial.print("AP SSID: "); Serial.println(AP_SSID);
    Serial.print("AP IP  : "); Serial.println(WiFi.softAPIP());
  }

  dns.start(DNS_PORT, "*", AP_IP);

  ws.begin();
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(30000, 5000, 3);
  if (verbose) {
    Serial.println("HTTP + WebSocket server started on port 80.");
    Serial.println("Smartphone: connect to AP above, then open http://192.168.4.1");
  }

  ensureStreamPlaybackTask();
  ensurePlayerPlaybackTask();
}

static void shutdownNetworkServices() {
  ws.disconnect();
  ws.close();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ====================================================
// setup / loop
// ====================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(VERSION_STRING);
  Serial.print("Build: ");
  Serial.println(FIRMWARE_BUILD);
  Serial.println();
  delay(100);

  g_power.begin();
#if UZU_SKIP_POWER_BUTTON
  Serial.println("[DEBUG] Power button wait skipped");
#endif
  g_power.waitUntilPowerOn();

  Serial.println();
  Serial.println("UZU Player + TDM + WiFi Control + Power");
  Serial.print("Firmware Build: ");
  Serial.println(FIRMWARE_BUILD);
  Serial.println("Type HELP or ?");

  performSystemInit(true);

  g_parser.begin(Serial);
}

void loop() {
  g_power.process();

  if (g_power.isPoweredOff()) {
    stopAllPlayback();
    delay(10);
    while (1) {
      delay(100);
    }
  }

  g_parser.process();
  i2cModeProcess();
#if UZU_ENABLE_I2C
  i2cHostProcess();
#endif
  i2sTestProcess();

  processSdCardDetect();

  dns.processNextRequest();
  ws.loop();
  wsProcessNotifications();

  if (g_uzcPlaying && !g_uzcPlayer.isActive()) {
    g_uzcPlaying = false;
    tdm_disable_output();
  }
}
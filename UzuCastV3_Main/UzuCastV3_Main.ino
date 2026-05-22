#include <Arduino.h>
#include "SD_MMC.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <WebSocketsServer.h>

#include "driver/i2s_tdm.h"
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION_MAJOR != 3
  #error "This project requires ESP32 Arduino Core 3.x (IDF5)."
#endif

#define VERSION_STRING  "UZU CAST Version 1.00"
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
static void wsSendTrackInfo(uint8_t clientId, int trackIndex0);

static const char* getCardTypeString();
static bool mount_sdmmc_4bit(uint32_t freq_hz);
static void clearFileList();
static int16_t apply_volume_127(int16_t sample, uint8_t vol);

static bool scanUzuFiles(const char* path);

static bool i2s_tdm_tx_init(uint32_t sampleRate);
static bool tdm_set_sample_rate(uint32_t sampleRate);
static bool tdm_enable_output();
static void tdm_disable_output();

static void wsSendTracks(uint8_t clientId);
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
static constexpr uint32_t TDM_NUM_CH           = 8;
static constexpr uint32_t FRAMES_PER_WRITE     = 256;
static constexpr uint32_t BYTES_PER_TDM_FRAME  = TDM_NUM_CH * 2;

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
static uint8_t g_pcmReadBuf[FRAMES_PER_WRITE * TDM_NUM_CH * 2];
static int16_t g_tdmTxBuf[FRAMES_PER_WRITE * TDM_NUM_CH];
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

    m_state = State::WAIT_ON_PRESS;
    m_pressStartMs = 0;
    m_releaseGuardStartMs = 0;
  }

  void waitUntilPowerOn() {
    while (m_state != State::POWER_ON &&
           m_state != State::WAIT_STABLE_RELEASE_AFTER_ON) {
      processBeforeOn();
      delay(1);
    }
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
  </style>
</head>
<body>
  <h1>UZU CAST HOST</h1>

  <div class="card">
    <div class="row">
      <div class="status mono" id="wsState">WS: (connecting)</div>
    </div>
  </div>

  <div class="card">
    <h2>1) 曲の選択</h2>
    <div class="row">
      <select id="trackSelect"></select>
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

  let ws;
  let currentTrackIndex = 0;
  let currentLenMs = 0;
  let currentPlayState = "STOP";
  let detailVisible = false;

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

  function updateTrackLock(){
    trackSelect.disabled = (currentPlayState === "PLAY" || currentPlayState === "PAUSE");
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

  function send(obj){
    const s = JSON.stringify(obj);
    if(ws && ws.readyState === WebSocket.OPEN){
      ws.send(s);
    }
  }

  function connectWS(){
    const host = window.location.hostname || "192.168.4.1";
    const url = "ws://" + host + "/";
    wsState.textContent = "WS: CONNECTING";

    ws = new WebSocket(url);

    ws.onopen = ()=>{
      wsState.textContent = "WS: CONNECTED";
      send({cmd:"hello", client:"web"});
      send({cmd:"get_tracks"});
      send({cmd:"get_state"});
    };

    ws.onclose = (ev)=>{
      wsState.textContent = "WS: CLOSED code=" + ev.code;
      setTimeout(connectWS, 1000);
    };

    ws.onerror = ()=>{
      wsState.textContent = "WS: ERROR";
    };

    ws.onmessage = (ev)=>{
      try{
        const msg = JSON.parse(ev.data);

        if(msg.type === "tracks" && Array.isArray(msg.items)){
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
          trackInfo.textContent = formatTrackInfo(msg);
          return;
        }

        if(msg.type === "state"){
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

  trackSelect.onchange = ()=>{
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
    if (detailVisible) {
      send({cmd:"get_info", trackIndex: currentTrackIndex});
    }
  };

  $("btnPlay").onclick = ()=> send({cmd:"play"});
  $("btnStop").onclick = ()=> send({cmd:"stop"});
  $("btnPause").onclick = ()=> send({cmd:"pause"});

  seek.oninput = ()=> updateSeekLabel();
  seek.onchange = ()=>{
    const posMs = parseInt(seek.value,10) || 0;
    send({cmd:"seek", posMs});
  };

  vol.oninput = ()=> updateVolLabel();
  vol.onchange = ()=>{
    const value = parseInt(vol.value,10) || 0;
    send({cmd:"vol", value});
  };

  setInterval(()=>{
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

        if (hasExtensionIgnoreCase(showName, ".UZU")) {
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
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_disable error: %d\n", err);
    return;
  }

  g_tdm_enabled = false;
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

      for (uint32_t ch = 0; ch < m_channels; ++ch) {
        int16_t s = le16_to_s16(&g_pcmReadBuf[srcBase + ch * 2]);
        g_tdmTxBuf[dstBase + ch] = apply_volume_127(s, g_volume);
      }
    }

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
  }

  bool playIndex(int index) {
    if (index <= 0 || index > g_fileCount) {
      m_lastError = "ERR BAD_INDEX";
      return false;
    }

    stop();

    File f = SD_MMC.open(g_fileList[index - 1].path, FILE_READ);
    if (!f) {
      m_lastError = "ERR FILE_OPEN";
      return false;
    }

    if (!check_uzu_magic(f)) {
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

    if (!tdm_set_sample_rate(sampleRate)) {
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

    g_selectedTrack0 = index - 1;
    g_selectedLenMs = m_lengthMs;

    Serial.printf("PLAY open: ch=%lu rate=%lu bits=%lu samples=%lu dataStart=%lu vol=%u\n",
                  (unsigned long)m_channels,
                  (unsigned long)m_sampleRate,
                  (unsigned long)m_bits,
                  (unsigned long)m_totalSamples,
                  (unsigned long)m_dataStart,
                  (unsigned)g_volume);

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
};

static UzuTdmPlayer g_player;

// ====================================================
// Web helpers
// ====================================================
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
  if (g_player.state() != PlayerState::PLAY) {
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

static void wsSendTrackInfo(uint8_t clientId, int trackIndex0) {
  if (g_player.state() != PlayerState::PLAY) {
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
  uint32_t lenMs = (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE)
                 ? g_player.lengthMs()
                 : g_selectedLenMs;

  uint32_t posMs = (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE)
                 ? g_player.positionMs()
                 : 0;

  String st = "{\"type\":\"state\",\"play\":\"";
  switch (g_player.state()) {
    case PlayerState::PLAY:  st += "PLAY"; break;
    case PlayerState::PAUSE: st += "PAUSE"; break;
    case PlayerState::STOP:  st += "STOP"; break;
    default:                 st += "ERROR"; break;
  }
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
  uint32_t lenMs = (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE)
                 ? g_player.lengthMs()
                 : g_selectedLenMs;

  uint32_t posMs = (g_player.state() == PlayerState::PLAY || g_player.state() == PlayerState::PAUSE)
                 ? g_player.positionMs()
                 : 0;

  String st = "{\"type\":\"state\",\"play\":\"";
  switch (g_player.state()) {
    case PlayerState::PLAY:  st += "PLAY"; break;
    case PlayerState::PAUSE: st += "PAUSE"; break;
    case PlayerState::STOP:  st += "STOP"; break;
    default:                 st += "ERROR"; break;
  }
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

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected\n", num);
      wsSendTracks(num);
      wsSendState(num);
      wsSendTrackInfo(num, g_selectedTrack0);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;

    case WStype_TEXT: {
      String msg;
      msg.reserve(length + 1);
      for (size_t i = 0; i < length; i++) msg += (char)payload[i];

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
        if (g_fileCount > 0) g_player.playIndex(g_selectedTrack0 + 1);
      }
      else if (msg.indexOf("\"cmd\":\"stop\"") >= 0) {
        g_player.stop();
      }
      else if (msg.indexOf("\"cmd\":\"pause\"") >= 0) {
        g_player.pause();
      }
      else if (msg.indexOf("\"cmd\":\"select\"") >= 0) {
        if (g_player.state() == PlayerState::STOP) {
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

    if (strcmp(argv[0], "RESET") == 0) {
      if (argc != 1) { println("ERR BAD_PARAM"); return; }
      cmdReset(); return;
    }

    println("ERR UNKNOWN_CMD");
  }

  void cmdHelp() {
    println("OK HELP");
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
    println("  DIR [path]    : list *.UZU files");
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
    if (!g_player.playIndex(index)) {
      println(g_player.lastError());
      return;
    }

    m_serial->print("OK PLAY ");
    m_serial->print(index);
    m_serial->print(" ");
    m_serial->println(g_fileList[index - 1].name);
  }

  void cmdPlayResume() {
    if (!g_player.resume()) {
      println(g_player.lastError());
      return;
    }
    println("OK RESUME");
  }

  void cmdPause() {
    g_player.pause();
    println("OK PAUSE");
  }

  void cmdStop() {
    g_player.stop();
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
    m_serial->print("OK STAT STATE=");
    m_serial->print(stateToString(g_player.state()));
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
  clearFileList();
  g_selectedTrack0 = 0;
  g_selectedLenMs = 0;

  g_mounted = mount_sdmmc_4bit(g_sd_freq_hz);
  if (verbose) {
    Serial.println(g_mounted ? "Auto mount OK" : "Auto mount FAILED");
  }

  if (!g_player.begin()) {
    if (verbose) Serial.println("TDM init FAILED");
  }

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
  ws.enableHeartbeat(15000, 3000, 2);
  if (verbose) {
    Serial.println("HTTP + WebSocket server started on port 80.");
  }
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
  Serial.println(VERSION_STRING);
  delay(100);

  g_power.begin();
  g_power.waitUntilPowerOn();

  Serial.println();
  Serial.println("UZU Player + TDM + WiFi Control + Power");
  Serial.println("Type HELP or ?");

  performSystemInit(true);

  g_parser.begin(Serial);
}

void loop() {
  g_power.process();

  if (g_power.isPoweredOff()) {
    g_player.stop();
    delay(10);
    while (1) {
      delay(100);
    }
  }

  g_parser.process();
  g_player.process();

  dns.processNextRequest();
  ws.loop();
}
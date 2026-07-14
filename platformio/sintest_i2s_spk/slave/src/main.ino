/*
 * sintest_i2s_spk — Slave (ESP32)
 * Stage 1: Master I2S -> Slave I2S RX -> tag decode -> I2S TX -> Speaker
 *
 * I2S RX (from Master): BCLK=32, WS=33, DIN=35  (uzuCast Sub / MP3TEST2)
 * I2S TX (to DAC/amp):  BCLK=14, WS=13, DOUT=27 (MP3TEST2 slave)
 *
 * R=0xAAAA frames: output L as stereo PCM to speaker path.
 */

#include <Arduino.h>
#include "driver/i2s.h"

static constexpr uint32_t BUILD_NUMBER = 1;

static const int PIN_I2S_RX_BCLK = 32;
static const int PIN_I2S_RX_WS   = 33;
static const int PIN_I2S_RX_DIN  = 35;

static const int PIN_I2S_TX_BCLK = 14;
static const int PIN_I2S_TX_WS   = 13;
static const int PIN_I2S_TX_DOUT = 27;

static constexpr uint32_t I2S_SAMPLE_RATE = 44100;
static constexpr uint32_t PCM_HOLD_FRAMES = 4096;
static constexpr int16_t I2S_TAG_RAW = (int16_t)0xAAAA;

static portMUX_TYPE g_pcm_mux = portMUX_INITIALIZER_UNLOCKED;
static int16_t g_pcm_hold[PCM_HOLD_FRAMES * 2];
static uint32_t g_pcm_hold_w = 0;
static uint32_t g_pcm_hold_r = 0;
static uint32_t g_pcm_hold_used = 0;
static uint32_t g_tag_rx = 0;
static uint32_t g_underrun = 0;
static int16_t g_dbg_last_l = 0;
static uint16_t g_dbg_last_r = 0;

static volatile bool g_run = true;
static TaskHandle_t g_rx_task = nullptr;
static TaskHandle_t g_tx_task = nullptr;

static bool initI2sRx() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  cfg.sample_rate = I2S_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_RX_BCLK;
  pins.ws_io_num = PIN_I2S_RX_WS;
  pins.data_in_num = PIN_I2S_RX_DIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr) != ESP_OK) {
    return false;
  }
  return i2s_set_pin(I2S_NUM_1, &pins) == ESP_OK;
}

static bool initI2sTx() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = I2S_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_TX_BCLK;
  pins.ws_io_num = PIN_I2S_TX_WS;
  pins.data_out_num = PIN_I2S_TX_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) {
    return false;
  }
  return i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK;
}

static void pcmHoldAppendMono(int16_t sample) {
  portENTER_CRITICAL(&g_pcm_mux);
  if (g_pcm_hold_used < PCM_HOLD_FRAMES) {
    const uint32_t slot = g_pcm_hold_w % PCM_HOLD_FRAMES;
    g_pcm_hold[slot * 2 + 0] = sample;
    g_pcm_hold[slot * 2 + 1] = sample;
    g_pcm_hold_w++;
    g_pcm_hold_used++;
  }
  portEXIT_CRITICAL(&g_pcm_mux);
}

static uint32_t pcmHoldPop(int16_t* stereoOut, uint32_t maxFrames) {
  uint32_t popped = 0;
  portENTER_CRITICAL(&g_pcm_mux);
  popped = (g_pcm_hold_used < maxFrames) ? g_pcm_hold_used : maxFrames;
  for (uint32_t i = 0; i < popped; i++) {
    const uint32_t slot = g_pcm_hold_r % PCM_HOLD_FRAMES;
    stereoOut[i * 2 + 0] = g_pcm_hold[slot * 2 + 0];
    stereoOut[i * 2 + 1] = g_pcm_hold[slot * 2 + 1];
    g_pcm_hold_r++;
    g_pcm_hold_used--;
  }
  portEXIT_CRITICAL(&g_pcm_mux);
  return popped;
}

static void processRxFrames(const int16_t* stereo, uint32_t frameCount) {
  for (uint32_t i = 0; i < frameCount; i++) {
    const int16_t l = stereo[i * 2 + 0];
    const uint16_t tag = (uint16_t)stereo[i * 2 + 1];
    g_dbg_last_l = l;
    g_dbg_last_r = tag;
    if (tag == (uint16_t)I2S_TAG_RAW) {
      g_tag_rx++;
      pcmHoldAppendMono(l);
    }
  }
}

static void i2sRxTask(void* /*arg*/) {
  for (;;) {
    if (!g_run) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    for (int burst = 0; burst < 32; burst++) {
      int16_t tmp[128 * 2];
      size_t rbytes = 0;
      const esp_err_t e = i2s_read(I2S_NUM_1, tmp, sizeof(tmp), &rbytes, 0);
      if (e != ESP_OK || rbytes == 0) {
        break;
      }
      const uint32_t frames = (uint32_t)(rbytes / (sizeof(int16_t) * 2));
      if (frames > 0) {
        processRxFrames(tmp, frames);
      }
    }
    vTaskDelay(1);
  }
}

static void i2sTxTask(void* /*arg*/) {
  const size_t frameCount = 256;
  int16_t buf[frameCount * 2];

  for (;;) {
    if (!g_run) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const uint32_t popped = pcmHoldPop(buf, frameCount);
    if (popped > 0) {
      size_t written = 0;
      (void)i2s_write(I2S_NUM_0, buf, popped * sizeof(int16_t) * 2, &written, portMAX_DELAY);
    } else {
      g_underrun++;
      memset(buf, 0, sizeof(buf));
      size_t written = 0;
      (void)i2s_write(I2S_NUM_0, buf, 64, &written, pdMS_TO_TICKS(5));
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== sintest_i2s_spk SLAVE ==="));
  Serial.print(F("Build: "));
  Serial.println(BUILD_NUMBER);
  Serial.println(F("Stage 1: I2S RX -> tag(AAAA) -> I2S TX -> Speaker"));
  Serial.print(F("I2S RX BCLK/WS/DIN = GPIO"));
  Serial.print(PIN_I2S_RX_BCLK);
  Serial.print('/');
  Serial.print(PIN_I2S_RX_WS);
  Serial.print('/');
  Serial.println(PIN_I2S_RX_DIN);
  Serial.print(F("I2S TX BCLK/WS/DOUT = GPIO"));
  Serial.print(PIN_I2S_TX_BCLK);
  Serial.print('/');
  Serial.print(PIN_I2S_TX_WS);
  Serial.print('/');
  Serial.println(PIN_I2S_TX_DOUT);

  if (!initI2sRx()) {
    Serial.println(F("FATAL: I2S RX init failed"));
  } else {
    Serial.println(F("I2S RX OK (slave)"));
  }

  if (!initI2sTx()) {
    Serial.println(F("FATAL: I2S TX init failed"));
  } else {
    Serial.println(F("I2S TX OK (master -> speaker)"));
  }

  xTaskCreatePinnedToCore(i2sRxTask, "i2sRx", 4096, nullptr, 3, &g_rx_task, 0);
  xTaskCreatePinnedToCore(i2sTxTask, "i2sTx", 4096, nullptr, 2, &g_tx_task, 1);

  Serial.println(F("Ready — waiting for Master ON 1"));
}

void loop() {
  static uint32_t hb = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - hb) >= 2000U) {
    hb = now;
    uint32_t hold = 0;
    portENTER_CRITICAL(&g_pcm_mux);
    hold = g_pcm_hold_used;
    portEXIT_CRITICAL(&g_pcm_mux);
    Serial.print(F("[HB] rx="));
    Serial.print(g_tag_rx);
    Serial.print(F(" hold="));
    Serial.print(hold);
    Serial.print(F(" u="));
    Serial.print(g_underrun);
    Serial.print(F(" peek L="));
    Serial.print(g_dbg_last_l);
    Serial.print(F(" R="));
    Serial.println(g_dbg_last_r, HEX);
  }
  delay(10);
}

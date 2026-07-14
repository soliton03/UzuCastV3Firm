#include "tdm_tx.h"

#include <Arduino.h>
#include <cstring>

#include "driver/i2s_tdm.h"

static constexpr gpio_num_t TDM_BCLK_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t TDM_WS_GPIO   = GPIO_NUM_16;
static constexpr gpio_num_t TDM_DOUT_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t TDM_MCLK_GPIO = I2S_GPIO_UNUSED;

static constexpr i2s_tdm_slot_mask_t TDM_SLOT_MASK =
    (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
                          I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7);

static i2s_chan_handle_t s_txHandle = nullptr;
static bool s_tdmEnabled = false;
static uint32_t s_currentRate = 0;
static bool s_tdmReady = false;

static int16_t s_stopFooterPad[64 * TDM_NUM_CH];

static bool tdmWriteBytes(const int16_t* data, size_t bytes) {
  if (!s_txHandle || bytes == 0) {
    return false;
  }
  if (!tdmTxEnsureEnabled()) {
    return false;
  }
  size_t written = 0;
  return i2s_channel_write(s_txHandle, data, bytes, &written, portMAX_DELAY) == ESP_OK &&
         written == bytes;
}

static bool tdmInitChannel(uint32_t sampleRate) {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  esp_err_t err = i2s_new_channel(&chan_cfg, &s_txHandle, nullptr);
  if (err != ESP_OK) {
    Serial.printf("TDM i2s_new_channel error: %d\n", err);
    return false;
  }

  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(sampleRate),
      .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
                                                         TDM_SLOT_MASK),
      .gpio_cfg =
          {
              .mclk = TDM_MCLK_GPIO,
              .bclk = TDM_BCLK_GPIO,
              .ws = TDM_WS_GPIO,
              .dout = TDM_DOUT_GPIO,
              .din = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  tdm_cfg.slot_cfg.total_slot = TDM_NUM_CH;
  tdm_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;

  err = i2s_channel_init_tdm_mode(s_txHandle, &tdm_cfg);
  if (err != ESP_OK) {
    Serial.printf("TDM i2s_channel_init_tdm_mode error: %d\n", err);
    i2s_del_channel(s_txHandle);
    s_txHandle = nullptr;
    return false;
  }

  s_tdmEnabled = false;
  s_currentRate = sampleRate;
  s_tdmReady = true;
  Serial.printf("TDM init OK (%lu Hz, 8ch, CH8=tag)\n", (unsigned long)sampleRate);
  return true;
}

bool tdmTxInit(uint32_t sampleRate) {
  if (s_txHandle && s_currentRate == sampleRate) {
    return true;
  }
  if (s_txHandle) {
    tdmTxDisable();
    i2s_del_channel(s_txHandle);
    s_txHandle = nullptr;
    s_tdmReady = false;
  }
  return tdmInitChannel(sampleRate);
}

bool tdmTxSetRate(uint32_t sampleRate) {
  if (sampleRate == s_currentRate && s_txHandle) {
    return tdmTxEnsureEnabled();
  }
  if (s_txHandle) {
    if (s_tdmEnabled) {
      i2s_channel_disable(s_txHandle);
      s_tdmEnabled = false;
    }
    i2s_del_channel(s_txHandle);
    s_txHandle = nullptr;
    s_tdmReady = false;
  }
  return tdmInitChannel(sampleRate) && tdmTxEnsureEnabled();
}

bool tdmTxEnsureEnabled() {
  if (!s_txHandle) {
    return false;
  }
  if (s_tdmEnabled) {
    return true;
  }
  const esp_err_t err = i2s_channel_enable(s_txHandle);
  if (err != ESP_OK) {
    Serial.printf("TDM i2s_channel_enable error: %d\n", err);
    return false;
  }
  s_tdmEnabled = true;
  return true;
}

bool tdmTxIsReady() {
  return s_tdmReady;
}

void tdmTxDisable() {
  if (!s_txHandle || !s_tdmEnabled) {
    return;
  }
  i2s_channel_disable(s_txHandle);
  s_tdmEnabled = false;
}

bool tdmWriteFrames(const int16_t* tdm, size_t frameCount) {
  if (frameCount == 0) {
    return true;
  }
  return tdmWriteBytes(tdm, frameCount * TDM_NUM_CH * sizeof(int16_t));
}

bool tdmWriteTaggedStereoPairs(const int16_t* pairs, size_t stereoFrameCount) {
  if (stereoFrameCount == 0) {
    return true;
  }

  static int16_t buf[256 * TDM_NUM_CH];
  size_t chunkFrames = 0;

  for (size_t i = 0; i < stereoFrameCount; i++) {
    int16_t* frame = buf + chunkFrames * TDM_NUM_CH;
    frame[0] = pairs[i * 2];
    for (uint32_t ch = 1; ch < TDM_NUM_CH - 1; ch++) {
      frame[ch] = 0;
    }
    frame[TDM_TAG_CH] = pairs[i * 2 + 1];
    chunkFrames++;

    if (chunkFrames >= 256) {
      if (!tdmWriteFrames(buf, chunkFrames)) {
        return false;
      }
      chunkFrames = 0;
    }
  }

  if (chunkFrames > 0) {
    return tdmWriteFrames(buf, chunkFrames);
  }
  return true;
}

bool tdmWriteRawMono(const int16_t* mono, size_t sampleCount) {
  static int16_t buf[256 * TDM_NUM_CH];
  size_t chunkFrames = 0;

  for (size_t i = 0; i < sampleCount; i++) {
    int16_t* frame = buf + chunkFrames * TDM_NUM_CH;
    frame[0] = mono[i];
    for (uint32_t ch = 1; ch < TDM_NUM_CH - 1; ch++) {
      frame[ch] = 0;
    }
    frame[TDM_TAG_CH] = TDM_TAG_RAW;
    chunkFrames++;

    if (chunkFrames >= 256) {
      if (!tdmWriteFrames(buf, chunkFrames)) {
        return false;
      }
      chunkFrames = 0;
    }
  }

  if (chunkFrames > 0) {
    return tdmWriteFrames(buf, chunkFrames);
  }
  return true;
}

void tdmWriteInvalidKeepalive(size_t frameCount) {
  static int16_t inv[64 * TDM_NUM_CH];
  for (size_t i = 0; i < 64; i++) {
    for (uint32_t ch = 0; ch < TDM_NUM_CH - 1; ch++) {
      inv[i * TDM_NUM_CH + ch] = 0;
    }
    inv[i * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_INVALID;
  }
  while (frameCount > 0) {
    const size_t n = (frameCount < 64) ? frameCount : 64;
    tdmWriteFrames(inv, n);
    frameCount -= n;
  }
}

void fillMp3TdmPeriodTaggedMono(const int16_t* ch1Words, size_t n1, int16_t* tdm, size_t periodFrames,
                                size_t slotStereoFrames) {
  size_t f = 0;
  for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) {
    tdm[f * TDM_NUM_CH + ch] = 0;
  }
  tdm[f * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_MP3_START;
  f++;

  for (size_t i = 0; i < slotStereoFrames; i++) {
    tdm[f * TDM_NUM_CH + 0] = (i < n1) ? ch1Words[i] : 0;
    for (uint32_t ch = 1; ch < TDM_NUM_CH - 1; ch++) {
      tdm[f * TDM_NUM_CH + ch] = 0;
    }
    tdm[f * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_SLOT_DATA;
    f++;
  }

  for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) {
    tdm[f * TDM_NUM_CH + ch] = 0;
  }
  tdm[f * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_MP3_END;
  f++;

  for (; f < periodFrames; f++) {
    for (uint32_t ch = 0; ch < TDM_NUM_CH - 1; ch++) {
      tdm[f * TDM_NUM_CH + ch] = 0;
    }
    tdm[f * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_INVALID;
  }
}

void tdmTransmitStopFooter() {
  for (size_t i = 0; i < 64; i++) {
    for (uint32_t ch = 0; ch < TDM_NUM_CH - 1; ch++) {
      s_stopFooterPad[i * TDM_NUM_CH + ch] = 0;
    }
    s_stopFooterPad[i * TDM_NUM_CH + TDM_TAG_CH] = TDM_TAG_PAD;
  }
  tdmWriteFrames(s_stopFooterPad, 64);
  Serial.println(F("UZC TDM tag MP3 STOP"));
}

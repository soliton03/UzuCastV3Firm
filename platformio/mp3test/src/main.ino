/*
 * mp3test — Standalone ESP32 (from sintest_i2s_spk slave + sintest_i2s_mp3 master CLI)
 *
 * No Master I2S input. Embedded UZC MP3 slot or local 441Hz RAW sine -> I2S TX -> Speaker.
 *
 * I2S TX (to DAC/amp): BCLK=14, WS=13, DOUT=27 (MP3TEST2 slave)
 *
 * Serial CLI: 115200 8N1, prompt "#"
 *   MODE RAW | MODE MP3 | ON 1 | OFF ALL | STAT | HELP ...
 */

// Uncomment to stream decoded PCM to Serial Plotter (setup/loop become plot-only).
// #define WAVE_CHECK

#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"

#include "uzc_mp3_slot.h"
#include "uzc_mp3_pcm_ref.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_STATIC_SCRATCH
#include "minimp3.h"

static constexpr uint32_t BUILD_NUMBER = 13;

static const int PIN_I2S_TX_BCLK = 14;
static const int PIN_I2S_TX_WS   = 13;
static const int PIN_I2S_TX_DOUT = 27;

static constexpr uint32_t SAMPLE_RATE = 44100;
static constexpr uint32_t PCM_HOLD_FRAMES = 8192;
static constexpr float TONE_FREQ_HZ = 441.0f;
static constexpr int16_t TONE_LEVEL = 12000;

static constexpr uint32_t TEST_ID_MIN = 1;
static constexpr uint32_t TEST_ID_MAX = 1;
static constexpr uint32_t SINE_LUT_SIZE = 512;
static constexpr uint32_t PHASE_FRAC_BITS = 8;
static constexpr uint32_t PHASE_INC =
    (uint32_t)(((uint64_t)SINE_LUT_SIZE << PHASE_FRAC_BITS) * (uint64_t)TONE_FREQ_HZ / SAMPLE_RATE);

static constexpr uint16_t UZC_FRAME_DATA_OFF = 4;
static constexpr uint32_t RAW_PRODUCE_FRAMES = 256;
static constexpr uint32_t MP3_PCM_MAX_FRAMES = 4096;
static constexpr uint32_t PCM_LOW_WATER = RAW_PRODUCE_FRAMES * 4;

static const char PROMPT[] = "# ";

enum class OutputMode : uint8_t {
  RAW = 0,
  MP3 = 1,
};

static portMUX_TYPE g_pcm_mux = portMUX_INITIALIZER_UNLOCKED;
static int16_t g_pcm_hold[PCM_HOLD_FRAMES * 2];
static uint32_t g_pcm_hold_w = 0;
static uint32_t g_pcm_hold_r = 0;
static uint32_t g_pcm_hold_used = 0;

static volatile bool g_test_on[TEST_ID_MAX] = {};
static OutputMode g_mode = OutputMode::RAW;
static int16_t g_sine_lut[SINE_LUT_SIZE];
static uint32_t g_phase_acc = 0;
static String g_input_line;

static bool g_i2s_tx_ok = false;
static volatile bool g_run = true;
static TaskHandle_t g_produce_task = nullptr;
static TaskHandle_t g_tx_task = nullptr;

static mp3dec_t g_mp3dec;
static mp3d_sample_t g_mp3_pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
static int16_t g_mp3_pcm_cache[MP3_PCM_MAX_FRAMES * 2];
static uint32_t g_mp3_pcm_frame_count = 0;
static uint32_t g_mp3_play_start = 0;
static uint32_t g_mp3_play_frames = 0;
static uint32_t g_mp3_play_idx = 0;
static int g_mp3_decode_channels = 0;
static int g_mp3_decode_hz = 0;
static bool g_mp3_cache_ready = false;
static int64_t g_mp3_align_score = INT64_MAX;
static uint32_t g_mp3_energy_off = 0;
static uint8_t g_slot_buf[UZC_MP3_SLOT_BYTES];

static uint32_t g_stat_raw_frames = 0;
static uint32_t g_stat_mp3_decode_ok = 0;
static uint32_t g_stat_mp3_decode_fail = 0;
static uint32_t g_stat_pcm_drop = 0;
static uint32_t g_underrun = 0;

static const char* modeName(OutputMode mode) {
  switch (mode) {
    case OutputMode::RAW:
      return "RAW";
    case OutputMode::MP3:
      return "MP3";
  }
  return "?";
}

static bool anyTestActive() {
  for (uint32_t i = 0; i < TEST_ID_MAX; i++) {
    if (g_test_on[i]) {
      return true;
    }
  }
  return false;
}

static void pcmHoldReset() {
  portENTER_CRITICAL(&g_pcm_mux);
  g_pcm_hold_w = 0;
  g_pcm_hold_r = 0;
  g_pcm_hold_used = 0;
  portEXIT_CRITICAL(&g_pcm_mux);
}

static void pcmHoldAppendStereo(const int16_t* stereoFrames, uint32_t frameCount) {
  portENTER_CRITICAL(&g_pcm_mux);
  for (uint32_t i = 0; i < frameCount; i++) {
    if (g_pcm_hold_used >= PCM_HOLD_FRAMES) {
      g_stat_pcm_drop += frameCount - i;
      break;
    }
    const uint32_t slot = g_pcm_hold_w % PCM_HOLD_FRAMES;
    g_pcm_hold[slot * 2 + 0] = stereoFrames[i * 2 + 0];
    g_pcm_hold[slot * 2 + 1] = stereoFrames[i * 2 + 1];
    g_pcm_hold_w++;
    g_pcm_hold_used++;
  }
  portEXIT_CRITICAL(&g_pcm_mux);
}

static void pcmHoldAppendMono(int16_t sample) {
  int16_t frame[2] = {sample, sample};
  pcmHoldAppendStereo(frame, 1);
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

static uint32_t pcmHoldUsed() {
  portENTER_CRITICAL(&g_pcm_mux);
  const uint32_t used = g_pcm_hold_used;
  portEXIT_CRITICAL(&g_pcm_mux);
  return used;
}

static void initSlotBufferFromWords() {
  for (uint32_t i = 0; i < UZC_MP3_SLOT_WORDS; i++) {
    const int16_t w = kUzcMp3SlotWords[i];
    g_slot_buf[i * 2 + 0] = (uint8_t)(w & 0xFF);
    g_slot_buf[i * 2 + 1] = (uint8_t)((w >> 8) & 0xFF);
  }
}

static void initSineLut() {
  for (uint32_t i = 0; i < SINE_LUT_SIZE; i++) {
    const float rad = (2.0f * PI * (float)i) / (float)SINE_LUT_SIZE;
    g_sine_lut[i] = (int16_t)(sinf(rad) * (float)TONE_LEVEL);
  }
}

static int16_t nextToneSample() {
  const uint32_t idx = (g_phase_acc >> PHASE_FRAC_BITS) & (SINE_LUT_SIZE - 1);
  g_phase_acc += PHASE_INC;
  return g_sine_lut[idx];
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

static int32_t mp3CacheMaxAbsFrom(uint32_t start, uint32_t count) {
  int32_t maxAbs = 0;
  for (uint32_t i = 0; i < count; i++) {
    const uint32_t idx = start + i;
    if (idx >= g_mp3_pcm_frame_count) {
      break;
    }
    const int32_t s = g_mp3_pcm_cache[idx * 2];
    const int32_t a = (s < 0) ? -s : s;
    if (a > maxAbs) {
      maxAbs = a;
    }
  }
  return maxAbs;
}

static int32_t mp3CacheMaxAbs() {
  if (g_mp3_play_frames > 0) {
    return mp3CacheMaxAbsFrom(g_mp3_play_start, g_mp3_play_frames);
  }
  return mp3CacheMaxAbsFrom(0, g_mp3_pcm_frame_count);
}

static int32_t mp3CacheSampleAt(uint32_t frameIdx) {
  return g_mp3_pcm_cache[frameIdx * 2];
}

static int32_t sampleAbs(int32_t s) {
  return (s < 0) ? -s : s;
}

static uint32_t findMp3FirstEnergyOffset(int32_t threshold) {
  for (uint32_t off = 0; off + 8U <= g_mp3_pcm_frame_count; off++) {
    int32_t peak = 0;
    for (uint32_t j = 0; j < 8U; j++) {
      const int32_t a = sampleAbs(mp3CacheSampleAt(off + j));
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

static uint32_t findMp3PcmAlignOffset(int64_t* outScore, uint32_t* outEnergyOff) {
  if (outEnergyOff) {
    *outEnergyOff = findMp3FirstEnergyOffset(3000);
  }

  const uint32_t refLen = UZC_MP3_PCM_REF_SAMPLES;
  if (g_mp3_pcm_frame_count < refLen) {
    if (outScore) {
      *outScore = INT64_MAX;
    }
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

  if (outScore) {
    *outScore = bestScore;
  }
  return bestOff;
}

static void mp3UpdatePlayWindow() {
  g_mp3_play_start = findMp3PcmAlignOffset(&g_mp3_align_score, &g_mp3_energy_off);
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

static bool decodeMp3SlotToCache() {
  g_mp3_cache_ready = false;
  g_mp3_pcm_frame_count = 0;
  g_mp3_play_start = 0;
  g_mp3_play_frames = 0;

  const uint8_t* mp3Data = g_slot_buf + UZC_FRAME_DATA_OFF;
  const uint16_t mp3Len = (uint16_t)UZC_MP3_REAL_FRAME_SIZE;
  if (mp3Len < 4) {
    g_stat_mp3_decode_fail++;
    return false;
  }

  mp3dec_init(&g_mp3dec);
  uint32_t offset = 0;
  while (offset + 4U <= mp3Len && g_mp3_pcm_frame_count < MP3_PCM_MAX_FRAMES) {
    mp3dec_frame_info_t info;
    memset(&info, 0, sizeof(info));

    const int samplesPerCh =
        mp3dec_decode_frame(&g_mp3dec, mp3Data + offset, (int)(mp3Len - offset), g_mp3_pcm, &info);
    if (samplesPerCh <= 0 || info.frame_bytes <= 0) {
      break;
    }

    const uint32_t space = MP3_PCM_MAX_FRAMES - g_mp3_pcm_frame_count;
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
    g_mp3_decode_channels = info.channels;
    g_mp3_decode_hz = info.hz;
    offset += (uint32_t)info.frame_bytes;
  }

  if (g_mp3_pcm_frame_count == 0) {
    g_stat_mp3_decode_fail++;
    return false;
  }

  mp3UpdatePlayWindow();
  g_mp3_cache_ready = true;
  g_stat_mp3_decode_ok++;
  if (mp3CacheMaxAbs() < 1000) {
    Serial.println(F("[WARN] MP3 decode peak too low — cache may be silent"));
  }
  return true;
}

static bool prepareMp3PcmCache() {
  g_mp3_play_idx = 0;
  mp3dec_init(&g_mp3dec);
  return decodeMp3SlotToCache();
}

static void printMp3Diagnostics() {
  if (!g_mp3_cache_ready && !prepareMp3PcmCache()) {
    Serial.println(F("[DIAG] MP3 cache decode failed"));
    return;
  }

  Serial.println(F("[DIAG] MP3 decode dump (host-style verify)"));
  Serial.print(F("  decodedFrames="));
  Serial.print(g_mp3_pcm_frame_count);
  Serial.print(F(" playStart="));
  Serial.print(g_mp3_play_start);
  Serial.print(F(" playFrames="));
  Serial.print(g_mp3_play_frames);
  Serial.print(F(" ch="));
  Serial.print(g_mp3_decode_channels);
  Serial.print(F(" hz="));
  Serial.print(g_mp3_decode_hz);
  Serial.print(F(" maxAbs="));
  Serial.println(mp3CacheMaxAbs());
  Serial.print(F("  energyOff="));
  Serial.print(g_mp3_energy_off);
  Serial.print(F(" alignScore="));
  Serial.println((int64_t)g_mp3_align_score);

  int32_t maxErr = 0;
  int64_t sumSq = 0;
  const uint32_t cmpN = (g_mp3_play_frames < UZC_MP3_PCM_REF_SAMPLES)
                            ? g_mp3_play_frames
                            : UZC_MP3_PCM_REF_SAMPLES;
  for (uint32_t i = 0; i < cmpN; i++) {
    const int32_t got = g_mp3_pcm_cache[(g_mp3_play_start + i) * 2];
    const int32_t ref = kUzcMp3PcmRef[i];
    const int32_t err = got - ref;
    const int32_t ae = (err < 0) ? -err : err;
    if (ae > maxErr) {
      maxErr = ae;
    }
    sumSq += (int64_t)err * err;
  }
  const uint32_t rms = (cmpN > 0) ? (uint32_t)sqrt((double)sumSq / (double)cmpN) : 0U;
  Serial.print(F("  CMPREF aligned maxErr="));
  Serial.print(maxErr);
  Serial.print(F(" rmsErr="));
  Serial.println(rms);

  Serial.print(F("  raw got[0..7]: "));
  for (uint32_t i = 0; i < 8 && i < g_mp3_pcm_frame_count; i++) {
    Serial.print(g_mp3_pcm_cache[i * 2]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print(F("  aligned got[0..7]: "));
  for (uint32_t i = 0; i < 8 && i < g_mp3_play_frames; i++) {
    Serial.print(g_mp3_pcm_cache[(g_mp3_play_start + i) * 2]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print(F("  ref[0..7]: "));
  for (uint32_t i = 0; i < 8 && i < UZC_MP3_PCM_REF_SAMPLES; i++) {
    Serial.print(kUzcMp3PcmRef[i]);
    Serial.print(' ');
  }
  Serial.println();

  if (g_mp3_play_frames > 0) {
    const int32_t last = g_mp3_pcm_cache[(g_mp3_play_start + g_mp3_play_frames - 1U) * 2];
    const int32_t first = g_mp3_pcm_cache[g_mp3_play_start * 2];
    Serial.print(F("  seam last="));
    Serial.print(last);
    Serial.print(F(" first="));
    Serial.print(first);
    Serial.print(F(" diff="));
    Serial.println(last - first);
  }
}

static void produceMp3Frames() {
  if (!g_mp3_cache_ready || g_mp3_play_frames == 0) {
    return;
  }

  int16_t chunk[RAW_PRODUCE_FRAMES * 2];
  for (uint32_t i = 0; i < RAW_PRODUCE_FRAMES; i++) {
    const uint32_t idx = g_mp3_play_start + (g_mp3_play_idx % g_mp3_play_frames);
    const int16_t l = g_mp3_pcm_cache[idx * 2 + 0];
    const int16_t r = g_mp3_pcm_cache[idx * 2 + 1];
    chunk[i * 2 + 0] = l;
    chunk[i * 2 + 1] = r;
    g_mp3_play_idx++;
  }
  pcmHoldAppendStereo(chunk, RAW_PRODUCE_FRAMES);
}

static bool initI2sTx() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
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

static void produceRawFrames() {
  int16_t chunk[RAW_PRODUCE_FRAMES * 2];
  for (uint32_t i = 0; i < RAW_PRODUCE_FRAMES; i++) {
    const int16_t s = nextToneSample();
    chunk[i * 2 + 0] = s;
    chunk[i * 2 + 1] = s;
  }
  pcmHoldAppendStereo(chunk, RAW_PRODUCE_FRAMES);
  g_stat_raw_frames += RAW_PRODUCE_FRAMES;
}

static bool pcmHoldHasSpace() {
  return pcmHoldUsed() + RAW_PRODUCE_FRAMES <= PCM_HOLD_FRAMES;
}

static void produceOneChunk() {
  if (!pcmHoldHasSpace()) {
    return;
  }
  if (g_mode == OutputMode::RAW) {
    produceRawFrames();
  } else if (g_mode == OutputMode::MP3 && g_mp3_cache_ready) {
    produceMp3Frames();
  }
}

static void prefillPcmHold() {
  while (pcmHoldHasSpace() && pcmHoldUsed() < PCM_HOLD_FRAMES / 2U) {
    produceOneChunk();
  }
}

static void audioProduceTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(5);

  for (;;) {
    if (!g_run) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (!anyTestActive()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      lastWake = xTaskGetTickCount();
      continue;
    }

    produceOneChunk();
    vTaskDelayUntil(&lastWake, periodTicks);
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

    if (!anyTestActive()) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    const uint32_t popped = pcmHoldPop(buf, frameCount);
    if (popped > 0) {
      size_t written = 0;
      (void)i2s_write(I2S_NUM_0, buf, popped * sizeof(int16_t) * 2, &written, portMAX_DELAY);
    } else {
      g_underrun++;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

static void printPrompt() {
  Serial.print(PROMPT);
}

static void printTestCatalog() {
  Serial.println(F("TESTS:"));
  Serial.println(F("  1   441Hz sine"));
  Serial.println(F("      RAW   : local PCM sine"));
  Serial.println(F("      MP3   : embedded slot decode, aligned loop"));
}

static void printCommandHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  ON  <n>       Start test n"));
  Serial.println(F("  OFF <n>       Stop test n"));
  Serial.println(F("  OFF ALL       Stop all tests"));
  Serial.println(F("  MODE RAW      Local 441Hz sine -> I2S speaker"));
  Serial.println(F("  MODE MP3      Embedded slot decode, aligned loop -> I2S speaker"));
  Serial.println(F("  DUMP          Dump decoded MP3 PCM (first 64 samples)"));
  Serial.println(F("  CMPREF        Compare decoded PCM vs encode source ref"));
  Serial.println(F("  STAT          Show active tests / mode / playback status"));
  Serial.println(F("  BUILD         Show build number"));
  Serial.println(F("  TEST          List available tests"));
  Serial.println(F("  HELP          Show this menu"));
  Serial.println(F("  MENU          Same as HELP"));
  Serial.println(F("  ?             Same as HELP"));
}

static void printMenu() {
  Serial.println();
  Serial.println(F("=== mp3test CLI ==="));
  Serial.print(F("Build: "));
  Serial.println(BUILD_NUMBER);
  Serial.println(F("Path: embedded slot / local RAW -> I2S TX -> Speaker (no Master)"));
  Serial.print(F("Mode: "));
  Serial.println(modeName(g_mode));
  Serial.print(F("I2S TX BCLK/WS/DOUT = GPIO"));
  Serial.print(PIN_I2S_TX_BCLK);
  Serial.print('/');
  Serial.print(PIN_I2S_TX_WS);
  Serial.print('/');
  Serial.println(PIN_I2S_TX_DOUT);
  Serial.print(F("Rate: "));
  Serial.print(SAMPLE_RATE);
  Serial.println(F(" Hz, 16bit stereo"));
  Serial.print(F("MP3 slot: max="));
  Serial.print(UZC_MP3_MAX_FRAME_SIZE);
  Serial.print(F(" real="));
  Serial.print(UZC_MP3_REAL_FRAME_SIZE);
  Serial.print(F(" words="));
  Serial.print(UZC_MP3_SLOT_WORDS);
  Serial.print(F(" period="));
  Serial.println(UZC_MP3_PERIOD_STEREO);
  Serial.println();
  printTestCatalog();
  Serial.println();
  printCommandHelp();
  Serial.println();
  Serial.println(F("NOTES:"));
  Serial.println(F("  Example: MODE MP3 -> ON 1"));
  Serial.println();
}

static void printStatus() {
  Serial.println(F("TEST STATUS:"));
  for (uint32_t n = TEST_ID_MIN; n <= TEST_ID_MAX; n++) {
    Serial.print(F("  "));
    Serial.print(n);
    Serial.print(F(" = "));
    Serial.println(g_test_on[n - 1] ? F("ON") : F("OFF"));
  }
  Serial.print(F("MODE: "));
  Serial.println(modeName(g_mode));
  Serial.print(F("I2S TX: "));
  Serial.println(g_i2s_tx_ok ? F("OK") : F("FAILED"));
  Serial.print(F("OUTPUT: "));
  Serial.println(anyTestActive() ? F("ACTIVE") : F("SILENCE"));
  Serial.print(F("HOLD: "));
  Serial.println(pcmHoldUsed());
  Serial.print(F("RAW frames: "));
  Serial.println(g_stat_raw_frames);
  Serial.print(F("MP3 cache frames: "));
  Serial.print(g_mp3_pcm_frame_count);
  Serial.print(F(" playStart="));
  Serial.print(g_mp3_play_start);
  Serial.print(F(" playFrames="));
  Serial.print(g_mp3_play_frames);
  Serial.print(F(" maxAbs="));
  Serial.println(mp3CacheMaxAbs());
  Serial.print(F("MP3 decode ok/fail: "));
  Serial.print(g_stat_mp3_decode_ok);
  Serial.print('/');
  Serial.println(g_stat_mp3_decode_fail);
  Serial.print(F("PCM drop: "));
  Serial.println(g_stat_pcm_drop);
  Serial.print(F("Underrun: "));
  Serial.println(g_underrun);
}

static void cmdDump() {
  if (!g_mp3_cache_ready && !prepareMp3PcmCache()) {
    Serial.println(F("ERR MP3 cache not ready"));
    return;
  }
  const uint32_t n = (g_mp3_play_frames < 64U) ? g_mp3_play_frames : 64U;
  Serial.print(F("DUMP aligned L[0.."));
  Serial.print(n - 1);
  Serial.println(F("]:"));
  for (uint32_t i = 0; i < n; i++) {
    Serial.print(g_mp3_pcm_cache[(g_mp3_play_start + i) * 2]);
    if ((i % 16U) == 15U) {
      Serial.println();
    } else {
      Serial.print(' ');
    }
  }
  if ((n % 16U) != 0U) {
    Serial.println();
  }
  if (g_mp3_play_frames > 0) {
    Serial.print(F("seam last="));
    Serial.print(g_mp3_pcm_cache[(g_mp3_play_start + g_mp3_play_frames - 1U) * 2]);
    Serial.print(F(" first="));
    Serial.println(g_mp3_pcm_cache[g_mp3_play_start * 2]);
  }
  Serial.println(F("OK DUMP"));
}

static void cmdCmpRef() {
  if (!g_mp3_cache_ready && !prepareMp3PcmCache()) {
    Serial.println(F("ERR MP3 cache not ready"));
    return;
  }
  const uint32_t cmpN = (g_mp3_play_frames < UZC_MP3_PCM_REF_SAMPLES)
                            ? g_mp3_play_frames
                            : UZC_MP3_PCM_REF_SAMPLES;
  int32_t maxErr = 0;
  int64_t sumSq = 0;
  for (uint32_t i = 0; i < cmpN; i++) {
    const int32_t got = g_mp3_pcm_cache[(g_mp3_play_start + i) * 2];
    const int32_t ref = kUzcMp3PcmRef[i];
    const int32_t err = got - ref;
    const int32_t absErr = (err < 0) ? -err : err;
    if (absErr > maxErr) {
      maxErr = absErr;
    }
    sumSq += (int64_t)err * err;
  }
  const uint32_t rms = (cmpN > 0) ? (uint32_t)sqrt((double)sumSq / (double)cmpN) : 0U;
  Serial.print(F("CMPREF aligned samples="));
  Serial.print(cmpN);
  Serial.print(F(" maxErr="));
  Serial.print(maxErr);
  Serial.print(F(" rmsErr="));
  Serial.println(rms);
  Serial.print(F("  got[0..3]: "));
  for (uint32_t i = 0; i < 4 && i < g_mp3_play_frames; i++) {
    Serial.print(g_mp3_pcm_cache[(g_mp3_play_start + i) * 2]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print(F("  ref[0..3]: "));
  for (uint32_t i = 0; i < 4 && i < UZC_MP3_PCM_REF_SAMPLES; i++) {
    Serial.print(kUzcMp3PcmRef[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(F("OK CMPREF"));
}

static bool parseTestId(const String& args, uint32_t& testOut) {
  String s = args;
  s.trim();
  if (s.length() == 0) {
    return false;
  }
  const int n = s.toInt();
  if (n < (int)TEST_ID_MIN || n > (int)TEST_ID_MAX) {
    Serial.print(F("ERR BAD_TEST (use "));
    Serial.print(TEST_ID_MIN);
    Serial.print('-');
    Serial.print(TEST_ID_MAX);
    Serial.println(')');
    return false;
  }
  testOut = (uint32_t)n;
  return true;
}

static void cmdOn(const String& args) {
  uint32_t testId = 0;
  if (!parseTestId(args, testId)) {
    Serial.println(F("USAGE: ON <n>   (see TEST)"));
    return;
  }
  pcmHoldReset();
  g_phase_acc = 0;
  g_mp3_play_idx = 0;
  g_underrun = 0;
  memset(&g_mp3dec, 0, sizeof(g_mp3dec));
  if (g_mode == OutputMode::MP3) {
    (void)prepareMp3PcmCache();
  }
  prefillPcmHold();
  g_test_on[testId - 1] = true;
  Serial.print(F("OK ON "));
  Serial.println(testId);
}

static void cmdOff(const String& args) {
  String s = args;
  s.trim();
  s.toUpperCase();

  if (s.length() == 0) {
    Serial.println(F("USAGE: OFF <n> | OFF ALL"));
    return;
  }

  if (s == "ALL") {
    for (uint32_t i = 0; i < TEST_ID_MAX; i++) {
      g_test_on[i] = false;
    }
    g_phase_acc = 0;
    pcmHoldReset();
    Serial.println(F("OK OFF ALL"));
    return;
  }

  uint32_t testId = 0;
  if (!parseTestId(s, testId)) {
    return;
  }
  g_test_on[testId - 1] = false;
  if (!anyTestActive()) {
    g_phase_acc = 0;
    pcmHoldReset();
  }
  Serial.print(F("OK OFF "));
  Serial.println(testId);
}

static void cmdMode(const String& args) {
  String s = args;
  s.trim();
  s.toUpperCase();
  if (s == "RAW") {
    g_mode = OutputMode::RAW;
    Serial.println(F("OK MODE RAW"));
    return;
  }
  if (s == "MP3") {
    g_mode = OutputMode::MP3;
    g_mp3_play_idx = 0;
    if (prepareMp3PcmCache()) {
      Serial.println(F("OK MODE MP3"));
    } else {
      Serial.println(F("ERR MODE MP3 (decode failed)"));
    }
    return;
  }
  Serial.println(F("USAGE: MODE RAW | MODE MP3"));
}

static void parseCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String upper = line;
  upper.toUpperCase();

  int space = upper.indexOf(' ');
  String cmd = (space >= 0) ? upper.substring(0, space) : upper;
  String args = (space >= 0) ? line.substring(space + 1) : String();

  if (cmd == "ON") {
    cmdOn(args);
    return;
  }
  if (cmd == "OFF") {
    cmdOff(args);
    return;
  }
  if (cmd == "MODE") {
    cmdMode(args);
    return;
  }
  if (cmd == "STAT") {
    printStatus();
    Serial.println(F("OK STAT"));
    return;
  }
  if (cmd == "DUMP") {
    cmdDump();
    return;
  }
  if (cmd == "CMPREF") {
    cmdCmpRef();
    return;
  }
  if (cmd == "BUILD") {
    Serial.print(F("BUILD "));
    Serial.println(BUILD_NUMBER);
    return;
  }
  if (cmd == "TEST") {
    printTestCatalog();
    Serial.println(F("OK TEST"));
    return;
  }
  if (cmd == "HELP" || cmd == "MENU" || cmd == "?") {
    printMenu();
    Serial.println(F("OK HELP"));
    return;
  }

  Serial.println(F("ERR UNKNOWN_CMD (type HELP)"));
}

static void processSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      Serial.println();
      parseCommand(g_input_line);
      g_input_line = "";
      printPrompt();
      continue;
    }
    if (c == '\b' || c == 127) {
      if (g_input_line.length() > 0) {
        g_input_line.remove(g_input_line.length() - 1);
        Serial.print(F("\b \b"));
      }
      continue;
    }
    if (c >= 32 && c <= 126) {
      Serial.print(c);
      g_input_line += c;
    }
  }
}

#ifdef WAVE_CHECK

void setup() {
  Serial.begin(115200);
  delay(500);
  initSlotBufferFromWords();
  if (!prepareMp3PcmCache()) {
    for (;;) {
      Serial.println(0);
      delay(100);
    }
  }
  printMp3Diagnostics();
}

void loop() {
  for (uint32_t i = 0; i < g_mp3_play_frames; i++) {
    Serial.println(g_mp3_pcm_cache[(g_mp3_play_start + i) * 2 + 0]);
  }
}

#else

void setup() {
  Serial.begin(115200);
  delay(300);
  initSineLut();
  initSlotBufferFromWords();
  printMp3Diagnostics();

  Serial.println();
  Serial.print(F("mp3test CLI ready  build "));
  Serial.println(BUILD_NUMBER);
  printMenu();
  printPrompt();

  g_i2s_tx_ok = initI2sTx();
  if (g_i2s_tx_ok) {
    Serial.println(F("I2S TX OK (master -> speaker)"));
  } else {
    Serial.println(F("FATAL: I2S TX init failed"));
  }

  xTaskCreatePinnedToCore(audioProduceTask, "produce", 6144, nullptr, 3, &g_produce_task, 0);
  xTaskCreatePinnedToCore(i2sTxTask, "i2sTx", 4096, nullptr, 2, &g_tx_task, 1);

  printPrompt();
}

void loop() {
  processSerial();

  static uint32_t hb = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - hb) >= 2000U) {
    hb = now;
    if (anyTestActive()) {
      Serial.print(F("[HB] mode="));
      Serial.print(modeName(g_mode));
      Serial.print(F(" hold="));
      Serial.print(pcmHoldUsed());
      Serial.print(F(" u="));
      Serial.print(g_underrun);
      if (g_mode == OutputMode::MP3) {
        Serial.print(F(" mp3Start="));
        Serial.print(g_mp3_play_start);
        Serial.print(F(" playFrames="));
        Serial.print(g_mp3_play_frames);
        Serial.print(F(" maxAbs="));
        Serial.print(mp3CacheMaxAbs());
        Serial.print(F(" drop="));
        Serial.print(g_stat_pcm_drop);
      } else {
        Serial.print(F(" raw="));
        Serial.print(g_stat_raw_frames);
      }
      Serial.println();
    }
  }
}

#endif  // WAVE_CHECK

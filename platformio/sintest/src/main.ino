/*
 * UzuCast TDM Sine Test (Master)
 * ESP32-S3 — 8ch TDM 16bit 44100Hz, 440Hz sine on CH1-CH5
 * Reference: FPGA/Release/TestApp/TDM_SINWAVE
 *
 * CH1 ON 時は Sub I2S ステレオ R(slot2) と TDM CH8 の両方に 0xAAAA を出力する。
 */

#include <Arduino.h>
#include <math.h>
#include "driver/i2s_tdm.h"

static constexpr uint32_t BUILD_NUMBER = 3;
static constexpr gpio_num_t I2S_BCLK_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t I2S_WS_GPIO   = GPIO_NUM_16;
static constexpr gpio_num_t I2S_DOUT_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t I2S_MCLK_GPIO = I2S_GPIO_UNUSED;

static constexpr uint32_t SAMPLE_RATE    = 44100;
static constexpr uint32_t TDM_NUM_CH      = 8;
static constexpr uint32_t FRAME_SAMPLES   = 256;
static constexpr float    TONE_FREQ_HZ    = 440.0f;
static constexpr int16_t  TONE_LEVEL      = 12000;
static constexpr int16_t  I2S_TAG_RAW     = (int16_t)0xAAAA;

static constexpr uint32_t CH_CMD_MIN = 1;
static constexpr uint32_t CH_CMD_MAX = 5;
static constexpr uint32_t CH2_INDEX  = 1;  // Sub I2S stereo R = TDM slot 2
static constexpr uint32_t CH8_INDEX  = 7;

static constexpr uint32_t SINE_LUT_SIZE = 512;
static constexpr uint32_t PHASE_FRAC_BITS = 8;
static constexpr uint32_t PHASE_INC =
    (uint32_t)(((uint64_t)SINE_LUT_SIZE << PHASE_FRAC_BITS) * (uint64_t)TONE_FREQ_HZ / SAMPLE_RATE);

static const char PROMPT[] = "# ";

static i2s_chan_handle_t g_tx_handle = nullptr;
static volatile bool g_ch_enable[TDM_NUM_CH] = {};
static int16_t g_sine_lut[SINE_LUT_SIZE];
static uint32_t g_phase_acc = 0;
static String g_input_line;

static void printPrompt() {
  Serial.print(PROMPT);
}

static void printMenu() {
  Serial.println();
  Serial.println("=== UZU CAST TDM SINE TEST ===");
  Serial.print("Build: ");
  Serial.println(BUILD_NUMBER);
  Serial.print("TDM: ");
  Serial.print(SAMPLE_RATE);
  Serial.println(" Hz, 8ch 16bit, tone 440 Hz");
  Serial.println();
  Serial.println("COMMANDS:");
  Serial.println("  ON  <n>     Turn channel n on  (n=1..5, sine wave)");
  Serial.println("  OFF <n>     Turn channel n off (n=1..5)");
  Serial.println("  STAT        Show channel status");
  Serial.println("  HELP        Show this menu");
  Serial.println("  MENU        Same as HELP");
  Serial.println();
  Serial.println("NOTES:");
  Serial.println("  CH1 ON drives CH2(R)=0xAAAA + CH8=0xAAAA (Sub RAW tag path)");
  Serial.println();
}

static void printStatus() {
  Serial.print("CH STATUS: ");
  for (uint32_t i = 0; i < TDM_NUM_CH; i++) {
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(g_ch_enable[i] ? "ON" : "OFF");
    Serial.print(' ');
  }
  Serial.println();
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

static bool parseChannelArg(const String& args, uint32_t& chOut) {
  String s = args;
  s.trim();
  if (s.length() == 0) {
    return false;
  }
  const int n = s.toInt();
  if (n < (int)CH_CMD_MIN || n > (int)CH_CMD_MAX) {
    Serial.print("ERR BAD_CH (use ");
    Serial.print(CH_CMD_MIN);
    Serial.print('-');
    Serial.print(CH_CMD_MAX);
    Serial.println(')');
    return false;
  }
  chOut = (uint32_t)n;
  return true;
}

static void setChannel(uint32_t ch1based, bool on) {
  g_ch_enable[ch1based - 1] = on;
  Serial.print("OK ");
  Serial.print(on ? "ON " : "OFF ");
  Serial.println(ch1based);
  printStatus();
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
    uint32_t ch = 0;
    if (!parseChannelArg(args, ch)) {
      Serial.println("USAGE: ON <n>   (n=1..5)");
      return;
    }
    setChannel(ch, true);
    return;
  }

  if (cmd == "OFF") {
    uint32_t ch = 0;
    if (!parseChannelArg(args, ch)) {
      Serial.println("USAGE: OFF <n>   (n=1..5)");
      return;
    }
    setChannel(ch, false);
    return;
  }

  if (cmd == "STAT") {
    printStatus();
    Serial.println("OK STAT");
    return;
  }

  if (cmd == "HELP" || cmd == "MENU" || cmd == "?") {
    printMenu();
    Serial.println("OK HELP");
    return;
  }

  Serial.println("ERR UNKNOWN_CMD (type HELP)");
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
        Serial.print("\b \b");
      }
      continue;
    }

    if (c >= 32 && c <= 126) {
      Serial.print(c);
      g_input_line += c;
    }
  }
}

static bool initTdm() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 16;
  chan_cfg.dma_frame_num = FRAME_SAMPLES;

  esp_err_t err = i2s_new_channel(&chan_cfg, &g_tx_handle, nullptr);
  if (err != ESP_OK) {
    Serial.printf("i2s_new_channel error: %d\n", (int)err);
    return false;
  }

  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO,
          (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
                                I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7)),
      .gpio_cfg = {
          .mclk = I2S_MCLK_GPIO,
          .bclk = I2S_BCLK_GPIO,
          .ws = I2S_WS_GPIO,
          .dout = I2S_DOUT_GPIO,
          .din = I2S_GPIO_UNUSED,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      },
  };

  tdm_cfg.slot_cfg.total_slot = TDM_NUM_CH;
  tdm_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;

  err = i2s_channel_init_tdm_mode(g_tx_handle, &tdm_cfg);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_init_tdm_mode error: %d\n", (int)err);
    return false;
  }

  err = i2s_channel_enable(g_tx_handle);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_enable error: %d\n", (int)err);
    return false;
  }

  Serial.printf("TDM init OK (%lu Hz, 8ch, 16bit)\n", (unsigned long)SAMPLE_RATE);
  return true;
}

static void serviceTdmOutput() {
  if (!g_tx_handle) {
    return;
  }

  static int16_t buffer[FRAME_SAMPLES * TDM_NUM_CH];
  bool ch_on[TDM_NUM_CH];
  for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) {
    ch_on[ch] = g_ch_enable[ch];
  }

  for (uint32_t frame = 0; frame < FRAME_SAMPLES; frame++) {
    const int16_t tone = nextToneSample();
    const uint32_t base = frame * TDM_NUM_CH;

    for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) {
      buffer[base + ch] = ch_on[ch] ? tone : 0;
    }

    if (ch_on[0]) {
      buffer[base + CH2_INDEX] = I2S_TAG_RAW;
      buffer[base + CH8_INDEX] = I2S_TAG_RAW;
    }
  }

  size_t bytesWritten = 0;
  (void)i2s_channel_write(g_tx_handle, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
}

static void tdmOutputTask(void* /*param*/) {
  for (;;) {
    serviceTdmOutput();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  initSineLut();
  Serial.println();
  printMenu();
  printPrompt();

  if (!initTdm()) {
    Serial.println("FATAL: TDM init failed");
    return;
  }

  xTaskCreatePinnedToCore(tdmOutputTask, "tdmOut", 4096, nullptr, 5, nullptr, 1);
}

void loop() {
  processSerial();
  vTaskDelay(pdMS_TO_TICKS(10));
}

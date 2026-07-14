/*
 * sintest_i2s_mp3 — Master (ESP32)
 * Stage 2b: Master I2S -> Slave I2S -> Bluetooth Speaker (RAW or MP3 tag path)
 *
 * Stereo I2S 44.1kHz 16bit (STD Philips I2S)
 *   mode raw: L = 441 Hz sine, R = 0xAAAA
 *   mode mp3: UZC V2 slot (441 Hz sine MP3) with AA00/AA55/5500 + period pad
 *
 * Serial CLI: 115200 8N1, prompt "#"
 * Pins: MP3テスト回路/SD_I2S/ピンアサイン.md
 */

#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>

#include "uzc_mp3_slot.h"

static constexpr uint32_t BUILD_NUMBER = 5;

// Master I2S TX -> Slave I2S RX
static const int PIN_I2S_BCLK = 27;
static const int PIN_I2S_WS   = 26;
static const int PIN_I2S_DOUT = 25;

static constexpr uint32_t SAMPLE_RATE   = 44100;
static constexpr uint32_t FRAME_SAMPLES = 256;
static constexpr float    TONE_FREQ_HZ  = 441.0f;
static constexpr int16_t  TONE_LEVEL    = 12000;
static constexpr int16_t  I2S_TAG_RAW       = (int16_t)0xAAAA;
static constexpr int16_t  I2S_TAG_MP3_START = (int16_t)0xAA00;
static constexpr int16_t  I2S_TAG_SLOT_DATA = (int16_t)0xAA55;
static constexpr int16_t  I2S_TAG_MP3_END   = (int16_t)0x5500;

static constexpr uint32_t TEST_ID_MIN = 1;
static constexpr uint32_t TEST_ID_MAX = 1;

static constexpr uint32_t SINE_LUT_SIZE = 512;
static constexpr uint32_t PHASE_FRAC_BITS = 8;
static constexpr uint32_t PHASE_INC =
    (uint32_t)(((uint64_t)SINE_LUT_SIZE << PHASE_FRAC_BITS) * (uint64_t)TONE_FREQ_HZ / SAMPLE_RATE);

static const char PROMPT[] = "# ";

enum class OutputMode : uint8_t {
  RAW = 0,
  MP3 = 1,
};

static I2SClass g_i2s;
static bool g_i2s_ok = false;
static volatile bool g_test_on[TEST_ID_MAX] = {};
static OutputMode g_mode = OutputMode::RAW;
static int16_t g_sine_lut[SINE_LUT_SIZE];
static uint32_t g_phase_acc = 0;
static uint32_t g_mp3_period_pos = 0;
static String g_input_line;

static const char* modeName(OutputMode mode) {
  return (mode == OutputMode::RAW) ? "RAW" : "MP3";
}

static void resetMp3Period() {
  g_mp3_period_pos = 0;
}

static void printPrompt() {
  Serial.print(PROMPT);
}

static void printTestCatalog() {
  Serial.println(F("TESTS:"));
  Serial.println(F("  1   441Hz sine (mode raw: R=AAAA, mode mp3: UZC slot stream)"));
}

static void printCommandHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  ON  <n>       Start test n"));
  Serial.println(F("  OFF <n>       Stop test n"));
  Serial.println(F("  OFF ALL       Stop all tests"));
  Serial.println(F("  MODE RAW      I2S RAW tag path (R=0xAAAA)"));
  Serial.println(F("  MODE MP3      I2S UZC MP3 slot path (R=AA00/AA55/5500)"));
  Serial.println(F("  STAT          Show active tests / mode / I2S status"));
  Serial.println(F("  BUILD         Show build number"));
  Serial.println(F("  TEST          List available tests"));
  Serial.println(F("  HELP          Show this menu"));
  Serial.println(F("  MENU          Same as HELP"));
  Serial.println(F("  ?             Same as HELP"));
}

static void printMenu() {
  Serial.println();
  Serial.println(F("=== sintest_i2s_mp3 MASTER CLI ==="));
  Serial.print(F("Build: "));
  Serial.println(BUILD_NUMBER);
  Serial.println(F("Path: Master I2S -> Slave I2S -> Bluetooth (Stage 2 MP3 test)"));
  Serial.print(F("Mode: "));
  Serial.println(modeName(g_mode));
  Serial.print(F("I2S TX BCLK/WS/DOUT = GPIO"));
  Serial.print(PIN_I2S_BCLK);
  Serial.print('/');
  Serial.print(PIN_I2S_WS);
  Serial.print('/');
  Serial.println(PIN_I2S_DOUT);
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
  Serial.println(F("  Example: MODE MP3  ->  ON 1"));
  Serial.println();
}

static bool anyTestActive() {
  for (uint32_t i = 0; i < TEST_ID_MAX; i++) {
    if (g_test_on[i]) {
      return true;
    }
  }
  return false;
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
  Serial.println(g_i2s_ok ? F("OK") : F("FAILED"));
  Serial.print(F("OUTPUT: "));
  Serial.println(anyTestActive() ? F("ACTIVE") : F("SILENCE"));
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

static bool initI2sTx() {
  g_i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT);
  if (!g_i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.print(F("I2S begin failed: "));
    Serial.println(g_i2s.lastError());
    return false;
  }
  return true;
}

static void fillRawStereoFrame(int16_t* outL, int16_t* outR) {
  *outL = nextToneSample();
  *outR = I2S_TAG_RAW;
}

static void fillMp3StereoFrame(int16_t* outL, int16_t* outR) {
  if (g_mp3_period_pos == 0) {
    *outL = 0;
    *outR = I2S_TAG_MP3_START;
  } else if (g_mp3_period_pos >= 1 && g_mp3_period_pos <= UZC_MP3_SLOT_WORDS) {
    *outL = kUzcMp3SlotWords[g_mp3_period_pos - 1];
    *outR = I2S_TAG_SLOT_DATA;
  } else if (g_mp3_period_pos == (UZC_MP3_SLOT_WORDS + 1)) {
    *outL = 0;
    *outR = I2S_TAG_MP3_END;
  } else {
    *outL = 0;
    *outR = 0;
  }

  g_mp3_period_pos++;
  if (g_mp3_period_pos >= UZC_MP3_PERIOD_STEREO) {
    g_mp3_period_pos = 0;
  }
}

static void serviceI2sOutput() {
  if (!g_i2s_ok) {
    return;
  }

  int16_t buffer[FRAME_SAMPLES * 2];
  const bool active = anyTestActive();

  for (uint32_t i = 0; i < FRAME_SAMPLES; i++) {
    if (active && g_test_on[0]) {
      int16_t l = 0;
      int16_t r = 0;
      if (g_mode == OutputMode::RAW) {
        fillRawStereoFrame(&l, &r);
      } else {
        fillMp3StereoFrame(&l, &r);
      }
      buffer[i * 2 + 0] = l;
      buffer[i * 2 + 1] = r;
    } else {
      buffer[i * 2 + 0] = 0;
      buffer[i * 2 + 1] = 0;
    }
  }
  g_i2s.write((const uint8_t*)buffer, sizeof(buffer));
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
  if (g_mode == OutputMode::MP3) {
    resetMp3Period();
  }
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
    resetMp3Period();
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
    resetMp3Period();
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
    resetMp3Period();
    Serial.println(F("OK MODE RAW"));
    return;
  }
  if (s == "MP3") {
    g_mode = OutputMode::MP3;
    resetMp3Period();
    Serial.println(F("OK MODE MP3"));
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

void setup() {
  Serial.begin(115200);
  delay(300);
  initSineLut();

  Serial.println();
  Serial.print(F("sintest_i2s_mp3 Master CLI ready  build "));
  Serial.println(BUILD_NUMBER);
  printMenu();
  printPrompt();

  g_i2s_ok = initI2sTx();
  if (g_i2s_ok) {
    Serial.println(F("I2S TX OK"));
  } else {
    Serial.println(F("FATAL: I2S TX init failed"));
  }
  printPrompt();
}

void loop() {
  processSerial();
  serviceI2sOutput();
}

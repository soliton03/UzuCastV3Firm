#include "tdm_tx.h"

#include <Arduino.h>

static constexpr uint32_t kSampleRate = 44100;
static constexpr size_t kPeriodFrames = 1152;
static constexpr size_t kSlotFrames = 159;
static constexpr size_t kPaceChunkFrames = 64;
static constexpr size_t kPhaseFrames = 2048;
static constexpr size_t kSyncFrames = 128;

static int16_t g_ch1Words[kSlotFrames];
static int16_t g_periodBuf[kPeriodFrames * TDM_NUM_CH];

enum TestMode : uint8_t {
  MODE_UZC_LOOP = 1,
  MODE_RAW_AAAA = 2,
  MODE_STEREO_PAIRS = 3,
  MODE_FIXED_5555_AAAA = 5,
  MODE_SWEEP = 6,
  MODE_IDLE = 0,
};

static TestMode g_mode = MODE_SWEEP;
static uint32_t g_periodSeq = 0;
static uint32_t g_rawCounter = 0;
static uint32_t g_sweepCounter = 0;

static uint16_t tagU16(int16_t v) {
  return (uint16_t)v;
}

static bool tdmWriteFixedPaced(int16_t ch1, int16_t ch8, size_t frameCount) {
  static int16_t buf[kPaceChunkFrames * TDM_NUM_CH];
  size_t sent = 0;
  const uint32_t t0 = micros();
  while (sent < frameCount) {
    const size_t chunk = (frameCount - sent < kPaceChunkFrames) ? (frameCount - sent) : kPaceChunkFrames;
    for (size_t i = 0; i < chunk; i++) {
      int16_t* f = buf + i * TDM_NUM_CH;
      f[0] = ch1;
      for (uint32_t ch = 1; ch < TDM_NUM_CH - 1; ch++) {
        f[ch] = 0;
      }
      f[TDM_TAG_CH] = ch8;
    }
    if (!tdmWriteFrames(buf, chunk)) {
      return false;
    }
    sent += chunk;
    const uint32_t elapsedUs = micros() - t0;
    const uint32_t targetUs = (uint32_t)((sent * 1000000ULL) / kSampleRate);
    if (targetUs > elapsedUs) {
      delayMicroseconds(targetUs - elapsedUs);
    }
  }
  return true;
}

static void sendSyncMarker() {
  Serial.println(F("  SYNC R=FFFF"));
  tdmWriteFixedPaced(0, (int16_t)0xFFFF, kSyncFrames);
}

static void sendPhase(const char* name, int16_t ch1, int16_t ch8, size_t frames) {
  Serial.printf("PHASE %s TX CH1=%04X CH8=%04X frames=%u\n", name, tagU16(ch1), tagU16(ch8),
                (unsigned)frames);
  tdmWriteFixedPaced(ch1, ch8, frames);
}

static void sendCounterPhase(size_t frames) {
  Serial.printf("PHASE COUNTER TX CH1=1000+i R=AAAA frames=%u\n", (unsigned)frames);
  static int16_t buf[kPaceChunkFrames * TDM_NUM_CH];
  size_t sent = 0;
  uint16_t counter = 0;
  const uint32_t t0 = micros();
  while (sent < frames) {
    const size_t chunk = (frames - sent < kPaceChunkFrames) ? (frames - sent) : kPaceChunkFrames;
    for (size_t i = 0; i < chunk; i++) {
      int16_t* f = buf + i * TDM_NUM_CH;
      f[0] = (int16_t)(0x1000 + (counter & 0xFF));
      for (uint32_t ch = 1; ch < TDM_NUM_CH - 1; ch++) {
        f[ch] = 0;
      }
      f[TDM_TAG_CH] = (int16_t)0xAAAA;
      counter++;
    }
    if (!tdmWriteFrames(buf, chunk)) {
      return;
    }
    sent += chunk;
    const uint32_t elapsedUs = micros() - t0;
    const uint32_t targetUs = (uint32_t)((sent * 1000000ULL) / kSampleRate);
    if (targetUs > elapsedUs) {
      delayMicroseconds(targetUs - elapsedUs);
    }
  }
}

static void runSweepOnce() {
  g_sweepCounter++;
  Serial.printf("\n======== SWEEP #%lu ========\n", (unsigned long)g_sweepCounter);

  sendSyncMarker();
  sendPhase("5555_AAAA", (int16_t)0x5555, (int16_t)0xAAAA, kPhaseFrames);

  sendSyncMarker();
  sendPhase("AAAA_AAAA", (int16_t)0xAAAA, (int16_t)0xAAAA, kPhaseFrames);

  sendSyncMarker();
  sendPhase("0000_AAAA", 0, (int16_t)0xAAAA, kPhaseFrames);

  sendSyncMarker();
  sendPhase("0001_AA55", (int16_t)0x0001, (int16_t)0xAA55, kPhaseFrames);

  sendSyncMarker();
  sendPhase("009F_AA55", (int16_t)0x009F, (int16_t)0xAA55, kPhaseFrames);

  sendSyncMarker();
  sendPhase("0000_5500", 0, (int16_t)0x5500, kPhaseFrames);

  sendSyncMarker();
  sendPhase("8000_AAAA", (int16_t)0x8000, (int16_t)0xAAAA, kPhaseFrames);

  sendSyncMarker();
  sendCounterPhase(kPhaseFrames);

  sendSyncMarker();
  Serial.println(F("SWEEP done"));
}

static void buildUzcTestPeriod() {
  for (size_t i = 0; i < kSlotFrames; i++) {
    g_ch1Words[i] = (int16_t)(i + 1);
  }
  fillMp3TdmPeriodTaggedMono(g_ch1Words, kSlotFrames, g_periodBuf, kPeriodFrames, kSlotFrames);
}

static bool tdmWriteFramesPaced(const int16_t* tdm, size_t frameCount) {
  size_t sent = 0;
  const uint32_t t0 = micros();
  while (sent < frameCount) {
    const size_t chunk = (frameCount - sent < kPaceChunkFrames) ? (frameCount - sent) : kPaceChunkFrames;
    if (!tdmWriteFrames(tdm + sent * TDM_NUM_CH, chunk)) {
      return false;
    }
    sent += chunk;
    const uint32_t elapsedUs = micros() - t0;
    const uint32_t targetUs = (uint32_t)((sent * 1000000ULL) / kSampleRate);
    if (targetUs > elapsedUs) {
      delayMicroseconds(targetUs - elapsedUs);
    }
  }
  return true;
}

static void sendUzcPeriodOnce() {
  buildUzcTestPeriod();
  g_periodSeq++;
  Serial.printf("\n=== UZC period #%lu ===\n", (unsigned long)g_periodSeq);
  if (!tdmWriteFramesPaced(g_periodBuf, kPeriodFrames)) {
    return;
  }
  Serial.println(F("TX period done"));
}

static void runUzcLoopStep() {
  sendUzcPeriodOnce();
  tdmWriteInvalidKeepalive(256);
  delay(500);
}

static void runRawAaaaStep() {
  static int16_t mono[kPaceChunkFrames];
  for (size_t i = 0; i < kPaceChunkFrames; i++) {
    mono[i] = (int16_t)(g_rawCounter + i);
  }
  g_rawCounter += kPaceChunkFrames;
  if (!tdmWriteRawMono(mono, kPaceChunkFrames)) {
    Serial.println(F("RAW write failed"));
  }
  delayMicroseconds((uint32_t)((kPaceChunkFrames * 1000000ULL) / kSampleRate));
}

static void runFixed8000_0001Step() {
  tdmWriteFixedPaced((int16_t)0x8000, (int16_t)0x0001, kPaceChunkFrames);
  delayMicroseconds((uint32_t)((kPaceChunkFrames * 1000000ULL) / kSampleRate));
}

static void printHelp() {
  Serial.println(F("\nTDM-I2S-TEST Master commands:"));
  Serial.println(F("  6  Pattern sweep (default)"));
  Serial.println(F("  1  UZC period loop"));
  Serial.println(F("  5  Fixed CH1=8000 CH8=0001 (MSB/LSB boundary test)"));
  Serial.println(F("  0  Stop"));
  Serial.println(F("  h  Help"));
}

static void handleSerialCommand() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    switch (c) {
      case '6':
        g_mode = MODE_SWEEP;
        Serial.println(F("Mode: pattern sweep"));
        break;
      case '1':
        g_mode = MODE_UZC_LOOP;
        Serial.println(F("Mode: UZC period loop"));
        break;
      case '5':
        g_mode = MODE_FIXED_5555_AAAA;
        Serial.println(F("Mode: CH1=8000 CH8=0001"));
        break;
      case '0':
        g_mode = MODE_IDLE;
        Serial.println(F("Mode: idle"));
        break;
      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;
      default:
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\nTDM-I2S-TEST Master (ESP32-S3)"));
  if (!tdmTxInit(kSampleRate) || !tdmTxEnsureEnabled()) {
    Serial.println(F("TDM init FAILED"));
    while (true) {
      delay(1000);
    }
  }
  printHelp();
  Serial.println(F("Starting mode 6 (pattern sweep)..."));
  g_mode = MODE_SWEEP;
}

void loop() {
  handleSerialCommand();
  switch (g_mode) {
    case MODE_SWEEP:
      runSweepOnce();
      delay(1500);
      break;
    case MODE_UZC_LOOP:
      runUzcLoopStep();
      break;
    case MODE_RAW_AAAA:
      runRawAaaaStep();
      break;
    case MODE_FIXED_5555_AAAA:
      runFixed8000_0001Step();
      break;
    case MODE_IDLE:
      tdmWriteInvalidKeepalive(64);
      delay(100);
      break;
    default:
      delay(100);
      break;
  }
}

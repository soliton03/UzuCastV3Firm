/*
 * uzcTest_i2s_bt — Master (PlatformIO)
 * Ported from MP3TEST2/Arduino/master (SD_I2S)
 *
 * SD_I2S - ESP32 DevKit-C
 * シリアル CLI + SD (SPI) + I2S
 *   WAV: 44.1kHz PCM16 再生 (play)
 *   UZC: mplay=Main MP3デコード→PCM, splay=圧縮I2S (Slaveデコード)
 *   I2S: 常時 44.1kHz。L=データ, R=制御タグ (AAAA/AA00/AA55/5500/FFFF/0000)
 *        play で UZC 指定時は splay 相当
 * 115200 8N1, プロンプト "#", 改行は \n のみ
 */

#ifndef BUILD_NUMBER
#define BUILD_NUMBER 43
#endif

#include <SPI.h>
#include <SD.h>
#include <ESP_I2S.h>
#include <strings.h>

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_STATIC_SCRATCH
#include "minimp3.h"

// I2S — ピンアサイン.md
static const int PIN_I2S_BCLK = 27;
static const int PIN_I2S_WS   = 26;
static const int PIN_I2S_DOUT = 25;

// SD（SPI）— ピンアサイン.md
static const int PIN_SD_CS   = 5;
static const int PIN_SD_MOSI = 23;
static const int PIN_SD_MISO = 19;
static const int PIN_SD_SCK  = 18;
static const int PIN_SD_CD   = 17;  // LOW = カード挿入（プルアップ）
static const int PIN_LED     = 13;

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SD_SPI_HZ = 20000000;
static const uint32_t I2S_SAMPLE_RATE = 44100;
static const char PROMPT[] = "#";
static const size_t CLI_LINE_MAX = 128;
static const size_t TYPE_MAX_BYTES = 16384;
static const size_t CATALOG_MAX = 64;
static const size_t PLAY_READ_BYTES = 2048;
static const uint32_t UZC_DEFAULT_HEADER_SIZE = 32768;
static const uint16_t UZC_CODEC_MP3 = 0;
static const uint16_t UZC_MAX_FRAME_SIZE_FIELD = 2;
static const uint16_t UZC_REAL_FRAME_SIZE_FIELD = 2;
static const uint16_t UZC_SLOT_META_BYTES = 4;  // MaxFrameSizeField + RealFrameSizeField
static const uint16_t UZC_FRAME_DATA_OFFSET = 4;
static const size_t UZC_MAX_SLOT_SIZE = 520;
static const size_t UZC_MAX_STEREO_WORDS = 256;
// 44.1kHz で 1 UZC スロット期間分のステレオフレーム上限（低レート I2S TX 不調回避）
static const size_t UZC_I2S_PERIOD_STEREO_MAX = 1280;

// I2S R-ch 制御タグ (Slave と共通)。デフォルト R=0x0000 (無効)
static const int16_t I2S_TAG_RAW        = (int16_t)0xAAAA;
static const int16_t I2S_TAG_MP3_START  = (int16_t)0xAA00;
static const int16_t I2S_TAG_MP3_END    = (int16_t)0x5500;
static const int16_t I2S_TAG_PAD        = (int16_t)0xFFFF;
static const int16_t I2S_TAG_SLOT_DATA  = (int16_t)0xAA55;
static const int16_t I2S_TAG_INVALID      = (int16_t)0x0000;
// 1 MP3 フレーム期間 = 1152 PCM samples @ 44.1kHz I2S
static const size_t UZC_MP3_PERIOD_STEREO_441 = 1152;

struct CatalogEntry {
  char path[128];
  char name[96];
  uint64_t size;
  bool isDir;
};

struct WavInfo {
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint16_t bitsPerSample;
  uint32_t dataOffset;
  uint32_t dataSize;
};

struct UzcHeader {
  uint16_t version;
  uint16_t codecType;
  uint16_t channelCount;
  uint32_t sampleRate;
  uint32_t bitRatePerChannel;
  uint16_t maxFrameSize;
  uint16_t frameDurationMs;
  uint32_t totalFrameCount;
  uint32_t headerSize;
  uint16_t slotSize;
  uint32_t blockSize;
};

enum PlayState : uint8_t {
  PLAY_STOPPED = 0,
  PLAY_PLAYING,
  PLAY_PAUSED,
};

enum PlayUzcRoute : uint8_t {
  PLAY_ROUTE_WAV = 0,
  PLAY_ROUTE_UZC_MASTER,
  PLAY_ROUTE_UZC_SLAVE,
};

static I2SClass g_i2s;
static CatalogEntry g_catalog[CATALOG_MAX];
static size_t g_catalogCount = 0;

static char lineBuf[CLI_LINE_MAX];
static size_t lineLen = 0;

static bool g_sdOk = false;
static bool g_i2sOk = false;
static uint8_t g_fileAccessDepth = 0;

static volatile PlayState g_playState = PLAY_STOPPED;
static volatile bool g_playStopReq = false;
static volatile bool g_playPaused = false;
static TaskHandle_t g_playTask = nullptr;
static char g_playPath[128] = "";
static char g_playLabel[96] = "";
static bool g_playIsUzc = false;
static PlayUzcRoute g_playUzcRoute = PLAY_ROUTE_WAV;
static volatile uint32_t g_uzcFrameIndex = 0;
static uint16_t g_uzc_tx_slot_size = 0;
static uint16_t g_uzc_tx_max_frame = 0;
static uint16_t g_uzc_tx_frame_ms = 26;

static mp3dec_t g_mp3dec;
static mp3d_sample_t g_mp3_pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
static int16_t g_pcm_stereo_out[1152 * 2];
static uint8_t g_uzc_ch1_file_buf[UZC_MAX_SLOT_SIZE];
static uint8_t g_uzc_ch1_tx_buf[UZC_MAX_SLOT_SIZE];
static int16_t g_stest_ch1_words[UZC_MAX_STEREO_WORDS];
static int16_t g_stest_stereo_pad[UZC_I2S_PERIOD_STEREO_MAX * 2];
static int16_t g_pcm_chunk[PLAY_READ_BYTES / 2];

static void ledSet(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static void i2sWriteTaggedFrames(const int16_t* pairs, size_t stereoFrameCount) {
  if (stereoFrameCount == 0) {
    return;
  }
  g_i2s.write((const uint8_t*)pairs, stereoFrameCount * sizeof(int16_t) * 2);
}

static void i2sWriteInvalidKeepalive(size_t stereoFrameCount) {
  static int16_t inv[64 * 2];
  for (size_t i = 0; i < 64; i++) {
    inv[i * 2] = 0;
    inv[i * 2 + 1] = I2S_TAG_INVALID;
  }
  while (stereoFrameCount > 0) {
    const size_t n = (stereoFrameCount < 64) ? stereoFrameCount : 64;
    i2sWriteTaggedFrames(inv, n);
    stereoFrameCount -= n;
  }
}

static void i2sWriteRawTaggedMono(const int16_t* mono, size_t sampleCount) {
  static int16_t buf[256 * 2];
  size_t off = 0;
  for (size_t i = 0; i < sampleCount; i++) {
    buf[off++] = mono[i];
    buf[off++] = I2S_TAG_RAW;
    if (off >= sizeof(buf) / sizeof(buf[0])) {
      i2sWriteTaggedFrames(buf, off / 2);
      off = 0;
    }
  }
  if (off > 0) {
    i2sWriteTaggedFrames(buf, off / 2);
  }
}

// MP3スロット @44.1kHz: [AA00][AA55×slotWords][5500][0000×無効パッド]
static void fillMp3SlotPeriod441Tagged(const int16_t* ch1Words, size_t n1, int16_t* stereo,
                                       size_t periodStereoFrames, size_t slotStereoFrames) {
  size_t f = 0;
  stereo[f * 2] = 0;
  stereo[f * 2 + 1] = I2S_TAG_MP3_START;
  f++;
  for (size_t i = 0; i < slotStereoFrames && i < n1; i++) {
    stereo[f * 2] = ch1Words[i];
    stereo[f * 2 + 1] = I2S_TAG_SLOT_DATA;
    f++;
  }
  stereo[f * 2] = 0;
  stereo[f * 2 + 1] = I2S_TAG_MP3_END;
  f++;
  for (; f < periodStereoFrames; f++) {
    stereo[f * 2] = 0;
    stereo[f * 2 + 1] = I2S_TAG_INVALID;
  }
}

static void fileAccessBegin() {
  if (g_fileAccessDepth++ == 0) {
    ledSet(true);
  }
}

static void fileAccessEnd() {
  if (g_fileAccessDepth > 0 && --g_fileAccessDepth == 0) {
    ledSet(false);
  }
}

static void showPrompt() {
  Serial.print(PROMPT);
}

static bool sdCardDetected() {
  return digitalRead(PIN_SD_CD) == LOW;
}

static bool initSd() {
  fileAccessBegin();
  pinMode(PIN_SD_CD, INPUT_PULLUP);
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!sdCardDetected()) {
    Serial.println(F("SD: card detect = not inserted"));
  }
  bool ok = SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ);
  g_sdOk = ok;
  fileAccessEnd();
  return ok;
}

static bool ensureSd() {
  if (g_sdOk) {
    return true;
  }
  Serial.println(F("SD: retry init..."));
  return initSd();
}

static bool initI2s() {
  if (g_i2sOk) {
    return true;
  }
  g_i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT);
  if (!g_i2s.begin(I2S_MODE_STD, I2S_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.print(F("I2S init failed, err="));
    Serial.println(g_i2s.lastError());
    return false;
  }
  g_i2sOk = true;
  return true;
}

static bool setI2sTxRate(uint32_t rate) {
  if (!g_i2sOk && !initI2s()) {
    return false;
  }
  if (!g_i2s.configureTX(rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.print(F("I2S rate change failed, err="));
    Serial.println(g_i2s.lastError());
    return false;
  }
  return true;
}

static const char* baseName(const char* path) {
  if (path == nullptr) {
    return "";
  }
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

static void copyCatalogPath(char* dest, size_t destSize, const char* src) {
  if (destSize == 0) {
    return;
  }
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

static bool catalogAdd(const char* path, const char* displayName, uint64_t size, bool isDir) {
  if (g_catalogCount >= CATALOG_MAX) {
    return false;
  }
  CatalogEntry* e = &g_catalog[g_catalogCount];
  copyCatalogPath(e->path, sizeof(e->path), path);
  copyCatalogPath(e->name, sizeof(e->name), displayName);
  e->size = size;
  e->isDir = isDir;
  g_catalogCount++;
  return true;
}

static void printHumanSize(uint64_t bytes) {
  if (bytes < 1024) {
    Serial.print(bytes);
    Serial.print(F(" B"));
  } else if (bytes < 1024ULL * 1024) {
    Serial.print(bytes / 1024);
    Serial.print(F(" kB"));
  } else {
    uint64_t mb10 = (bytes * 10 + (512ULL * 1024)) / (1024ULL * 1024);
    Serial.print(mb10 / 10);
    Serial.print('.');
    Serial.print(mb10 % 10);
    Serial.print(F(" MB"));
  }
}

static void printCatalogLine(size_t index) {
  const CatalogEntry* e = &g_catalog[index];
  Serial.print(index + 1);
  Serial.print(F("  "));
  Serial.print(e->name);
  Serial.print(F("  "));
  if (e->isDir) {
    Serial.println(F("DIR"));
  } else {
    printHumanSize(e->size);
    Serial.println();
  }
}

static void collectDirRecursive(const char* dirname) {
  File root = SD.open(dirname);
  if (!root || !root.isDirectory()) {
    if (root) {
      root.close();
    }
    return;
  }

  File entry = root.openNextFile();
  while (entry) {
    const char* path = entry.path();
    const char* name = baseName(path);
    if (name[0] == '\0') {
      name = path;
    }

    if (entry.isDirectory()) {
      if (!catalogAdd(path, name, 0, true)) {
        entry.close();
        root.close();
        return;
      }
      collectDirRecursive(path);
    } else {
      if (!catalogAdd(path, name, entry.size(), false)) {
        entry.close();
        root.close();
        return;
      }
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
}

static bool g_typeLastWasLf = false;

static void serialWriteText(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    const uint8_t c = buf[i];
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (!g_typeLastWasLf) {
        Serial.write('\n');
        g_typeLastWasLf = true;
      }
      continue;
    }
    g_typeLastWasLf = false;
    Serial.write(c);
  }
}

static File openSdFile(const char* path) {
  if (path == nullptr || *path == '\0') {
    return File();
  }
  File f = SD.open(path, FILE_READ);
  if (f) {
    return f;
  }
  if (path[0] != '/') {
    char withSlash[CLI_LINE_MAX];
    snprintf(withSlash, sizeof(withSlash), "/%s", path);
    return SD.open(withSlash, FILE_READ);
  }
  return File();
}

static bool isWavPath(const char* path) {
  const char* ext = strrchr(path, '.');
  if (ext == nullptr) {
    return false;
  }
  return strcasecmp(ext, ".wav") == 0;
}

static bool isUzcPath(const char* path) {
  const char* ext = strrchr(path, '.');
  if (ext == nullptr) {
    return false;
  }
  return strcasecmp(ext, ".uzc") == 0 || strcasecmp(ext, ".uzu") == 0;
}

static void printUzcCodecName(uint16_t codecType) {
  switch (codecType) {
    case 0:
      Serial.println(F("MP3 (0)"));
      break;
    case 1:
      Serial.println(F("AAC (1)"));
      break;
    case 2:
      Serial.println(F("Opus (2)"));
      break;
    case 3:
      Serial.println(F("LC3 (3)"));
      break;
    case 4:
      Serial.println(F("ADPCM (4)"));
      break;
    default:
      Serial.print(F("unknown ("));
      Serial.print(codecType);
      Serial.println(F(")"));
      break;
  }
}

static void printUzcHeaderInfo(File& f, uint64_t fileSize) {
  UzcHeader hdr;
  if (!parseUzcHeader(f, hdr)) {
    Serial.println(F("error: invalid UZUC header"));
    return;
  }

  const uint16_t txSlotSize = uzcTransmitSlotSize(hdr);
  const uint16_t legacySlot = (uint16_t)(UZC_REAL_FRAME_SIZE_FIELD + hdr.maxFrameSize);
  const uint64_t payloadBytes = (uint64_t)hdr.blockSize * hdr.totalFrameCount;
  const uint64_t expectedSize = (uint64_t)hdr.headerSize + payloadBytes;

  Serial.println(F("--- UZUC Header ---"));
  Serial.println(F("Magic:              UZUC"));
  Serial.print(F("Version:            "));
  Serial.println(hdr.version);
  Serial.print(F("CodecType:          "));
  printUzcCodecName(hdr.codecType);
  Serial.print(F("ChannelCount:       "));
  Serial.println(hdr.channelCount);
  Serial.print(F("SampleRate:         "));
  Serial.print(hdr.sampleRate);
  Serial.println(F(" Hz"));
  Serial.print(F("BitRatePerChannel:  "));
  Serial.print(hdr.bitRatePerChannel);
  Serial.println(F(" bps"));
  Serial.print(F("MaxFrameSize:       "));
  Serial.println(hdr.maxFrameSize);
  Serial.print(F("FrameDurationMs:    "));
  Serial.println(hdr.frameDurationMs);
  Serial.print(F("TotalFrameCount:    "));
  Serial.println(hdr.totalFrameCount);
  Serial.print(F("HeaderSize:         "));
  Serial.println(hdr.headerSize);
  Serial.print(F("SlotSize:           "));
  Serial.println(hdr.slotSize);
  Serial.println(F("--- derived ---"));
  Serial.print(F("BlockSize:          "));
  Serial.println(hdr.blockSize);
  Serial.print(F("TxSlotSize:         "));
  Serial.println(txSlotSize);
  Serial.print(F("I2sSplayRate:       "));
  Serial.print(I2S_SAMPLE_RATE);
  Serial.println(F(" Hz (fixed, R-tag bus)"));
  Serial.print(F("PayloadBytes:       "));
  Serial.println((unsigned long)payloadBytes);
  Serial.print(F("ExpectedFileSize:   "));
  Serial.println((unsigned long)expectedSize);
  Serial.print(F("ActualFileSize:     "));
  Serial.println((unsigned long)fileSize);
  if (hdr.slotSize != txSlotSize && hdr.slotSize != legacySlot) {
    Serial.print(F("warn: SlotSize mismatch (expected "));
    Serial.print(txSlotSize);
    Serial.print(F(" or legacy "));
    Serial.print(legacySlot);
    Serial.print(F(", got "));
    Serial.println(hdr.slotSize);
  }
  if (expectedSize > fileSize) {
    Serial.println(F("warn: file shorter than header implies"));
  } else if (expectedSize < fileSize) {
    Serial.print(F("note: "));
    Serial.print((unsigned long)(fileSize - expectedSize));
    Serial.println(F(" trailing bytes after payload"));
  }
}

static void printWavHeaderInfo(File& f, uint64_t fileSize) {
  WavInfo info;
  if (!parseWavHeader(f, info)) {
    Serial.println(F("error: invalid WAV header"));
    return;
  }

  Serial.println(F("--- WAV Header ---"));
  Serial.print(F("AudioFormat:        "));
  Serial.println(info.audioFormat == 1 ? F("PCM (1)") : F("non-PCM"));
  Serial.print(F("NumChannels:        "));
  Serial.println(info.numChannels);
  Serial.print(F("SampleRate:         "));
  Serial.print(info.sampleRate);
  Serial.println(F(" Hz"));
  Serial.print(F("BitsPerSample:      "));
  Serial.println(info.bitsPerSample);
  Serial.print(F("DataOffset:         "));
  Serial.println(info.dataOffset);
  Serial.print(F("DataSize:           "));
  Serial.println(info.dataSize);
  Serial.print(F("FileSize:           "));
  Serial.println((unsigned long)fileSize);
  if (info.sampleRate > 0 && info.numChannels > 0 && info.bitsPerSample > 0) {
    const uint32_t bytesPerSec =
        info.sampleRate * info.numChannels * ((uint32_t)info.bitsPerSample / 8U);
    if (bytesPerSec > 0) {
      Serial.print(F("DurationSec:        "));
      Serial.println((double)info.dataSize / (double)bytesPerSec, 3);
    }
  }
}

static bool readU32(File& f, uint32_t& v) {
  return f.read((uint8_t*)&v, 4) == 4;
}

static bool readU16(File& f, uint16_t& v) {
  return f.read((uint8_t*)&v, 2) == 2;
}

static bool parseUzcHeader(File& f, UzcHeader& hdr) {
  memset(&hdr, 0, sizeof(hdr));
  f.seek(0);

  char magic[4];
  if (f.read((uint8_t*)magic, 4) != 4 || memcmp(magic, "UZUC", 4) != 0) {
    return false;
  }
  if (!readU16(f, hdr.version) || !readU16(f, hdr.codecType) || !readU16(f, hdr.channelCount) ||
      !readU32(f, hdr.sampleRate) || !readU32(f, hdr.bitRatePerChannel) || !readU16(f, hdr.maxFrameSize) ||
      !readU16(f, hdr.frameDurationMs) || !readU32(f, hdr.totalFrameCount)) {
    return false;
  }

  f.seek(0x1A);
  if (!readU32(f, hdr.headerSize)) {
    return false;
  }
  f.seek(0x1E);
  if (!readU16(f, hdr.slotSize)) {
    return false;
  }

  if (hdr.headerSize == 0) {
    hdr.headerSize = UZC_DEFAULT_HEADER_SIZE;
  }
  if (hdr.slotSize == 0) {
    hdr.slotSize = (uint16_t)(UZC_SLOT_META_BYTES + hdr.maxFrameSize);
  }
  hdr.blockSize = (uint32_t)hdr.slotSize * hdr.channelCount;
  return true;
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

static uint16_t uzcReadU16Le(const uint8_t* p) {
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void uzcWriteU16Le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static bool uzcMp3SyncAt(const uint8_t* slot, size_t off) {
  return (slot[off] == 0xFF && ((slot[off + 1] & 0xE0) == 0xE0));
}

static bool uzcSlotIsV2Format(const uint8_t* slot, uint16_t hdrMaxFrameSize) {
  const uint16_t maxField = uzcReadU16Le(slot);
  const uint16_t realField = uzcReadU16Le(slot + UZC_REAL_FRAME_SIZE_FIELD);
  if (maxField != hdrMaxFrameSize || realField < 4 || realField > maxField) {
    return false;
  }
  return uzcMp3SyncAt(slot, UZC_FRAME_DATA_OFFSET);
}

static bool uzcSlotIsV1Format(const uint8_t* slot, uint16_t hdrMaxFrameSize) {
  const uint16_t realField = uzcReadU16Le(slot);
  if (realField < 4 || realField > hdrMaxFrameSize) {
    return false;
  }
  return uzcMp3SyncAt(slot, UZC_REAL_FRAME_SIZE_FIELD);
}

static bool uzcSlotLooksLegacyMp3AtStart(const uint8_t* slot) {
  return uzcMp3SyncAt(slot, 0);
}

static uint16_t uzcTransmitSlotSize(const UzcHeader& hdr) {
  return (uint16_t)(UZC_SLOT_META_BYTES + hdr.maxFrameSize);
}

static bool uzcPrepareSlotForTransmit(const uint8_t* fileSlot, uint16_t fileSlotSize, uint16_t hdrMaxFrameSize,
                                      uint8_t* txSlot, uint16_t txSlotSize) {
  if (txSlotSize != (uint16_t)(UZC_SLOT_META_BYTES + hdrMaxFrameSize)) {
    return false;
  }
  memset(txSlot, 0, txSlotSize);

  if (fileSlotSize == txSlotSize && uzcSlotIsV2Format(fileSlot, hdrMaxFrameSize)) {
    memcpy(txSlot, fileSlot, txSlotSize);
    return true;
  }

  if (fileSlotSize == (uint16_t)(UZC_REAL_FRAME_SIZE_FIELD + hdrMaxFrameSize) &&
      uzcSlotIsV1Format(fileSlot, hdrMaxFrameSize)) {
    const uint16_t realField = uzcReadU16Le(fileSlot);
    uzcWriteU16Le(txSlot, hdrMaxFrameSize);
    uzcWriteU16Le(txSlot + UZC_REAL_FRAME_SIZE_FIELD, realField);
    memcpy(txSlot + UZC_FRAME_DATA_OFFSET, fileSlot + UZC_REAL_FRAME_SIZE_FIELD, hdrMaxFrameSize);
    return true;
  }

  if (uzcSlotLooksLegacyMp3AtStart(fileSlot)) {
    const uint16_t frameBytes = uzcMp3FrameByteLength(fileSlot);
    if (frameBytes < 4 || frameBytes > hdrMaxFrameSize) {
      return false;
    }
    uzcWriteU16Le(txSlot, hdrMaxFrameSize);
    uzcWriteU16Le(txSlot + UZC_REAL_FRAME_SIZE_FIELD, frameBytes);
    memcpy(txSlot + UZC_FRAME_DATA_OFFSET, fileSlot, frameBytes);
    return true;
  }

  return false;
}

static size_t uzcWordsPerSlot(uint16_t slotSize) {
  return ((size_t)slotSize + 1U) / 2U;
}

static bool validateUzcForPlay(File& f, UzcHeader& hdr) {
  if (!parseUzcHeader(f, hdr)) {
    Serial.println(F("error: invalid UZC file"));
    return false;
  }
  if (hdr.codecType != UZC_CODEC_MP3) {
    Serial.println(F("error: only MP3 UZC supported"));
    return false;
  }
  if (hdr.channelCount < 1 || hdr.channelCount > 8) {
    Serial.println(F("error: channel count must be 1..8"));
    return false;
  }
  if (hdr.sampleRate != I2S_SAMPLE_RATE) {
    Serial.print(F("error: sample rate "));
    Serial.print(hdr.sampleRate);
    Serial.println(F(" Hz (require 44100 Hz)"));
    return false;
  }
  if (hdr.maxFrameSize == 0 || hdr.maxFrameSize > 4096) {
    Serial.println(F("error: invalid MaxFrameSize"));
    return false;
  }
  const uint16_t expectedSlot = (uint16_t)(UZC_SLOT_META_BYTES + hdr.maxFrameSize);
  const uint16_t legacySlot = (uint16_t)(UZC_REAL_FRAME_SIZE_FIELD + hdr.maxFrameSize);
  if (hdr.slotSize != expectedSlot && hdr.slotSize != legacySlot) {
    Serial.print(F("warn: SlotSize "));
    Serial.print(hdr.slotSize);
    Serial.print(F(" (expected "));
    Serial.print(expectedSlot);
    Serial.print(F(" or legacy "));
    Serial.print(legacySlot);
    Serial.println(F(")"));
  }
  if (hdr.slotSize < legacySlot) {
    Serial.println(F("error: SlotSize too small"));
    return false;
  }
  if (hdr.slotSize > UZC_MAX_SLOT_SIZE) {
    Serial.print(F("error: slot size too large (max "));
    Serial.print(UZC_MAX_SLOT_SIZE);
    Serial.println(F(")"));
    return false;
  }
  if (uzcWordsPerSlot(hdr.slotSize) > UZC_MAX_STEREO_WORDS) {
    Serial.println(F("error: slot size exceeds I2S buffer"));
    return false;
  }
  if (hdr.frameDurationMs == 0) {
    hdr.frameDurationMs = 26;
  }
  if (hdr.totalFrameCount == 0) {
    Serial.println(F("error: no frames"));
    return false;
  }
  const uint64_t needSize = (uint64_t)hdr.headerSize + (uint64_t)hdr.blockSize * hdr.totalFrameCount;
  if (needSize > (uint64_t)f.size()) {
    Serial.println(F("error: UZC data truncated"));
    return false;
  }
  return true;
}

static size_t bytesToInt16Words(const uint8_t* bytes, size_t nbytes, int16_t* out, size_t outCap) {
  const size_t words = (nbytes + 1) / 2;
  const size_t n = words < outCap ? words : outCap;
  for (size_t i = 0; i < n; i++) {
    const uint8_t lo = bytes[i * 2];
    const uint8_t hi = (i * 2 + 1 < nbytes) ? bytes[i * 2 + 1] : 0;
    out[i] = (int16_t)(lo | ((uint16_t)hi << 8));
  }
  return n;
}

static uint32_t calcUzcFixedI2sRate(uint16_t slotSize, uint16_t frameDurationMs) {
  if (frameDurationMs == 0) {
    frameDurationMs = 26;
  }
  const size_t stereoFrames = uzcWordsPerSlot(slotSize);
  uint32_t rate = (uint32_t)((stereoFrames * 1000UL) / frameDurationMs);
  if (rate < 4000) {
    rate = 4000;
  }
  if (rate > 192000) {
    rate = 192000;
  }
  return rate;
}

// UZC 圧縮ビットレートに合わせた I2S ステレオ・サンプルレート [Hz]。
// 1ch スロットバイト列を L=R 複製して 16bit ステレオで送るため bitRate*2/16 = bitRate/8。
static uint32_t calcUzcI2sTxRate(const UzcHeader& hdr, uint16_t txSlotSize) {
  const uint16_t frameMs = (hdr.frameDurationMs != 0) ? hdr.frameDurationMs : 26;
  if (hdr.bitRatePerChannel >= 8000) {
    const uint32_t rate = hdr.bitRatePerChannel / 8U;
    if (rate >= 4000 && rate <= 192000) {
      return rate;
    }
  }
  return calcUzcFixedI2sRate(txSlotSize, frameMs);
}

// 1 MP3 スロット期間のステレオフレーム数。
// L に slotWords を載せるとき、平均バイトレートが bitRate/8 と一致するよう
// period = slotWords * 16 * I2Srate / bitRate （仕様 11.1: 1152/44100 ≒ 26.122ms と整合）
static uint32_t calcUzcPeriodStereoFrames(const UzcHeader& hdr, size_t slotStereoFrames, uint32_t uzcI2sRate) {
  if (hdr.bitRatePerChannel >= 8000) {
    const uint64_t num = (uint64_t)slotStereoFrames * 16ULL * (uint64_t)uzcI2sRate;
    const uint32_t period = (uint32_t)(num / (uint64_t)hdr.bitRatePerChannel);
    if (period > 0) {
      return period;
    }
  }
  if (hdr.sampleRate > 0) {
    return (uint32_t)(((uint64_t)uzcI2sRate * 1152ULL + hdr.sampleRate - 1ULL) / (uint64_t)hdr.sampleRate);
  }
  const uint16_t frameMs = (hdr.frameDurationMs != 0) ? hdr.frameDurationMs : 26;
  return (uint32_t)(((uint64_t)uzcI2sRate * frameMs) / 1000ULL);
}

static uint32_t g_uzcPaceNextWriteMs = 0;

static void uzcPaceReset() {
  g_uzcPaceNextWriteMs = 0;
}

static void uzcPaceWaitBeforeWrite(uint32_t rate, size_t periodFrames) {
  if (periodFrames == 0 || rate == 0) {
    return;
  }
  const uint32_t now = millis();
  if (g_uzcPaceNextWriteMs != 0 && now < g_uzcPaceNextWriteMs) {
    vTaskDelay(pdMS_TO_TICKS(g_uzcPaceNextWriteMs - now));
  }
}

static void uzcPaceMarkAfterWrite(uint32_t rate, size_t periodFrames) {
  if (periodFrames == 0 || rate == 0) {
    return;
  }
  const uint32_t periodMs = (uint32_t)(((uint64_t)periodFrames * 1000ULL + rate - 1ULL) / rate);
  const uint32_t now = millis();
  if (g_uzcPaceNextWriteMs == 0) {
    g_uzcPaceNextWriteMs = now + periodMs;
    return;
  }
  g_uzcPaceNextWriteMs += periodMs;
  if (g_uzcPaceNextWriteMs < now) {
    g_uzcPaceNextWriteMs = now + periodMs;
  }
}

static bool readUzcChannelSlotFull(File& f, uint16_t slotSize, uint8_t* slotBuf) {
  return f.read(slotBuf, slotSize) == slotSize;
}

static bool readTag(File& f, char tag[4]) {
  return f.read((uint8_t*)tag, 4) == 4;
}

static bool parseWavHeader(File& f, WavInfo& info) {
  memset(&info, 0, sizeof(info));
  f.seek(0);

  char tag[4];
  uint32_t chunkSize = 0;
  if (!readTag(f, tag) || memcmp(tag, "RIFF", 4) != 0) {
    return false;
  }
  f.read((uint8_t*)&chunkSize, 4);
  if (!readTag(f, tag) || memcmp(tag, "WAVE", 4) != 0) {
    return false;
  }

  uint32_t pos = 12;
  const uint32_t fileSize = (uint32_t)f.size();
  while (pos + 8 <= fileSize) {
    f.seek(pos);
    if (!readTag(f, tag)) {
      return false;
    }
    f.read((uint8_t*)&chunkSize, 4);

    if (memcmp(tag, "fmt ", 4) == 0 && chunkSize >= 16) {
      f.read((uint8_t*)&info.audioFormat, 2);
      f.read((uint8_t*)&info.numChannels, 2);
      f.read((uint8_t*)&info.sampleRate, 4);
      uint32_t byteRate = 0;
      uint16_t blockAlign = 0;
      f.read((uint8_t*)&byteRate, 4);
      f.read((uint8_t*)&blockAlign, 2);
      f.read((uint8_t*)&info.bitsPerSample, 2);
    } else if (memcmp(tag, "data", 4) == 0) {
      info.dataOffset = pos + 8;
      info.dataSize = chunkSize;
    }

    pos += 8 + chunkSize + (chunkSize & 1U);
  }

  return info.dataOffset > 0 && info.sampleRate > 0;
}

static bool validateWavForPlay(File& f, WavInfo& info) {
  if (!parseWavHeader(f, info)) {
    Serial.println(F("error: invalid WAV file"));
    return false;
  }
  if (info.audioFormat != 1) {
    Serial.println(F("error: only PCM WAV supported"));
    return false;
  }
  if (info.bitsPerSample != 16) {
    Serial.print(F("error: bits per sample "));
    Serial.print(info.bitsPerSample);
    Serial.println(F(" (require 16)"));
    return false;
  }
  if (info.numChannels < 1 || info.numChannels > 2) {
    Serial.println(F("error: channel count must be 1 or 2"));
    return false;
  }
  if (info.sampleRate != I2S_SAMPLE_RATE) {
    Serial.print(F("error: sample rate "));
    Serial.print(info.sampleRate);
    Serial.println(F(" Hz (require 44100 Hz)"));
    return false;
  }
  return true;
}

static void playbackStopAndWait(bool restoreWavMode = true) {
  (void)restoreWavMode;
  if (g_playTask == nullptr) {
    g_playState = PLAY_STOPPED;
    g_playPaused = false;
    return;
  }
  g_playStopReq = true;
  g_playPaused = false;
  while (g_playTask != nullptr) {
    delay(1);
    yield();
  }
  g_playStopReq = false;
  if (g_i2sOk) {
    setI2sTxRate(I2S_SAMPLE_RATE);
  }
}

static void i2sWritePcm(const uint8_t* data, size_t nbytes, uint16_t channels) {
  if (nbytes == 0) {
    return;
  }
  const size_t frames = (channels == 2) ? (nbytes / 4) : (nbytes / 2);
  const int16_t* in = (const int16_t*)data;
  static int16_t tagged[256 * 2];
  size_t outPairs = 0;
  for (size_t i = 0; i < frames; i++) {
    tagged[outPairs++] = (channels == 2) ? in[i * 2] : in[i];
    tagged[outPairs++] = I2S_TAG_RAW;
    if (outPairs >= sizeof(tagged) / sizeof(tagged[0])) {
      i2sWriteTaggedFrames(tagged, outPairs / 2);
      outPairs = 0;
    }
  }
  if (outPairs > 0) {
    i2sWriteTaggedFrames(tagged, outPairs / 2);
  }
}

static bool playWavStream(File& f, const WavInfo& wav) {
  if (!setI2sTxRate(I2S_SAMPLE_RATE)) {
    return false;
  }
  f.seek(wav.dataOffset);
  uint8_t buf[PLAY_READ_BYTES];
  while (!g_playStopReq) {
    if (g_playPaused) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    const size_t avail = f.available();
    if (avail == 0) {
      return true;
    }
    size_t toRead = avail;
    if (toRead > PLAY_READ_BYTES) {
      toRead = PLAY_READ_BYTES;
    }
    if ((toRead & 1U) != 0) {
      toRead--;
    }
    if (toRead == 0) {
      return true;
    }
    const size_t n = f.read(buf, toRead);
    if (n == 0) {
      return true;
    }
    if (!setI2sTxRate(I2S_SAMPLE_RATE)) {
      return false;
    }
    i2sWritePcm(buf, n, wav.numChannels);
  }
  return false;
}

static uint16_t uzcEffectiveMp3ByteLength(const uint8_t* mp3Data, uint16_t realField) {
  if (realField < 4) {
    return 0;
  }
  const uint16_t hdrLen = uzcMp3FrameByteLength(mp3Data);
  if (hdrLen == 0) {
    return realField;
  }
  if (hdrLen <= realField) {
    return hdrLen;
  }
  return realField;
}

static bool uzcSlotMp3PayloadAt(const uint8_t* slot, uint16_t hdrMaxFrame, size_t realOff, size_t dataOff,
                                const uint8_t** mp3Out, uint16_t* mp3LenOut) {
  const uint16_t realField = uzcReadU16Le(slot + realOff);
  if (realField < 4 || realField > hdrMaxFrame) {
    return false;
  }
  if (!uzcMp3SyncAt(slot, dataOff)) {
    return false;
  }
  const uint8_t* mp3Data = slot + dataOff;
  const uint16_t mp3Len = uzcEffectiveMp3ByteLength(mp3Data, realField);
  if (mp3Len < 4) {
    return false;
  }
  *mp3Out = mp3Data;
  *mp3LenOut = mp3Len;
  return true;
}

static bool uzcFileSlotMp3Payload(const uint8_t* fileSlot, uint16_t fileSlotSize, uint16_t hdrMaxFrame,
                                  const uint8_t** mp3Out, uint16_t* mp3LenOut) {
  if (fileSlotSize >= UZC_FRAME_DATA_OFFSET + 4 && uzcSlotIsV2Format(fileSlot, hdrMaxFrame)) {
    return uzcSlotMp3PayloadAt(fileSlot, hdrMaxFrame, UZC_REAL_FRAME_SIZE_FIELD, UZC_FRAME_DATA_OFFSET, mp3Out,
                               mp3LenOut);
  }
  if (fileSlotSize >= 6 && uzcSlotIsV1Format(fileSlot, hdrMaxFrame)) {
    return uzcSlotMp3PayloadAt(fileSlot, hdrMaxFrame, 0, UZC_REAL_FRAME_SIZE_FIELD, mp3Out, mp3LenOut);
  }
  return false;
}

// mplay: SD スロットから MP3 を直接解決（splay 用 tx 変換は不要）
static bool uzcMplayResolveSlotMp3(const uint8_t* fileSlot, uint16_t fileSlotSize, uint16_t hdrMaxFrame,
                                   const uint8_t** mp3Out, uint16_t* mp3LenOut) {
  if (uzcFileSlotMp3Payload(fileSlot, fileSlotSize, hdrMaxFrame, mp3Out, mp3LenOut)) {
    return true;
  }
  if (uzcSlotLooksLegacyMp3AtStart(fileSlot)) {
    const uint16_t frameBytes = uzcMp3FrameByteLength(fileSlot);
    if (frameBytes >= 4 && frameBytes <= hdrMaxFrame) {
      *mp3Out = fileSlot;
      *mp3LenOut = frameBytes;
      return true;
    }
  }
  const size_t scanMax = (fileSlotSize < 64U) ? fileSlotSize : 64U;
  for (size_t off = 0; off + 4U <= scanMax; off++) {
    if (!uzcMp3SyncAt(fileSlot, off)) {
      continue;
    }
    const uint16_t avail = (uint16_t)(fileSlotSize - off);
    const uint16_t cap = (avail > hdrMaxFrame) ? hdrMaxFrame : avail;
    const uint16_t mp3Len = uzcEffectiveMp3ByteLength(fileSlot + off, cap);
    if (mp3Len >= 4) {
      *mp3Out = fileSlot + off;
      *mp3LenOut = mp3Len;
      return true;
    }
  }
  return false;
}

static bool uzcTxSlotMp3Payload(const uint8_t* txSlot, uint16_t hdrMaxFrame, const uint8_t** mp3Out,
                                uint16_t* mp3LenOut) {
  return uzcSlotMp3PayloadAt(txSlot, hdrMaxFrame, UZC_REAL_FRAME_SIZE_FIELD, UZC_FRAME_DATA_OFFSET, mp3Out,
                             mp3LenOut);
}

static uint64_t g_pcmPaceSamplesSent = 0;
static uint32_t g_pcmPaceStartMs = 0;

static void pcmPaceReset() {
  g_pcmPaceSamplesSent = 0;
  g_pcmPaceStartMs = millis();
}

static void pcmPaceAfterWrite(size_t stereoFrames) {
  if (stereoFrames == 0) {
    return;
  }
  if (g_pcmPaceStartMs == 0) {
    g_pcmPaceStartMs = millis();
  }
  g_pcmPaceSamplesSent += (uint64_t)stereoFrames;
  const uint64_t elapsedMs = (uint64_t)(millis() - g_pcmPaceStartMs);
  const uint64_t dueSamples = (elapsedMs * (uint64_t)I2S_SAMPLE_RATE) / 1000ULL;
  if (g_pcmPaceSamplesSent > dueSamples) {
    const uint32_t waitMs =
        (uint32_t)(((g_pcmPaceSamplesSent - dueSamples) * 1000ULL) / (uint64_t)I2S_SAMPLE_RATE);
    if (waitMs > 0 && waitMs < 2000U) {
      vTaskDelay(pdMS_TO_TICKS(waitMs));
    }
  }
}

static void uzcWriteDecodedPcmStereo(const int16_t* stereo, size_t stereoFrames) {
  if (stereoFrames == 0) {
    return;
  }
  static int16_t tagged[256 * 2];
  size_t off = 0;
  for (size_t i = 0; i < stereoFrames; i++) {
    tagged[off++] = stereo[i * 2];
    tagged[off++] = I2S_TAG_RAW;
    if (off >= sizeof(tagged) / sizeof(tagged[0])) {
      i2sWriteTaggedFrames(tagged, off / 2);
      off = 0;
    }
  }
  if (off > 0) {
    i2sWriteTaggedFrames(tagged, off / 2);
  }
}

static bool playUzcDecodedPcmStream(File& f, const UzcHeader& hdr) {
  const bool duplicateMono = hdr.channelCount < 2;
  size_t pcmChunkFill = 0;

  if (!setI2sTxRate(I2S_SAMPLE_RATE)) {
    return false;
  }
  mp3dec_init(&g_mp3dec);
  pcmPaceReset();

  Serial.print(F("UZC decode->PCM I2S (R=AAAA) build "));
  Serial.print(BUILD_NUMBER);
  Serial.print(F(" @ "));
  Serial.print(I2S_SAMPLE_RATE);
  Serial.print(F(" Hz, "));
  Serial.print(hdr.channelCount);
  Serial.print(F("ch, frames="));
  Serial.println(hdr.totalFrameCount);

  uint32_t decodeOk = 0;
  uint32_t decodeFail = 0;

  for (uint32_t frameNo = g_uzcFrameIndex; frameNo < hdr.totalFrameCount; frameNo++) {
    if (g_playStopReq) {
      return false;
    }
    while (g_playPaused) {
      g_uzcFrameIndex = frameNo;
      vTaskDelay(pdMS_TO_TICKS(20));
      if (g_playStopReq) {
        return false;
      }
    }

    const uint32_t blockOff = hdr.headerSize + frameNo * hdr.blockSize;
    if (!f.seek(blockOff, SeekSet)) {
      Serial.println(F("error: UZC seek failed"));
      return false;
    }

    if (!readUzcChannelSlotFull(f, hdr.slotSize, g_uzc_ch1_file_buf)) {
      Serial.println(F("error: UZC CH1 slot read failed"));
      return false;
    }

    const uint16_t skipFromCh = duplicateMono ? 1 : 2;
    for (uint16_t ch = skipFromCh; ch < hdr.channelCount; ch++) {
      if (!f.seek(f.position() + hdr.slotSize, SeekSet)) {
        Serial.println(F("error: UZC slot skip failed"));
        return false;
      }
    }

    const uint8_t* mp3Data = nullptr;
    uint16_t mp3Len = 0;
    if (!uzcMplayResolveSlotMp3(g_uzc_ch1_file_buf, hdr.slotSize, hdr.maxFrameSize, &mp3Data, &mp3Len)) {
      Serial.print(F("error: UZC mplay slot MP3 not found frame="));
      Serial.print(frameNo);
      Serial.print(F(" b0="));
      for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", g_uzc_ch1_file_buf[i]);
      }
      Serial.println();
      return false;
    }

    if (frameNo == 0) {
      const uint16_t realField = uzcReadU16Le(g_uzc_ch1_file_buf + UZC_REAL_FRAME_SIZE_FIELD);
      const uint16_t hdrLen = uzcMp3FrameByteLength(mp3Data);
      Serial.print(F("UZC slot bytes[0..7]="));
      for (int i = 0; i < 8; i++) {
        Serial.printf(" %02X", g_uzc_ch1_file_buf[i]);
      }
      Serial.println();
      Serial.print(F("UZC MP3 realField="));
      Serial.print(realField);
      Serial.print(F(" hdrLen="));
      Serial.print(hdrLen);
      Serial.print(F(" decodeLen="));
      Serial.println(mp3Len);
    }

    mp3dec_frame_info_t info;
    memset(&info, 0, sizeof(info));
    const int samplesPerCh = mp3dec_decode_frame(&g_mp3dec, mp3Data, (int)mp3Len, g_mp3_pcm, &info);
    if (samplesPerCh <= 0) {
      decodeFail++;
      if (decodeFail <= 3) {
        Serial.print(F("warn: MP3 decode failed frame="));
        Serial.print(frameNo);
        Serial.print(F(" realSize="));
        Serial.println(mp3Len);
      }
      continue;
    }

    decodeOk++;
    const int outFrames = (samplesPerCh > 1152) ? 1152 : samplesPerCh;
    if (info.channels == 1 || duplicateMono) {
      for (int i = 0; i < outFrames; i++) {
        const int16_t s = g_mp3_pcm[i];
        g_pcm_stereo_out[i * 2 + 0] = s;
        g_pcm_stereo_out[i * 2 + 1] = s;
      }
    } else {
      for (int i = 0; i < outFrames; i++) {
        g_pcm_stereo_out[i * 2 + 0] = g_mp3_pcm[i * 2 + 0];
        g_pcm_stereo_out[i * 2 + 1] = g_mp3_pcm[i * 2 + 1];
      }
    }
    for (int i = 0; i < outFrames; i++) {
      g_pcm_chunk[pcmChunkFill++] = g_pcm_stereo_out[i * 2 + 0];
      g_pcm_chunk[pcmChunkFill++] = g_pcm_stereo_out[i * 2 + 1];
      if (pcmChunkFill >= (sizeof(g_pcm_chunk) / sizeof(g_pcm_chunk[0]))) {
        const size_t stereoFrames = pcmChunkFill / 2;
        uzcWriteDecodedPcmStereo(g_pcm_chunk, stereoFrames);
        pcmPaceAfterWrite(stereoFrames);
        pcmChunkFill = 0;
      }
    }

    if (frameNo == 0 || ((decodeOk % 200U) == 0U)) {
      Serial.print(F("MP3 decode ok count="));
      Serial.print(decodeOk);
      Serial.print(F(" samples="));
      Serial.print(outFrames);
      Serial.print(F(" ch="));
      Serial.println(info.channels);
    }

    g_uzcFrameIndex = frameNo + 1;
  }

  if (pcmChunkFill >= 2) {
    const size_t stereoFrames = pcmChunkFill / 2;
    uzcWriteDecodedPcmStereo(g_pcm_chunk, stereoFrames);
    pcmPaceAfterWrite(stereoFrames);
  }

  Serial.print(F("UZC decode done ok="));
  Serial.print(decodeOk);
  Serial.print(F(" fail="));
  Serial.println(decodeFail);
  return true;
}

static void uzcTransmitStopFooter() {
  if (g_uzc_tx_slot_size == 0) {
    return;
  }
  static int16_t pad[64 * 2];
  for (size_t i = 0; i < 64; i++) {
    pad[i * 2] = 0;
    pad[i * 2 + 1] = I2S_TAG_PAD;
  }
  i2sWriteTaggedFrames(pad, 64);
  Serial.println(F("UZC I2S tag MP3 STOP"));
  g_uzc_tx_slot_size = 0;
}

static bool playUzcStream(File& f, const UzcHeader& hdr) {
  uint8_t ch1FileBuf[UZC_MAX_SLOT_SIZE];
  uint8_t ch1TxBuf[UZC_MAX_SLOT_SIZE];
  int16_t ch1Words[UZC_MAX_STEREO_WORDS];
  int16_t stereoPad[UZC_I2S_PERIOD_STEREO_MAX * 2];
  const bool duplicateMono = hdr.channelCount < 2;
  const uint16_t txSlotSize = uzcTransmitSlotSize(hdr);
  const size_t wordsPerSlot = uzcWordsPerSlot(txSlotSize);
  const size_t slotStereoFrames = wordsPerSlot;
  const size_t periodStereoFrames = UZC_MP3_PERIOD_STEREO_441;
  const size_t taggedPayloadFrames = slotStereoFrames + 2U;
  const size_t invalidPadFrames =
      (periodStereoFrames > taggedPayloadFrames) ? (periodStereoFrames - taggedPayloadFrames) : 0;

  if (periodStereoFrames > UZC_I2S_PERIOD_STEREO_MAX) {
    Serial.println(F("error: UZC I2S period buffer too small"));
    return false;
  }
  bool ok = false;
  bool slotPrefetched = false;

  if (!setI2sTxRate(I2S_SAMPLE_RATE)) {
    return false;
  }
  Serial.print(F("UZC I2S tagged MP3 @ "));
  Serial.print(I2S_SAMPLE_RATE);
  Serial.print(F(" Hz period "));
  Serial.print(periodStereoFrames);
  Serial.print(F("f (AA00+AA55×"));
  Serial.print(slotStereoFrames);
  Serial.print(F("+5500+0000×"));
  Serial.print(invalidPadFrames);
  Serial.print(F("), txSlot="));
  Serial.print(txSlotSize);
  Serial.print(F(" bytes (file slot="));
  Serial.print(hdr.slotSize);
  Serial.println(F(")"));

  g_uzc_tx_slot_size = txSlotSize;
  g_uzc_tx_max_frame = hdr.maxFrameSize;
  g_uzc_tx_frame_ms = (uint16_t)((1152ULL * 1000ULL) / I2S_SAMPLE_RATE);

  Serial.print(F("UZC I2S tag MP3 START txSlot="));
  Serial.print(txSlotSize);
  Serial.print(F(" maxFrame="));
  Serial.print(hdr.maxFrameSize);
  Serial.print(F(" periodF="));
  Serial.println(periodStereoFrames);

  if (!duplicateMono) {
    Serial.println(F("error: tagged splay supports mono UZC only"));
    goto uzc_tx_done;
  }

  for (uint32_t frameNo = g_uzcFrameIndex; frameNo < hdr.totalFrameCount; frameNo++) {
    if (g_playStopReq) {
      goto uzc_tx_done;
    }
    while (g_playPaused) {
      g_uzcFrameIndex = frameNo;
      slotPrefetched = false;
      vTaskDelay(pdMS_TO_TICKS(20));
      if (g_playStopReq) {
        goto uzc_tx_done;
      }
    }

    if (!slotPrefetched) {
      const uint32_t blockOff = hdr.headerSize + frameNo * hdr.blockSize;
      if (!f.seek(blockOff, SeekSet)) {
        Serial.println(F("error: UZC seek failed"));
        goto uzc_tx_done;
      }
      if (!readUzcChannelSlotFull(f, hdr.slotSize, ch1FileBuf)) {
        Serial.println(F("error: UZC CH1 slot read failed"));
        goto uzc_tx_done;
      }
      if (!uzcPrepareSlotForTransmit(ch1FileBuf, hdr.slotSize, hdr.maxFrameSize, ch1TxBuf, txSlotSize)) {
        Serial.println(F("error: UZC CH1 slot format (re-create .uzc)"));
        goto uzc_tx_done;
      }
      for (uint16_t ch = 1; ch < hdr.channelCount; ch++) {
        if (!f.seek(f.position() + hdr.slotSize, SeekSet)) {
          Serial.println(F("error: UZC slot skip failed"));
          goto uzc_tx_done;
        }
      }
    }
    slotPrefetched = false;

    const size_t n1 = bytesToInt16Words(ch1TxBuf, txSlotSize, ch1Words, wordsPerSlot);
    fillMp3SlotPeriod441Tagged(ch1Words, n1, stereoPad, periodStereoFrames, slotStereoFrames);
    if (frameNo == 0) {
      Serial.print(F("UZC TX slot bytes[0..7]="));
      for (int i = 0; i < 8; i++) {
        Serial.printf(" %02X", ch1TxBuf[i]);
      }
      Serial.println();
    }

    const size_t slotPartFrames = slotStereoFrames + 2U;
    const size_t padPartFrames = periodStereoFrames - slotPartFrames;
    g_i2s.write((const uint8_t*)stereoPad, slotPartFrames * sizeof(int16_t) * 2);
    if (padPartFrames > 0) {
      g_i2s.write((const uint8_t*)(stereoPad + slotPartFrames * 2),
                  padPartFrames * sizeof(int16_t) * 2);
    }

    if (frameNo + 1 < hdr.totalFrameCount) {
      const uint32_t nextOff = hdr.headerSize + (frameNo + 1) * hdr.blockSize;
      if (!f.seek(nextOff, SeekSet)) {
        Serial.println(F("error: UZC seek failed"));
        goto uzc_tx_done;
      }
      if (!readUzcChannelSlotFull(f, hdr.slotSize, ch1FileBuf)) {
        Serial.println(F("error: UZC CH1 slot read failed"));
        goto uzc_tx_done;
      }
      if (!uzcPrepareSlotForTransmit(ch1FileBuf, hdr.slotSize, hdr.maxFrameSize, ch1TxBuf, txSlotSize)) {
        Serial.println(F("error: UZC CH1 slot format (re-create .uzc)"));
        goto uzc_tx_done;
      }
      for (uint16_t ch = 1; ch < hdr.channelCount; ch++) {
        if (!f.seek(f.position() + hdr.slotSize, SeekSet)) {
          Serial.println(F("error: UZC slot skip failed"));
          goto uzc_tx_done;
        }
      }
      slotPrefetched = true;
    }
    g_uzcFrameIndex = frameNo + 1;
  }
  ok = true;

uzc_tx_done:
  uzcTransmitStopFooter();
  return ok;
}

static void playbackTask(void* /*param*/) {
  bool finished = false;

  fileAccessBegin();

  File f = openSdFile(g_playPath);
  if (!f) {
    Serial.print(F("error: cannot open "));
    Serial.println(g_playPath);
    goto done;
  }

  if (!initI2s()) {
    f.close();
    goto done;
  }

  g_playPaused = false;
  g_playStopReq = false;
  g_playState = PLAY_PLAYING;

  Serial.print(F("playing: "));
  Serial.println(g_playLabel);

  if (g_playIsUzc) {
    UzcHeader hdr;
    if (!validateUzcForPlay(f, hdr)) {
      f.close();
      goto done;
    }
    if (g_playUzcRoute == PLAY_ROUTE_UZC_MASTER) {
      finished = playUzcDecodedPcmStream(f, hdr);
    } else {
      finished = playUzcStream(f, hdr);
      setI2sTxRate(I2S_SAMPLE_RATE);
    }
  } else {
    WavInfo wav;
    if (!validateWavForPlay(f, wav)) {
      f.close();
      goto done;
    }
    finished = playWavStream(f, wav);
  }

  if (finished && !g_playStopReq) {
    Serial.println(F("playback finished"));
  } else if (g_playStopReq) {
    Serial.println(F("playback stopped"));
  }

  f.close();

done:
  g_playState = PLAY_STOPPED;
  g_playPaused = false;
  g_playIsUzc = false;
  g_playUzcRoute = PLAY_ROUTE_WAV;
  g_uzcFrameIndex = 0;
  g_playPath[0] = '\0';
  g_playLabel[0] = '\0';
  g_playTask = nullptr;
  fileAccessEnd();
  vTaskDelete(nullptr);
}

static bool startPlaybackRoute(const char* path, const char* label, PlayUzcRoute route) {
  playbackStopAndWait(false);

  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    return false;
  }

  const bool isWav = isWavPath(path);
  const bool isUzc = isUzcPath(path);
  if (!isWav && !isUzc) {
    Serial.println(F("error: not a WAV or UZC file"));
    return false;
  }

  if (route == PLAY_ROUTE_WAV) {
    if (!isWav) {
      Serial.println(F("error: play supports WAV only (UZC: use play/splay/mplay)"));
      return false;
    }
  } else {
    if (!isUzc) {
      Serial.println(F("error: mplay/splay require UZC file"));
      return false;
    }
  }

  fileAccessBegin();
  File f = openSdFile(path);
  if (!f) {
    Serial.print(F("cannot open: "));
    Serial.println(label);
    fileAccessEnd();
    return false;
  }

  bool ok = false;
  if (isUzc) {
    UzcHeader hdr;
    ok = validateUzcForPlay(f, hdr);
  } else {
    WavInfo wav;
    ok = validateWavForPlay(f, wav);
  }
  f.close();
  fileAccessEnd();

  if (!ok) {
    return false;
  }

  g_playIsUzc = isUzc;
  g_playUzcRoute = isUzc ? route : PLAY_ROUTE_WAV;
  g_uzcFrameIndex = 0;
  copyCatalogPath(g_playPath, sizeof(g_playPath), path);
  copyCatalogPath(g_playLabel, sizeof(g_playLabel), label);

  const uint32_t taskStack = (route == PLAY_ROUTE_UZC_MASTER) ? 32768U : 16384U;
  if (xTaskCreate(playbackTask, "audioPlay", taskStack, nullptr, 2, &g_playTask) != pdPASS) {
    Serial.println(F("error: playback task create failed"));
    g_playPath[0] = '\0';
    g_playLabel[0] = '\0';
    g_playIsUzc = false;
    g_playUzcRoute = PLAY_ROUTE_WAV;
    return false;
  }
  return true;
}

static void trimInPlace(char* s) {
  if (s == nullptr || *s == '\0') {
    return;
  }
  char* start = s;
  while (*start == ' ' || *start == '\t') {
    start++;
  }
  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[--len] = '\0';
  }
}

static char* firstToken(char* line, char** rest) {
  trimInPlace(line);
  if (*line == '\0') {
    *rest = nullptr;
    return nullptr;
  }
  char* sp = strchr(line, ' ');
  if (sp != nullptr) {
    *sp = '\0';
    *rest = sp + 1;
    trimInPlace(*rest);
    if (**rest == '\0') {
      *rest = nullptr;
    }
  } else {
    *rest = nullptr;
  }
  return line;
}

static bool isAllDigits(const char* s) {
  if (s == nullptr || *s == '\0') {
    return false;
  }
  for (const char* p = s; *p != '\0'; p++) {
    if (*p < '0' || *p > '9') {
      return false;
    }
  }
  return true;
}

static bool resolveCatalogIndex(const char* arg, size_t* outIndex) {
  if (!isAllDigits(arg)) {
    return false;
  }
  if (g_catalogCount == 0) {
    Serial.println(F("catalog empty: run dir first"));
    return false;
  }
  unsigned long n = strtoul(arg, nullptr, 10);
  if (n == 0 || n > g_catalogCount) {
    Serial.print(F("invalid file number: "));
    Serial.println(arg);
    return false;
  }
  *outIndex = (size_t)(n - 1);
  return true;
}

static const char* resolveOpenPath(const char* arg, size_t* catalogIndex, bool* fromCatalog) {
  *fromCatalog = false;
  if (resolveCatalogIndex(arg, catalogIndex)) {
    *fromCatalog = true;
    return g_catalog[*catalogIndex].path;
  }
  if (isAllDigits(arg)) {
    return nullptr;
  }
  return arg;
}

static uint32_t pcmFingerprintMonoL(const int16_t* stereo, int frames) {
  uint32_t c = 2166136261u;
  for (int i = 0; i < frames; i++) {
    c ^= (uint16_t)stereo[i * 2];
    c *= 16777619u;
  }
  return c;
}

static void printPcmSamplePrefix(const char* label, const int16_t* stereo, int count) {
  Serial.print(label);
  for (int i = 0; i < count; i++) {
    Serial.print(stereo[i * 2]);
    if (i + 1 < count) {
      Serial.print(',');
    }
  }
  Serial.println();
}

static void printHexBytes(const char* label, const uint8_t* data, size_t n) {
  Serial.print(label);
  for (size_t i = 0; i < n; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

static bool stestDecodeAndPrintRef(const UzcHeader& hdr, uint32_t frameIndex, bool verboseHex) {
  const uint8_t* mp3Data = nullptr;
  uint16_t mp3Len = 0;
  if (!uzcMplayResolveSlotMp3(g_uzc_ch1_file_buf, hdr.slotSize, hdr.maxFrameSize, &mp3Data, &mp3Len)) {
    Serial.print(F("STEST ref frame="));
    Serial.print(frameIndex);
    Serial.println(F(" MP3 resolve failed"));
    return false;
  }

  mp3dec_frame_info_t info;
  memset(&info, 0, sizeof(info));
  const int samplesPerCh = mp3dec_decode_frame(&g_mp3dec, mp3Data, (int)mp3Len, g_mp3_pcm, &info);
  if (samplesPerCh <= 0) {
    Serial.print(F("STEST ref frame="));
    Serial.print(frameIndex);
    Serial.println(F(" MP3 decode failed"));
    return false;
  }

  const int outFrames = (samplesPerCh > 1152) ? 1152 : samplesPerCh;
  if (info.channels == 1 || hdr.channelCount < 2) {
    for (int i = 0; i < outFrames; i++) {
      const int16_t s = g_mp3_pcm[i];
      g_pcm_stereo_out[i * 2 + 0] = s;
      g_pcm_stereo_out[i * 2 + 1] = s;
    }
  } else {
    for (int i = 0; i < outFrames; i++) {
      g_pcm_stereo_out[i * 2 + 0] = g_mp3_pcm[i * 2 + 0];
      g_pcm_stereo_out[i * 2 + 1] = g_mp3_pcm[i * 2 + 1];
    }
  }

  Serial.print(F("STEST ref frame="));
  Serial.print(frameIndex);
  Serial.print(F(" samples="));
  Serial.print(outFrames);
  Serial.print(F(" ch="));
  Serial.print(info.channels);
  Serial.print(F(" mp3Len="));
  Serial.print(mp3Len);
  Serial.print(F(" fp=0x"));
  Serial.println(pcmFingerprintMonoL(g_pcm_stereo_out, outFrames), HEX);
  if (verboseHex) {
    printPcmSamplePrefix("STEST ref pcmL[0..7]=", g_pcm_stereo_out, 8);
    printHexBytes("STEST ref fileSlot[0..15]=", g_uzc_ch1_file_buf, 16);
    printHexBytes("STEST ref txSlot[0..15]=", g_uzc_ch1_tx_buf, 16);
  }
  return true;
}

static void cmdStest(const char* arg) {
  if (arg == nullptr || *arg == '\0') {
    Serial.println(F("usage: stest <file#> [frameNo] [sec]"));
    return;
  }
  if (g_playTask != nullptr) {
    Serial.println(F("error: stop playback first"));
    return;
  }

  char argCopy[CLI_LINE_MAX];
  strncpy(argCopy, arg, sizeof(argCopy) - 1);
  argCopy[sizeof(argCopy) - 1] = '\0';

  char* rest = nullptr;
  char* fileTok = firstToken(argCopy, &rest);
  if (fileTok == nullptr) {
    Serial.println(F("usage: stest <file#> [frameNo] [sec]"));
    return;
  }

  uint32_t frameNo = 0;
  float sendSec = 0.0f;
  if (rest != nullptr && *rest != '\0') {
    char* frameRest = nullptr;
    char* frameTok = firstToken(rest, &frameRest);
    if (frameTok != nullptr && isAllDigits(frameTok)) {
      frameNo = (uint32_t)strtoul(frameTok, nullptr, 10);
    }
    if (frameRest != nullptr && *frameRest != '\0') {
      char* secRest = nullptr;
      char* secTok = firstToken(frameRest, &secRest);
      if (secTok != nullptr) {
        sendSec = strtof(secTok, nullptr);
      }
    }
  }

  fileAccessBegin();
  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    fileAccessEnd();
    return;
  }

  size_t catalogIndex = 0;
  bool fromCatalog = false;
  const char* path = resolveOpenPath(fileTok, &catalogIndex, &fromCatalog);
  if (path == nullptr) {
    fileAccessEnd();
    return;
  }

  File f = openSdFile(path);
  if (!f) {
    Serial.println(F("error: cannot open file"));
    fileAccessEnd();
    return;
  }

  UzcHeader hdr;
  if (!validateUzcForPlay(f, hdr)) {
    f.close();
    fileAccessEnd();
    return;
  }
  if (frameNo >= hdr.totalFrameCount) {
    Serial.println(F("error: frameNo out of range"));
    f.close();
    fileAccessEnd();
    return;
  }

  const uint32_t frameMs = (hdr.frameDurationMs != 0) ? hdr.frameDurationMs : 26U;
  uint32_t frameCount = 1;
  if (sendSec > 0.0f) {
    frameCount = (uint32_t)((sendSec * 1000.0f + (float)frameMs - 1.0f) / (float)frameMs);
    if (frameCount < 1) {
      frameCount = 1;
    }
  }
  if (frameNo + frameCount > hdr.totalFrameCount) {
    frameCount = hdr.totalFrameCount - frameNo;
  }

  const uint16_t txSlotSize = uzcTransmitSlotSize(hdr);
  const size_t wordsPerSlot = uzcWordsPerSlot(txSlotSize);
  const size_t slotStereoFrames = wordsPerSlot;
  const size_t periodStereoFrames = UZC_MP3_PERIOD_STEREO_441;
  const size_t slotPartFrames = slotStereoFrames + 2U;
  const size_t padPartFrames = periodStereoFrames - slotPartFrames;

  if (!initI2s()) {
    Serial.println(F("error: I2S init failed"));
    f.close();
    fileAccessEnd();
    return;
  }
  if (!setI2sTxRate(I2S_SAMPLE_RATE)) {
    Serial.println(F("error: I2S init failed"));
    f.close();
    fileAccessEnd();
    return;
  }

  g_uzc_tx_slot_size = txSlotSize;
  g_uzc_tx_max_frame = hdr.maxFrameSize;

  Serial.print(F("STEST send frames="));
  Serial.print(frameCount);
  Serial.print(F(" from="));
  Serial.print(frameNo);
  Serial.print(F(" (~"));
  Serial.print((frameCount * frameMs) / 1000.0f, 2);
  Serial.print(F(" s @ "));
  Serial.print(frameMs);
  Serial.println(F(" ms/frame)"));
  Serial.print(F("STEST tx words="));
  Serial.print(wordsPerSlot);
  Serial.print(F(" slotFrames="));
  Serial.print(slotStereoFrames);
  Serial.print(F(" periodFrames="));
  Serial.println(periodStereoFrames);

  mp3dec_init(&g_mp3dec);

  for (uint32_t i = 0; i < frameCount; i++) {
    const uint32_t fn = frameNo + i;
    const uint32_t blockOff = hdr.headerSize + fn * hdr.blockSize;
    if (!f.seek(blockOff, SeekSet)) {
      Serial.print(F("error: UZC seek failed frame="));
      Serial.println(fn);
      break;
    }
    if (!readUzcChannelSlotFull(f, hdr.slotSize, g_uzc_ch1_file_buf)) {
      Serial.print(F("error: UZC slot read failed frame="));
      Serial.println(fn);
      break;
    }
    for (uint16_t ch = 1; ch < hdr.channelCount; ch++) {
      if (!f.seek(f.position() + hdr.slotSize, SeekSet)) {
        Serial.print(F("error: UZC slot skip failed frame="));
        Serial.println(fn);
        f.close();
        fileAccessEnd();
        return;
      }
    }

    const bool verboseRef = (i == 0 || i + 1 == frameCount || (i % 50U) == 0U);
    if (!uzcPrepareSlotForTransmit(g_uzc_ch1_file_buf, hdr.slotSize, hdr.maxFrameSize, g_uzc_ch1_tx_buf,
                                   txSlotSize)) {
      Serial.print(F("error: tx slot prepare failed frame="));
      Serial.println(fn);
      break;
    }
    if (verboseRef) {
      stestDecodeAndPrintRef(hdr, fn, i == 0);
    }

    const size_t n1 = bytesToInt16Words(g_uzc_ch1_tx_buf, txSlotSize, g_stest_ch1_words, wordsPerSlot);
    fillMp3SlotPeriod441Tagged(g_stest_ch1_words, n1, g_stest_stereo_pad, periodStereoFrames, slotStereoFrames);
    g_i2s.write((const uint8_t*)g_stest_stereo_pad, slotPartFrames * sizeof(int16_t) * 2);
    if (padPartFrames > 0) {
      g_i2s.write((const uint8_t*)(g_stest_stereo_pad + slotPartFrames * 2),
                  padPartFrames * sizeof(int16_t) * 2);
    }
  }

  f.close();
  fileAccessEnd();
  uzcTransmitStopFooter();

  Serial.println(F("STEST done (Slave: menu 3, compare fp/hb)"));
}

static void cmdHelp() {
  Serial.print(F("SD_I2S build "));
  Serial.println(BUILD_NUMBER);
  Serial.println(F("help  - コマンド一覧と説明"));
  Serial.println(F("dir   - SDカードのファイル一覧 (番号付き)"));
  Serial.println(F("type  - ファイル表示 (type <番号>)"));
  Serial.println(F("stat  - 現在の演奏状態"));
  Serial.println(F("play  - WAV演奏 / UZCはsplay相当 (play <番号>)"));
  Serial.println(F("mplay - UZCをMainでMP3デコード→PCM (I2S R=AAAA)"));
  Serial.println(F("splay - UZCを圧縮I2S転送 (I2S Rタグ / Slaveデコード)"));
  Serial.println(F("stest - スロット送信+mplay参照 (stest <番号> [frameNo] [sec])"));
  Serial.println(F("info  - ファイル情報 (info <番号>)"));
  Serial.println(F("pause - 一時停止 / 再開"));
  Serial.println(F("stop  - 演奏を停止"));
}

static void cmdDir() {
  fileAccessBegin();
  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    fileAccessEnd();
    return;
  }

  g_catalogCount = 0;
  collectDirRecursive("/");

  if (g_catalogCount >= CATALOG_MAX) {
    Serial.println(F("warning: catalog truncated (max 64 entries)"));
  }

  for (size_t i = 0; i < g_catalogCount; i++) {
    printCatalogLine(i);
  }
  fileAccessEnd();
}

static void cmdType(const char* arg) {
  if (arg == nullptr || *arg == '\0') {
    Serial.println(F("usage: type <file#>"));
    return;
  }

  fileAccessBegin();
  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    fileAccessEnd();
    return;
  }

  size_t catalogIndex = 0;
  bool fromCatalog = false;
  const char* path = resolveOpenPath(arg, &catalogIndex, &fromCatalog);
  if (path == nullptr) {
    fileAccessEnd();
    return;
  }
  if (fromCatalog && g_catalog[catalogIndex].isDir) {
    Serial.println(F("is a directory"));
    fileAccessEnd();
    return;
  }

  File f = openSdFile(path);
  if (!f) {
    Serial.print(F("cannot open: "));
    if (fromCatalog) {
      Serial.println(g_catalog[catalogIndex].name);
    } else {
      Serial.println(path);
    }
    fileAccessEnd();
    return;
  }
  if (f.isDirectory()) {
    Serial.println(F("is a directory"));
    f.close();
    fileAccessEnd();
    return;
  }

  g_typeLastWasLf = false;
  size_t sent = 0;
  bool truncated = false;
  uint8_t buf[128];
  while (f.available() && sent < TYPE_MAX_BYTES) {
    size_t toRead = sizeof(buf);
    if (sent + toRead > TYPE_MAX_BYTES) {
      toRead = TYPE_MAX_BYTES - sent;
    }
    size_t n = f.read(buf, toRead);
    if (n == 0) {
      break;
    }
    serialWriteText(buf, n);
    sent += n;
  }
  truncated = f.available() > 0;
  f.close();

  if (truncated) {
    Serial.println();
    Serial.print(F("--- truncated at "));
    Serial.print(TYPE_MAX_BYTES);
    Serial.println(F(" bytes ---"));
  }
  fileAccessEnd();
}

static void cmdInfo(const char* arg) {
  if (arg == nullptr || *arg == '\0') {
    Serial.println(F("usage: info <file#>"));
    return;
  }

  fileAccessBegin();
  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    fileAccessEnd();
    return;
  }

  size_t catalogIndex = 0;
  bool fromCatalog = false;
  const char* path = resolveOpenPath(arg, &catalogIndex, &fromCatalog);
  if (path == nullptr) {
    fileAccessEnd();
    return;
  }
  if (fromCatalog && g_catalog[catalogIndex].isDir) {
    Serial.println(F("is a directory"));
    fileAccessEnd();
    return;
  }

  File f = openSdFile(path);
  if (!f) {
    Serial.print(F("cannot open: "));
    if (fromCatalog) {
      Serial.println(g_catalog[catalogIndex].name);
    } else {
      Serial.println(path);
    }
    fileAccessEnd();
    return;
  }
  if (f.isDirectory()) {
    Serial.println(F("is a directory"));
    f.close();
    fileAccessEnd();
    return;
  }

  const uint64_t fileSize = f.size();
  Serial.print(F("path: "));
  Serial.println(path);
  Serial.print(F("name: "));
  if (fromCatalog) {
    Serial.println(g_catalog[catalogIndex].name);
  } else {
    Serial.println(baseName(path));
  }
  Serial.print(F("size: "));
  Serial.println((unsigned long)fileSize);

  if (isUzcPath(path)) {
    printUzcHeaderInfo(f, fileSize);
  } else if (isWavPath(path)) {
    printWavHeaderInfo(f, fileSize);
  } else {
    Serial.println(F("format: unknown (not WAV/UZC)"));
  }

  f.close();
  fileAccessEnd();
}

static void cmdStat() {
  switch (g_playState) {
    case PLAY_PLAYING:
      Serial.print(F("status: playing  file: "));
      Serial.println(g_playLabel);
      break;
    case PLAY_PAUSED:
      Serial.print(F("status: paused   file: "));
      Serial.println(g_playLabel);
      break;
    default:
      Serial.println(F("status: stopped"));
      break;
  }
}

static bool cmdPlaybackPrepare(const char* arg, const char* usage) {
  if (g_playState == PLAY_PAUSED) {
    if (arg == nullptr || *arg == '\0') {
      g_playPaused = false;
      g_playState = PLAY_PLAYING;
      Serial.println(F("resumed"));
      return false;
    }
    playbackStopAndWait();
  } else if (g_playState == PLAY_PLAYING) {
    if (arg == nullptr || *arg == '\0') {
      Serial.println(F("already playing"));
      return false;
    }
    playbackStopAndWait();
  }

  if (arg == nullptr || *arg == '\0') {
    Serial.println(usage);
    return false;
  }

  if (!ensureSd()) {
    Serial.println(F("SD card not ready"));
    return false;
  }
  return true;
}

static bool cmdPlaybackResolve(const char* arg, const char** outPath, const char** outLabel) {
  size_t catalogIndex = 0;
  bool fromCatalog = false;
  const char* path = resolveOpenPath(arg, &catalogIndex, &fromCatalog);
  if (path == nullptr) {
    return false;
  }
  if (fromCatalog && g_catalog[catalogIndex].isDir) {
    Serial.println(F("is a directory"));
    return false;
  }
  *outPath = path;
  *outLabel = fromCatalog ? g_catalog[catalogIndex].name : baseName(path);
  return true;
}

static void cmdPlay(const char* arg) {
  if (!cmdPlaybackPrepare(arg, "usage: play <file#>")) {
    return;
  }

  const char* path = nullptr;
  const char* label = nullptr;
  if (!cmdPlaybackResolve(arg, &path, &label)) {
    return;
  }

  if (isUzcPath(path)) {
    startPlaybackRoute(path, label, PLAY_ROUTE_UZC_SLAVE);
    return;
  }
  if (isWavPath(path)) {
    startPlaybackRoute(path, label, PLAY_ROUTE_WAV);
    return;
  }
  Serial.println(F("error: not a WAV or UZC file"));
}

static void cmdMplay(const char* arg) {
  if (!cmdPlaybackPrepare(arg, "usage: mplay <file#>")) {
    return;
  }

  const char* path = nullptr;
  const char* label = nullptr;
  if (!cmdPlaybackResolve(arg, &path, &label)) {
    return;
  }
  startPlaybackRoute(path, label, PLAY_ROUTE_UZC_MASTER);
}

static void cmdSplay(const char* arg) {
  if (!cmdPlaybackPrepare(arg, "usage: splay <file#>")) {
    return;
  }

  const char* path = nullptr;
  const char* label = nullptr;
  if (!cmdPlaybackResolve(arg, &path, &label)) {
    return;
  }
  startPlaybackRoute(path, label, PLAY_ROUTE_UZC_SLAVE);
}

static void cmdPause() {
  if (g_playState == PLAY_PLAYING) {
    g_playPaused = true;
    g_playState = PLAY_PAUSED;
    Serial.println(F("paused"));
    return;
  }
  if (g_playState == PLAY_PAUSED) {
    g_playPaused = false;
    g_playState = PLAY_PLAYING;
    Serial.println(F("resumed"));
    return;
  }
  Serial.println(F("not playing"));
}

static void cmdStop() {
  if (g_playTask == nullptr) {
    Serial.println(F("already stopped"));
    return;
  }
  playbackStopAndWait();
}

static void processLine(char* line) {
  char* rest = nullptr;
  char* cmd = firstToken(line, &rest);
  if (cmd == nullptr) {
    return;
  }

  if (strcmp(cmd, "help") == 0) {
    cmdHelp();
  } else if (strcmp(cmd, "dir") == 0) {
    cmdDir();
  } else if (strcmp(cmd, "type") == 0) {
    cmdType(rest);
  } else if (strcmp(cmd, "stat") == 0) {
    cmdStat();
  } else if (strcmp(cmd, "play") == 0) {
    cmdPlay(rest);
  } else if (strcmp(cmd, "mplay") == 0) {
    cmdMplay(rest);
  } else if (strcmp(cmd, "splay") == 0) {
    cmdSplay(rest);
  } else if (strcmp(cmd, "stest") == 0) {
    cmdStest(rest);
  } else if (strcmp(cmd, "info") == 0) {
    cmdInfo(rest);
  } else if (strcmp(cmd, "pause") == 0) {
    cmdPause();
  } else if (strcmp(cmd, "stop") == 0) {
    cmdStop();
  } else {
    Serial.print(F("unknown command: "));
    Serial.println(cmd);
  }
}

static void handleInputChar(int c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    Serial.print('\n');
    lineBuf[lineLen] = '\0';
    processLine(lineBuf);
    lineLen = 0;
    showPrompt();
    return;
  }

  if (c == '\b' || c == 127) {
    if (lineLen > 0) {
      lineLen--;
      Serial.print(F("\b \b"));
    }
    return;
  }

  if (c >= 32 && lineLen < CLI_LINE_MAX - 1) {
    lineBuf[lineLen++] = (char)c;
    Serial.write((char)c);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  ledSet(false);

  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.print(F("SD_I2S CLI ready  build "));
  Serial.println(BUILD_NUMBER);

  if (initSd()) {
    Serial.println(F("SD card OK"));
  } else {
    Serial.println(F("SD card init failed"));
  }

  if (initI2s()) {
    Serial.println(F("I2S OK (play=WAV, mplay=PCM, splay=UZC I2S)"));
  } else {
    Serial.println(F("I2S init failed"));
  }

  showPrompt();
}

void loop() {
  while (Serial.available() > 0) {
    handleInputChar(Serial.read());
  }
}

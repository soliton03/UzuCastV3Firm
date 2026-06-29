#include "uzc_tdm.h"

#include <cstring>

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_STATIC_SCRATCH
#include "minimp3.h"

static constexpr uint32_t TDM_NUM_CH = 8;
static constexpr size_t UZC_DEFAULT_HEADER_SIZE = 32768;
static constexpr uint16_t UZC_CODEC_MP3 = 0;
static constexpr uint16_t UZC_SLOT_META_BYTES = 4;
static constexpr uint16_t UZC_REAL_FRAME_SIZE_FIELD = 2;
static constexpr uint16_t UZC_FRAME_DATA_OFFSET = 4;
static constexpr size_t UZC_MAX_SLOT_SIZE = 520;
static constexpr size_t UZC_MAX_STEREO_WORDS = 256;
static constexpr size_t UZC_TDM_PERIOD_MAX = 1280;

static const int16_t I2S_TAG_MP3_START = (int16_t)0xAA00;
static const int16_t I2S_TAG_MP3_END   = (int16_t)0x5500;
static const int16_t I2S_TAG_SLOT_DATA = (int16_t)0xAA55;
static const int16_t I2S_TAG_PAD       = (int16_t)0xFFFF;
static const int16_t I2S_TAG_INVALID   = (int16_t)0x0000;
static const size_t UZC_MP3_PERIOD_FRAMES = 1152;

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

static bool readU16(File& f, uint16_t& v) {
  uint8_t b[2];
  if (f.read(b, 2) != 2) return false;
  v = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
  return true;
}

static bool readU32(File& f, uint32_t& v) {
  uint8_t b[4];
  if (f.read(b, 4) != 4) return false;
  v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  return true;
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

static uint16_t uzcMp3FrameByteLength(const uint8_t* h) {
  if (h[0] != 0xFF || ((h[1] & 0xE0) != 0xE0)) return 0;
  const uint8_t verBits = (h[1] >> 3) & 3;
  const uint8_t layerBits = (h[1] >> 1) & 3;
  if (layerBits != 1) return 0;
  const bool mpeg1 = (verBits == 3);
  const uint8_t brIdx = (h[2] >> 4) & 0x0F;
  const uint8_t srIdx = (h[2] >> 2) & 3;
  if (brIdx == 0 || brIdx == 0x0F || srIdx == 3) return 0;
  static const uint16_t kBrKbps[] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
  static const uint32_t kSrMpeg1[] = {44100, 48000, 32000, 0};
  static const uint32_t kSrMpeg2[] = {22050, 24000, 16000, 0};
  const uint32_t bitrate = (uint32_t)kBrKbps[brIdx] * 1000UL;
  const uint32_t samplerate = mpeg1 ? kSrMpeg1[srIdx] : kSrMpeg2[srIdx];
  if (bitrate == 0 || samplerate == 0) return 0;
  const uint8_t padding = (h[2] >> 1) & 1;
  const uint32_t numerator = mpeg1 ? (144UL * bitrate) : (72UL * bitrate);
  const uint32_t frameLen = (numerator / samplerate) + padding;
  if (frameLen < 4 || frameLen > 4096) return 0;
  return (uint16_t)frameLen;
}

static bool uzcSlotIsV2Format(const uint8_t* slot, uint16_t hdrMaxFrameSize) {
  const uint16_t maxField = uzcReadU16Le(slot);
  const uint16_t realField = uzcReadU16Le(slot + UZC_REAL_FRAME_SIZE_FIELD);
  if (maxField != hdrMaxFrameSize || realField < 4 || realField > maxField) return false;
  return uzcMp3SyncAt(slot, UZC_FRAME_DATA_OFFSET);
}

static bool uzcSlotIsV1Format(const uint8_t* slot, uint16_t hdrMaxFrameSize) {
  const uint16_t realField = uzcReadU16Le(slot);
  if (realField < 4 || realField > hdrMaxFrameSize) return false;
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
  if (txSlotSize != (uint16_t)(UZC_SLOT_META_BYTES + hdrMaxFrameSize)) return false;
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
    if (frameBytes < 4 || frameBytes > hdrMaxFrameSize) return false;
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

static bool parseUzcHeader(File& f, UzcHeader& hdr) {
  memset(&hdr, 0, sizeof(hdr));
  f.seek(0);
  char magic[4];
  if (f.read((uint8_t*)magic, 4) != 4 || memcmp(magic, "UZUC", 4) != 0) return false;
  if (!readU16(f, hdr.version) || !readU16(f, hdr.codecType) || !readU16(f, hdr.channelCount) ||
      !readU32(f, hdr.sampleRate) || !readU32(f, hdr.bitRatePerChannel) || !readU16(f, hdr.maxFrameSize) ||
      !readU16(f, hdr.frameDurationMs) || !readU32(f, hdr.totalFrameCount)) {
    return false;
  }
  f.seek(0x1A);
  if (!readU32(f, hdr.headerSize)) return false;
  f.seek(0x1E);
  if (!readU16(f, hdr.slotSize)) return false;
  if (hdr.headerSize == 0) hdr.headerSize = UZC_DEFAULT_HEADER_SIZE;
  if (hdr.slotSize == 0) hdr.slotSize = (uint16_t)(UZC_SLOT_META_BYTES + hdr.maxFrameSize);
  hdr.blockSize = (uint32_t)hdr.slotSize * hdr.channelCount;
  return true;
}

static bool validateUzcForPlay(File& f, UzcHeader& hdr) {
  if (isUzuPcmFile(f)) return false;
  if (!parseUzcHeader(f, hdr)) return false;
  if (hdr.codecType != UZC_CODEC_MP3) return false;
  if (hdr.channelCount < 1 || hdr.channelCount > 7) return false;
  if (hdr.sampleRate != 44100) return false;
  if (hdr.maxFrameSize == 0 || hdr.maxFrameSize > 4096) return false;
  const uint16_t expectedSlot = (uint16_t)(UZC_SLOT_META_BYTES + hdr.maxFrameSize);
  const uint16_t legacySlot = (uint16_t)(UZC_REAL_FRAME_SIZE_FIELD + hdr.maxFrameSize);
  if (hdr.slotSize != expectedSlot && hdr.slotSize != legacySlot && hdr.slotSize != 0) {
    // tolerate minor header mismatch; tx path uses uzcTransmitSlotSize()
  }
  if (hdr.slotSize > UZC_MAX_SLOT_SIZE) return false;
  if (uzcWordsPerSlot(hdr.slotSize) > UZC_MAX_STEREO_WORDS) return false;
  if (hdr.frameDurationMs == 0) hdr.frameDurationMs = 26;
  if (hdr.totalFrameCount == 0) return false;
  const uint64_t needSize = (uint64_t)hdr.headerSize + (uint64_t)hdr.blockSize * hdr.totalFrameCount;
  return needSize <= (uint64_t)f.size();
}

bool readUzcLengthMs(const char* path, uint32_t& lengthMs) {
  lengthMs = 0;
  if (!path || !path[0]) return false;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  UzcHeader hdr;
  const bool ok = validateUzcForPlay(f, hdr);
  f.close();
  if (!ok) return false;
  lengthMs = (uint32_t)((uint64_t)hdr.totalFrameCount * hdr.frameDurationMs);
  return lengthMs > 0;
}

bool readUzcTrackInfo(const char* path, uint32_t& channels, uint32_t& sampleRate, uint32_t& lengthMs) {
  channels = 0;
  sampleRate = 0;
  lengthMs = 0;
  if (!path || !path[0]) return false;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  UzcHeader hdr;
  const bool ok = validateUzcForPlay(f, hdr);
  f.close();
  if (!ok) return false;
  channels = hdr.channelCount;
  sampleRate = hdr.sampleRate;
  lengthMs = (uint32_t)((uint64_t)hdr.totalFrameCount * hdr.frameDurationMs);
  return true;
}

static bool readUzcChannelSlotFull(File& f, uint16_t slotSize, uint8_t* slotBuf) {
  return f.read(slotBuf, slotSize) == slotSize;
}

static void fillMp3TdmPeriodTagged(const int16_t* const chWords[7], const size_t nWords[7], uint16_t numCh,
                                   int16_t* tdm, size_t periodFrames, size_t slotStereoFrames) {
  size_t f = 0;
  for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) tdm[f * TDM_NUM_CH + ch] = 0;
  tdm[f * TDM_NUM_CH + 7] = I2S_TAG_MP3_START;
  f++;

  for (size_t i = 0; i < slotStereoFrames; i++) {
    for (uint16_t ch = 0; ch < 7; ch++) {
      tdm[f * TDM_NUM_CH + ch] = (ch < numCh && i < nWords[ch]) ? chWords[ch][i] : 0;
    }
    tdm[f * TDM_NUM_CH + 7] = I2S_TAG_SLOT_DATA;
    f++;
  }

  for (uint32_t ch = 0; ch < TDM_NUM_CH; ch++) tdm[f * TDM_NUM_CH + ch] = 0;
  tdm[f * TDM_NUM_CH + 7] = I2S_TAG_MP3_END;
  f++;

  for (; f < periodFrames; f++) {
    for (uint32_t ch = 0; ch < 7; ch++) tdm[f * TDM_NUM_CH + ch] = 0;
    tdm[f * TDM_NUM_CH + 7] = I2S_TAG_INVALID;
  }
}

static bool tdmWritePeriod(i2s_chan_handle_t tx, const int16_t* tdm, size_t periodFrames) {
  size_t bytesWritten = 0;
  const size_t bytes = periodFrames * TDM_NUM_CH * sizeof(int16_t);
  return i2s_channel_write(tx, tdm, bytes, &bytesWritten, portMAX_DELAY) == ESP_OK &&
         bytesWritten == bytes;
}

static void uzcTransmitStopFooterTdm(i2s_chan_handle_t tx) {
  int16_t pad[64 * TDM_NUM_CH];
  for (size_t i = 0; i < 64; i++) {
    for (uint32_t ch = 0; ch < 7; ch++) pad[i * TDM_NUM_CH + ch] = 0;
    pad[i * TDM_NUM_CH + 7] = I2S_TAG_PAD;
  }
  size_t written = 0;
  i2s_channel_write(tx, pad, sizeof(pad), &written, portMAX_DELAY);
}

static const uint8_t kUzuPcmMagic[8] = {'U', 'Z', 'U', 'C', 'P', 'W', '1', 0};

bool isUzuPcmFile(File& f) {
  uint8_t magic[8];
  if (!f.seek(0)) return false;
  if (f.read(magic, sizeof(magic)) != sizeof(magic)) return false;
  return memcmp(magic, kUzuPcmMagic, sizeof(kUzuPcmMagic)) == 0;
}

bool isUzuPcmFile(const char* path) {
  if (!path || !path[0]) return false;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  const bool ok = isUzuPcmFile(f);
  f.close();
  return ok;
}

bool isUzcMp3File(File& f) {
  uint8_t head[8];
  if (!f.seek(0)) return false;
  if (f.read(head, sizeof(head)) != sizeof(head)) return false;
  // PCM .UZU also starts with "UZUC" — require full UZUCPW1 magic for PCM path.
  if (memcmp(head, kUzuPcmMagic, sizeof(kUzuPcmMagic)) == 0) return false;
  if (memcmp(head, "UZUC", 4) != 0) return false;
  f.seek(0);
  UzcHeader hdr;
  return validateUzcForPlay(f, hdr);
}

bool isUzcMp3File(const char* path) {
  if (!path || !path[0]) return false;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  const bool ok = isUzcMp3File(f);
  f.close();
  return ok;
}

bool UzcTdmPlayer::begin(i2s_chan_handle_t txHandle) {
  m_tx = txHandle;
  return m_tx != nullptr;
}

void UzcTdmPlayer::process() {
}

bool UzcTdmPlayer::playPath(const char* path) {
  if (!path || !path[0] || !m_tx) {
    m_lastError = "ERR BAD_PATH";
    return false;
  }
  if (i2cModeGet() != TdmDataMode::MP3) {
    m_lastError = "ERR NOT_MP3_MODE";
    return false;
  }
  stop();
  strncpy(m_path, path, sizeof(m_path) - 1);
  m_path[sizeof(m_path) - 1] = '\0';
  m_stopReq = false;
  m_paused = false;
  m_active = true;
  m_lastError = nullptr;

  BaseType_t ok = xTaskCreatePinnedToCore(playbackTask, "uzcTdm", 8192, this, 5, &m_task, 1);
  if (ok != pdPASS) {
    m_active = false;
    m_lastError = "ERR TASK";
    return false;
  }
  return true;
}

void UzcTdmPlayer::stop() {
  m_stopReq = true;
  m_paused = false;
  for (int i = 0; i < 100 && m_task; i++) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (m_task) {
    vTaskDelete(m_task);
    m_task = nullptr;
  }
  m_active = false;
  m_stopReq = false;
}

void UzcTdmPlayer::pause() {
  m_paused = true;
}

bool UzcTdmPlayer::resume() {
  if (!m_active) {
    m_lastError = "ERR NOT_PLAYING";
    return false;
  }
  m_paused = false;
  return true;
}

void UzcTdmPlayer::playbackTask(void* param) {
  auto* self = static_cast<UzcTdmPlayer*>(param);
  File f = SD_MMC.open(self->m_path, FILE_READ);
  if (!f) {
    self->m_lastError = "ERR FILE_OPEN";
    self->m_active = false;
    self->m_task = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  UzcHeader hdr;
  if (!validateUzcForPlay(f, hdr)) {
    f.close();
    self->m_lastError = "ERR BAD_UZC";
    self->m_active = false;
    self->m_task = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  playUzcStreamTdm(f, self);
  f.close();
  self->m_active = false;
  self->m_task = nullptr;
  vTaskDelete(nullptr);
}

bool UzcTdmPlayer::playUzcStreamTdm(File& f, UzcTdmPlayer* self) {
  UzcHeader hdr;
  if (!parseUzcHeader(f, hdr)) return false;

  uint8_t chFileBuf[7][UZC_MAX_SLOT_SIZE];
  uint8_t chTxBuf[7][UZC_MAX_SLOT_SIZE];
  int16_t chWords[7][UZC_MAX_STEREO_WORDS];
  size_t nWords[7] = {0};
  int16_t tdmPad[UZC_TDM_PERIOD_MAX * TDM_NUM_CH];

  const uint16_t txSlotSize = uzcTransmitSlotSize(hdr);
  const size_t wordsPerSlot = uzcWordsPerSlot(txSlotSize);
  const size_t slotStereoFrames = wordsPerSlot;
  const size_t periodFrames = UZC_MP3_PERIOD_FRAMES;
  const uint16_t numCh = (hdr.channelCount > 7) ? 7 : hdr.channelCount;

  if (periodFrames > UZC_TDM_PERIOD_MAX) return false;

  Serial.printf("[UZC] TDM MP3 %uch period=%u slotWords=%u txSlot=%u (CH8=tag)\n",
                (unsigned)numCh, (unsigned)periodFrames, (unsigned)slotStereoFrames, (unsigned)txSlotSize);

  const int16_t* chWordPtrs[7];
  for (int i = 0; i < 7; i++) chWordPtrs[i] = chWords[i];

  for (uint32_t frameNo = 0; frameNo < hdr.totalFrameCount; frameNo++) {
    if (self->m_stopReq) break;
    while (self->m_paused) {
      vTaskDelay(pdMS_TO_TICKS(20));
      if (self->m_stopReq) break;
    }
    if (self->m_stopReq) break;

    const uint32_t blockOff = hdr.headerSize + frameNo * hdr.blockSize;
    if (!f.seek(blockOff, SeekSet)) break;

    for (uint16_t ch = 0; ch < numCh; ch++) {
      if (!readUzcChannelSlotFull(f, hdr.slotSize, chFileBuf[ch])) return false;
      if (!uzcPrepareSlotForTransmit(chFileBuf[ch], hdr.slotSize, hdr.maxFrameSize,
                                     chTxBuf[ch], txSlotSize)) {
        return false;
      }
      nWords[ch] = bytesToInt16Words(chTxBuf[ch], txSlotSize, chWords[ch], wordsPerSlot);
    }
    for (uint16_t ch = numCh; ch < hdr.channelCount; ch++) {
      if (!f.seek(f.position() + hdr.slotSize, SeekSet)) return false;
    }

    fillMp3TdmPeriodTagged(chWordPtrs, nWords, numCh, tdmPad, periodFrames, slotStereoFrames);
    if (!tdmWritePeriod(self->m_tx, tdmPad, periodFrames)) return false;
  }

  uzcTransmitStopFooterTdm(self->m_tx);
  return true;
}

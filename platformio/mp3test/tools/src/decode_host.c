/*
 * Host-side minimp3 decode of embedded UZC slot (for DUMP/CMPREF verification).
 * Build: pio run -d tools -e native
 * Run:   .pio/build/native/program.exe
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#include "minimp3.h"

#include "uzc_mp3_slot.h"
#include "uzc_mp3_pcm_ref.h"

static uint8_t g_slot[UZC_MP3_SLOT_BYTES];

static void slot_from_words(void) {
  for (uint32_t i = 0; i < UZC_MP3_SLOT_WORDS; i++) {
    int16_t w = kUzcMp3SlotWords[i];
    g_slot[i * 2 + 0] = (uint8_t)(w & 0xFF);
    g_slot[i * 2 + 1] = (uint8_t)((w >> 8) & 0xFF);
  }
}

static uint16_t mp3_frame_len(const uint8_t* h) {
  if (h[0] != 0xFF || ((h[1] & 0xE0) != 0xE0)) return 0;
  const bool mpeg1 = ((h[1] >> 3) & 3) == 3;
  const uint8_t br = (h[2] >> 4) & 0x0F;
  const uint8_t sr = (h[2] >> 2) & 3;
  static const uint16_t brk[] = {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0};
  static const uint32_t sr1[] = {44100,48000,32000,0};
  if (br == 0 || br == 0x0F || sr == 3) return 0;
  const uint32_t bitrate = brk[br] * 1000U;
  const uint32_t samplerate = sr1[sr];
  const uint8_t pad = (h[2] >> 1) & 1;
  return (uint16_t)((144U * bitrate / samplerate) + pad);
}

int main(void) {
  slot_from_words();
  const uint16_t mp3Len = mp3_frame_len(g_slot + 4);
  printf("slot real=%u mp3Len=%u\n", (unsigned)UZC_MP3_REAL_FRAME_SIZE, (unsigned)mp3Len);

  mp3dec_t dec;
  mp3dec_init(&dec);
  mp3d_sample_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  mp3dec_frame_info_t info;
  memset(&info, 0, sizeof(info));

  const int n = mp3dec_decode_frame(&dec, g_slot + 4, (int)mp3Len, pcm, &info);
  printf("decode samplesPerCh=%d channels=%d hz=%d\n", n, info.channels, info.hz);

  if (n <= 0) {
    return 1;
  }

  int32_t maxErr = 0;
  int64_t sumSq = 0;
  const int cmp = (n < (int)UZC_MP3_PCM_REF_SAMPLES) ? n : (int)UZC_MP3_PCM_REF_SAMPLES;
  for (int i = 0; i < cmp; i++) {
    const int32_t got = pcm[i];
    const int32_t ref = kUzcMp3PcmRef[i];
    const int32_t err = got - ref;
    const int32_t ae = err < 0 ? -err : err;
    if (ae > maxErr) maxErr = ae;
    sumSq += (int64_t)err * err;
  }
  const int rms = (cmp > 0) ? (int)(sqrt((double)sumSq / cmp)) : 0;
  printf("CMPREF maxErr=%ld rmsErr=%d\n", (long)maxErr, rms);
  printf("got[0..7]:");
  for (int i = 0; i < 8; i++) printf(" %d", pcm[i]);
  printf("\nref[0..7]:");
  for (int i = 0; i < 8; i++) printf(" %d", kUzcMp3PcmRef[i]);
  printf("\n");

  const uint32_t loopLen = (uint32_t)n;
  const int32_t last = pcm[loopLen - 1];
  const int32_t first = pcm[0];
  printf("seam raw last=%d first=%d diff=%ld (loopHz=%.1f)\n",
         last, first, (long)(last - first), 44100.0 / loopLen);

  return 0;
}

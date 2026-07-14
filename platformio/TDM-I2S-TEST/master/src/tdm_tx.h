#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr uint32_t TDM_NUM_CH = 8;
static constexpr uint32_t TDM_TAG_CH = 7;  // CH8 (0-based index 7)

static const int16_t TDM_TAG_RAW       = (int16_t)0xAAAA;
static const int16_t TDM_TAG_MP3_START = (int16_t)0xAA00;
static const int16_t TDM_TAG_MP3_END   = (int16_t)0x5500;
static const int16_t TDM_TAG_SLOT_DATA = (int16_t)0xAA55;
static const int16_t TDM_TAG_PAD       = (int16_t)0xFFFF;
static const int16_t TDM_TAG_INVALID   = (int16_t)0x0000;

bool tdmTxInit(uint32_t sampleRate);
bool tdmTxSetRate(uint32_t sampleRate);
bool tdmTxEnsureEnabled();
bool tdmTxIsReady();
void tdmTxDisable();

bool tdmWriteFrames(const int16_t* tdm, size_t frameCount);
bool tdmWriteTaggedStereoPairs(const int16_t* pairs, size_t stereoFrameCount);
bool tdmWriteRawMono(const int16_t* mono, size_t sampleCount);
void tdmWriteInvalidKeepalive(size_t frameCount);

void fillMp3TdmPeriodTaggedMono(const int16_t* ch1Words, size_t n1, int16_t* tdm,
                                size_t periodFrames, size_t slotStereoFrames);
void tdmTransmitStopFooter();

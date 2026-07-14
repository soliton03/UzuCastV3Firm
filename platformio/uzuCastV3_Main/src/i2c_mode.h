#pragma once

#include <Arduino.h>

enum class TdmDataMode : uint8_t {
  RAW = 1,
  MP3 = 2,
};

void i2cModeBegin();
void i2cModeProcess();
TdmDataMode i2cModeGet();
void i2cModeSet(TdmDataMode mode);
const char* tdmDataModeName(TdmDataMode mode);

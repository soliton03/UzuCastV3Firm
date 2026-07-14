#include "i2c_mode.h"

static volatile TdmDataMode g_tdmDataMode = TdmDataMode::RAW;
static volatile bool g_tdmModeChanged = false;

const char* tdmDataModeName(TdmDataMode mode) {
  switch (mode) {
    case TdmDataMode::RAW: return "RAW";
    case TdmDataMode::MP3: return "MP3";
    default: return "UNKNOWN";
  }
}

void i2cModeBegin() {
  g_tdmDataMode = TdmDataMode::RAW;
  g_tdmModeChanged = false;
}

void i2cModeProcess() {
  if (!g_tdmModeChanged) {
    return;
  }
  g_tdmModeChanged = false;
  Serial.printf("[TDM] mode -> %s\n", tdmDataModeName(g_tdmDataMode));
}

TdmDataMode i2cModeGet() {
  return g_tdmDataMode;
}

void i2cModeSet(TdmDataMode mode) {
  if (mode != TdmDataMode::RAW && mode != TdmDataMode::MP3) {
    return;
  }
  if (g_tdmDataMode != mode) {
    g_tdmDataMode = mode;
    g_tdmModeChanged = true;
  }
}

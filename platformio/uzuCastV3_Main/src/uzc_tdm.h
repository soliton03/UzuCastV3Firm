#pragma once

#include <Arduino.h>
#include <SD_MMC.h>
#include "driver/i2s_tdm.h"
#include "i2c_mode.h"

class UzcTdmPlayer {
public:
  bool begin(i2s_chan_handle_t txHandle);
  void process();
  bool playPath(const char* path);
  void stop();
  void pause();
  bool resume();
  bool isActive() const { return m_active; }
  bool isPaused() const { return m_paused; }
  const char* lastError() const { return m_lastError ? m_lastError : "ERR INTERNAL"; }

private:
  static void playbackTask(void* param);
  static bool playUzcStreamTdm(File& f, UzcTdmPlayer* self);

  i2s_chan_handle_t m_tx = nullptr;
  volatile bool m_active = false;
  volatile bool m_stopReq = false;
  volatile bool m_paused = false;
  TaskHandle_t m_task = nullptr;
  char m_path[160] = "";
  const char* m_lastError = nullptr;
};

bool isUzuPcmFile(const char* path);
bool isUzuPcmFile(File& f);
bool isUzcMp3File(const char* path);
bool isUzcMp3File(File& f);
bool readUzcLengthMs(const char* path, uint32_t& lengthMs);
bool readUzcTrackInfo(const char* path, uint32_t& channels, uint32_t& sampleRate, uint32_t& lengthMs);

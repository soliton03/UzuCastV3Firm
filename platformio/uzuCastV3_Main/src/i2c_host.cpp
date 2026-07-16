#include "uzu_i2c_config.h"
#if UZU_ENABLE_I2C

#include "i2c_host.h"
#include "i2c_protocol.h"
#include <Wire.h>

static constexpr int I2C_SDA_PIN = 41;
static constexpr int I2C_SCL_PIN = 40;
static constexpr uint32_t I2C_CLOCK_HZ = 100000;
static constexpr uint32_t I2C_POLL_MS = 200;
static constexpr uint32_t I2C_CONNECTED_REFRESH_MS = 800;

struct SlaveConnectedInfo {
  uint8_t flag = 0;
  uint8_t mac[6] = {0};
  bool valid = false;
};

static SlaveConnectedInfo g_slaveConnected[I2C_SLAVE_COUNT];
static uint8_t g_lastPendingMac[I2C_SLAVE_COUNT][6];
static bool g_pendingNotified[I2C_SLAVE_COUNT];
static uint32_t g_lastPollMs = 0;
static uint32_t g_lastConnectedRefreshMs = 0;

static bool i2cTransfer(uint8_t addr7, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) {
  if (addr7 == 0) {
    return false;
  }
  Wire.beginTransmission(addr7);
  for (size_t i = 0; i < txLen; i++) {
    Wire.write(tx[i]);
  }
  const uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    return false;
  }
  if (rxLen == 0) {
    return true;
  }
  if (Wire.requestFrom(addr7, (uint8_t)rxLen) != rxLen) {
    return false;
  }
  for (size_t i = 0; i < rxLen; i++) {
    if (!Wire.available()) {
      return false;
    }
    rx[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool i2cCmdRead(uint8_t addr7, uint8_t cmd, uint8_t* rx, size_t rxLen) {
  return i2cTransfer(addr7, &cmd, 1, rx, rxLen);
}

static bool i2cCmdWriteAck(uint8_t addr7, const uint8_t* tx, size_t txLen, uint8_t& ack) {
  return i2cTransfer(addr7, tx, txLen, &ack, 1);
}

void i2cHostBegin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
  g_lastPollMs = 0;
  g_lastConnectedRefreshMs = 0;
  for (uint8_t i = 0; i < I2C_SLAVE_COUNT; i++) {
    g_slaveConnected[i] = {};
    g_pendingNotified[i] = false;
    i2cZeroMac(g_lastPendingMac[i]);
  }
  Serial.printf("[I2C] host SDA=%d SCL=%d addr CH1-CH5=0x%02X-0x%02X\n",
                I2C_SDA_PIN, I2C_SCL_PIN,
                (unsigned)I2C_ADDR_BASE,
                (unsigned)(I2C_ADDR_BASE + I2C_SLAVE_COUNT - 1));
}

bool i2cHostGetStatus(uint8_t channelNo, uint8_t& status) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  return i2cCmdRead(addr, I2C_CMD_GET_STATUS, &status, 1);
}

bool i2cHostSetMode(uint8_t channelNo, uint8_t mode) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  const uint8_t tx[2] = {I2C_CMD_SET_MODE, mode};
  uint8_t ack = I2C_ACK_ERROR;
  return i2cCmdWriteAck(addr, tx, sizeof(tx), ack) && ack == I2C_ACK_OK;
}

bool i2cHostGetConnectedTarget(uint8_t channelNo, uint8_t& connectedFlag, uint8_t mac[6]) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  uint8_t rx[7] = {0};
  if (!i2cCmdRead(addr, I2C_CMD_GET_CONNECTED_TARGET, rx, sizeof(rx))) {
    return false;
  }
  connectedFlag = rx[0];
  memcpy(mac, rx + 1, 6);
  return true;
}

bool i2cHostGetPendingTarget(uint8_t channelNo, uint8_t& pendingFlag, uint8_t mac[6]) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  uint8_t rx[7] = {0};
  if (!i2cCmdRead(addr, I2C_CMD_GET_PENDING_TARGET, rx, sizeof(rx))) {
    return false;
  }
  pendingFlag = rx[0];
  memcpy(mac, rx + 1, 6);
  return true;
}

bool i2cHostNotifyConnectPermission(uint8_t channelNo, uint8_t result, const uint8_t mac[6]) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  uint8_t tx[8] = {I2C_CMD_NOTIFY_CONNECT_PERMISSION, result,
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]};
  uint8_t ack = I2C_ACK_ERROR;
  return i2cCmdWriteAck(addr, tx, sizeof(tx), ack) && ack == I2C_ACK_OK;
}

bool i2cHostClearBuffer(uint8_t channelNo, uint8_t bufferType) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  const uint8_t tx[2] = {I2C_CMD_CLEAR_BUFFER, bufferType};
  uint8_t ack = I2C_ACK_ERROR;
  return i2cCmdWriteAck(addr, tx, sizeof(tx), ack) && ack == I2C_ACK_OK;
}

bool i2cHostSetDelay(uint8_t channelNo, int16_t delayMs) {
  const uint8_t addr = i2cAddrFromChannel(channelNo);
  const uint8_t tx[3] = {I2C_CMD_SET_DELAY,
                         (uint8_t)(delayMs & 0xFF),
                         (uint8_t)((delayMs >> 8) & 0xFF)};
  uint8_t ack = I2C_ACK_ERROR;
  return i2cCmdWriteAck(addr, tx, sizeof(tx), ack) && ack == I2C_ACK_OK;
}

void i2cHostBroadcastSetMode(TdmDataMode mode) {
  const uint8_t i2cMode = (mode == TdmDataMode::MP3) ? I2C_MODE_MP3 : I2C_MODE_WAV_PCM;
  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    if (!i2cHostSetMode(ch, i2cMode)) {
      Serial.printf("[I2C] SET_MODE CH%u failed\n", (unsigned)ch);
    }
  }
}

void i2cHostBroadcastClearBuffer(uint8_t bufferType) {
  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    if (!i2cHostClearBuffer(ch, bufferType)) {
      Serial.printf("[I2C] CLEAR_BUFFER CH%u type=%u failed\n",
                    (unsigned)ch, (unsigned)bufferType);
    }
  }
}

static void refreshConnectedTable() {
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastConnectedRefreshMs) < I2C_CONNECTED_REFRESH_MS) {
    return;
  }
  g_lastConnectedRefreshMs = now;

  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    SlaveConnectedInfo info;
    if (i2cHostGetConnectedTarget(ch, info.flag, info.mac)) {
      info.valid = true;
      g_slaveConnected[ch - 1] = info;
    } else {
      g_slaveConnected[ch - 1].valid = false;
    }
  }
}

static bool isMacConnectedOnOtherSlave(uint8_t channelNo, const uint8_t mac[6]) {
  if (i2cMacIsZero(mac)) {
    return false;
  }
  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    if (ch == channelNo) {
      continue;
    }
    const SlaveConnectedInfo& info = g_slaveConnected[ch - 1];
    if (!info.valid || info.flag == 0) {
      continue;
    }
    if (i2cMacEquals(info.mac, mac)) {
      return true;
    }
  }
  return false;
}

static void processBtCoordination() {
  refreshConnectedTable();

  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    const uint8_t idx = (uint8_t)(ch - 1);
    uint8_t pendingFlag = 0;
    uint8_t pendingMac[6] = {0};
    if (!i2cHostGetPendingTarget(ch, pendingFlag, pendingMac)) {
      g_pendingNotified[idx] = false;
      continue;
    }
    if (pendingFlag == 0 || i2cMacIsZero(pendingMac)) {
      g_pendingNotified[idx] = false;
      continue;
    }

    if (g_pendingNotified[idx] && i2cMacEquals(g_lastPendingMac[idx], pendingMac)) {
      continue;
    }

    uint8_t connectedFlag = 0;
    uint8_t connectedMac[6] = {0};
    (void)i2cHostGetConnectedTarget(ch, connectedFlag, connectedMac);

    uint8_t result = 0x01;
    if (isMacConnectedOnOtherSlave(ch, pendingMac)) {
      result = 0x00;
      Serial.printf("[I2C] CH%u BT deny (in use): %02X:%02X:%02X:%02X:%02X:%02X\n",
                    (unsigned)ch,
                    pendingMac[0], pendingMac[1], pendingMac[2],
                    pendingMac[3], pendingMac[4], pendingMac[5]);
    } else if (connectedFlag != 0 && i2cMacEquals(connectedMac, pendingMac)) {
      result = 0x01;
    }

    if (!i2cHostNotifyConnectPermission(ch, result, pendingMac)) {
      Serial.printf("[I2C] CH%u NOTIFY_CONNECT_PERMISSION failed\n", (unsigned)ch);
    } else {
      memcpy(g_lastPendingMac[idx], pendingMac, 6);
      g_pendingNotified[idx] = true;
      if (result == 0x01) {
        Serial.printf("[I2C] CH%u BT allow: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      (unsigned)ch,
                      pendingMac[0], pendingMac[1], pendingMac[2],
                      pendingMac[3], pendingMac[4], pendingMac[5]);
      }
    }
  }
}

void i2cHostScanChannels() {
  Serial.println("[I2C] scan CH1-CH5...");
  uint8_t found = 0;
  for (uint8_t ch = 1; ch <= I2C_SLAVE_COUNT; ch++) {
    uint8_t status = 0;
    if (i2cHostGetStatus(ch, status)) {
      Serial.printf("[I2C]   CH%u addr=0x%02X status=0x%02X\n",
                    (unsigned)ch, (unsigned)i2cAddrFromChannel(ch), (unsigned)status);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   (no slaves found)");
  }
}

void i2cHostProcess() {
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastPollMs) < I2C_POLL_MS) {
    return;
  }
  g_lastPollMs = now;
  processBtCoordination();
}

#endif  // UZU_ENABLE_I2C

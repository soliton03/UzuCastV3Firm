#include "uzu_i2c_config.h"
#if UZU_ENABLE_I2C

#include "i2c_slave.h"
#include "i2c_protocol.h"
#include "i2c_slave_port.h"
#include <Wire.h>
#include <cstring>
#include <esp_system.h>

static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;
static constexpr int PIN_SEL0 = 17;
static constexpr int PIN_SEL1 = 18;
static constexpr int PIN_SEL2 = 19;

static uint8_t g_selValue = 0;
static uint8_t g_addr7 = 0;
static uint8_t g_respBuf[16];
static size_t g_respLen = 0;
static volatile bool g_resetRequested = false;

static uint8_t readSelValue() {
  const uint8_t sel0 = digitalRead(PIN_SEL0) ? 1 : 0;
  const uint8_t sel1 = digitalRead(PIN_SEL1) ? 1 : 0;
  const uint8_t sel2 = digitalRead(PIN_SEL2) ? 1 : 0;
  uint8_t sel = (uint8_t)((sel2 << 2) | (sel1 << 1) | sel0);
  if (sel > 5) {
    sel = 5;
  }
  return sel;
}

static void setAckResponse(uint8_t ack) {
  g_respBuf[0] = ack;
  g_respLen = 1;
}

static void handleCommand(uint8_t cmd) {
  g_respLen = 0;

  switch (cmd) {
    case I2C_CMD_GET_STATUS:
      g_respBuf[0] = i2cPortGetStatus();
      g_respLen = 1;
      break;

    case I2C_CMD_SET_MODE: {
      if (!Wire.available()) {
        setAckResponse(I2C_ACK_INVALID_PARAM);
        break;
      }
      const uint8_t mode = (uint8_t)Wire.read();
      setAckResponse(i2cPortSetMode(mode));
      break;
    }

    case I2C_CMD_GET_CONNECTED_TARGET: {
      uint8_t flag = 0;
      uint8_t mac[6] = {0};
      i2cPortGetConnectedTarget(&flag, mac);
      g_respBuf[0] = flag;
      memcpy(g_respBuf + 1, mac, 6);
      g_respLen = 7;
      break;
    }

    case I2C_CMD_GET_PENDING_TARGET: {
      uint8_t flag = 0;
      uint8_t mac[6] = {0};
      i2cPortGetPendingTarget(&flag, mac);
      g_respBuf[0] = flag;
      memcpy(g_respBuf + 1, mac, 6);
      g_respLen = 7;
      break;
    }

    case I2C_CMD_NOTIFY_CONNECT_PERMISSION: {
      if (Wire.available() < 7) {
        setAckResponse(I2C_ACK_INVALID_PARAM);
        break;
      }
      const uint8_t result = (uint8_t)Wire.read();
      uint8_t mac[6];
      for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)Wire.read();
      }
      setAckResponse(i2cPortNotifyConnectPermission(result, mac));
      break;
    }

    case I2C_CMD_SOFTWARE_RESET: {
      if (!Wire.available()) {
        setAckResponse(I2C_ACK_INVALID_PARAM);
        break;
      }
      const uint8_t code = (uint8_t)Wire.read();
      setAckResponse(i2cPortSoftwareReset(code));
      if (code == 0xA5 && g_respBuf[0] == I2C_ACK_OK) {
        g_resetRequested = true;
      }
      break;
    }

    case I2C_CMD_CLEAR_BUFFER: {
      if (!Wire.available()) {
        setAckResponse(I2C_ACK_INVALID_PARAM);
        break;
      }
      const uint8_t bufferType = (uint8_t)Wire.read();
      setAckResponse(i2cPortClearBuffer(bufferType));
      break;
    }

    case I2C_CMD_SET_DELAY: {
      if (Wire.available() < 2) {
        setAckResponse(I2C_ACK_INVALID_PARAM);
        break;
      }
      const uint8_t lo = (uint8_t)Wire.read();
      const uint8_t hi = (uint8_t)Wire.read();
      const int16_t delayMs = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
      setAckResponse(i2cPortSetDelay(delayMs));
      break;
    }

    default:
      setAckResponse(I2C_ACK_UNKNOWN_COMMAND);
      break;
  }

  while (Wire.available() > 0) {
    (void)Wire.read();
  }
}

static void onReceive(int len) {
  if (len < 1) {
    return;
  }
  const uint8_t cmd = (uint8_t)Wire.read();
  handleCommand(cmd);
}

static void onRequest() {
  if (g_respLen == 0) {
    Wire.write((uint8_t)0xFF);
    return;
  }
  Wire.write(g_respBuf, g_respLen);
}

uint8_t i2cSlaveSelValue() {
  return g_selValue;
}

uint8_t i2cSlaveAddr7() {
  return g_addr7;
}

uint8_t i2cSlaveChannelNo() {
  return (uint8_t)(g_selValue + 1);
}

void i2cSlaveBegin() {
  pinMode(PIN_SEL0, INPUT_PULLUP);
  pinMode(PIN_SEL1, INPUT_PULLUP);
  pinMode(PIN_SEL2, INPUT_PULLUP);

  g_selValue = readSelValue();
  g_addr7 = (uint8_t)(I2C_ADDR_BASE + g_selValue);
  g_resetRequested = false;
  g_respLen = 0;

  Wire.begin(g_addr7, I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.printf("[I2C] slave SEL=%u addr=0x%02X CH=%u SDA=%d SCL=%d\n",
                (unsigned)g_selValue, (unsigned)g_addr7, (unsigned)i2cSlaveChannelNo(),
                I2C_SDA_PIN, I2C_SCL_PIN);
}

void i2cSlavePoll() {
  if (g_resetRequested) {
    delay(10);
    esp_restart();
  }
}

#endif  // UZU_ENABLE_I2C

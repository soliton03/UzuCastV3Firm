#pragma once

#include <stdint.h>

// UzuCast Master-Slave I2C (UzuCast_Master_Slave_I2C_Spec.md)

static constexpr uint8_t I2C_ADDR_BASE = 0x30;
static constexpr uint8_t I2C_SLAVE_COUNT = 5;

static constexpr uint8_t I2C_CMD_GET_STATUS                = 0x01;
static constexpr uint8_t I2C_CMD_SET_MODE                  = 0x02;
static constexpr uint8_t I2C_CMD_GET_CONNECTED_TARGET      = 0x03;
static constexpr uint8_t I2C_CMD_GET_PENDING_TARGET        = 0x04;
static constexpr uint8_t I2C_CMD_NOTIFY_CONNECT_PERMISSION = 0x05;
static constexpr uint8_t I2C_CMD_SOFTWARE_RESET            = 0x06;
static constexpr uint8_t I2C_CMD_CLEAR_BUFFER              = 0x07;
static constexpr uint8_t I2C_CMD_SET_DELAY                 = 0x09;

static constexpr uint8_t I2C_ACK_OK              = 0x00;
static constexpr uint8_t I2C_ACK_UNKNOWN_COMMAND = 0x01;
static constexpr uint8_t I2C_ACK_INVALID_PARAM   = 0x02;
static constexpr uint8_t I2C_ACK_BUSY            = 0x03;
static constexpr uint8_t I2C_ACK_DENIED          = 0x04;
static constexpr uint8_t I2C_ACK_ERROR           = 0x05;

static constexpr uint8_t I2C_MODE_WAV_PCM = 0x00;
static constexpr uint8_t I2C_MODE_MP3     = 0x01;

static constexpr uint8_t I2C_STATUS_READY              = (1u << 0);
static constexpr uint8_t I2C_STATUS_PLAYING            = (1u << 1);
static constexpr uint8_t I2C_STATUS_BT_CONNECTED       = (1u << 2);
static constexpr uint8_t I2C_STATUS_BT_CONNECT_PENDING = (1u << 3);
static constexpr uint8_t I2C_STATUS_BUFFER_ACTIVE      = (1u << 4);
static constexpr uint8_t I2C_STATUS_ERROR              = (1u << 5);
static constexpr uint8_t I2C_STATUS_MODE_MP3           = (1u << 6);

static inline uint8_t i2cAddrFromChannel(uint8_t channelNo) {
  if (channelNo < 1 || channelNo > I2C_SLAVE_COUNT) {
    return 0;
  }
  return (uint8_t)(I2C_ADDR_BASE + (channelNo - 1));
}

static inline void i2cZeroMac(uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) {
    mac[i] = 0;
  }
}

static inline bool i2cMacEquals(const uint8_t a[6], const uint8_t b[6]) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

static inline bool i2cMacIsZero(const uint8_t mac[6]) {
  uint8_t z[6] = {0};
  return i2cMacEquals(mac, z);
}

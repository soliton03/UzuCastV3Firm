#pragma once

#include <stdint.h>

uint8_t i2cPortGetStatus();
void i2cPortGetConnectedTarget(uint8_t* connectedFlag, uint8_t mac[6]);
void i2cPortGetPendingTarget(uint8_t* pendingFlag, uint8_t mac[6]);
uint8_t i2cPortSetMode(uint8_t mode);
uint8_t i2cPortNotifyConnectPermission(uint8_t result, const uint8_t mac[6]);
uint8_t i2cPortClearBuffer(uint8_t bufferType);
uint8_t i2cPortSetDelay(int16_t delayMs);
uint8_t i2cPortSoftwareReset(uint8_t resetCode);

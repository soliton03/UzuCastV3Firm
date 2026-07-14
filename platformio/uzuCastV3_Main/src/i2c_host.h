#pragma once

#include <Arduino.h>
#include "i2c_mode.h"

void i2cHostBegin();
void i2cHostProcess();

bool i2cHostGetStatus(uint8_t channelNo, uint8_t& status);
bool i2cHostSetMode(uint8_t channelNo, uint8_t mode);
bool i2cHostGetConnectedTarget(uint8_t channelNo, uint8_t& connectedFlag, uint8_t mac[6]);
bool i2cHostGetPendingTarget(uint8_t channelNo, uint8_t& pendingFlag, uint8_t mac[6]);
bool i2cHostNotifyConnectPermission(uint8_t channelNo, uint8_t result, const uint8_t mac[6]);
bool i2cHostClearBuffer(uint8_t channelNo, uint8_t bufferType);
bool i2cHostSetDelay(uint8_t channelNo, int16_t delayMs);

void i2cHostBroadcastSetMode(TdmDataMode mode);
void i2cHostScanChannels();

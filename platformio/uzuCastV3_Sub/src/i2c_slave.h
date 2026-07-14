#pragma once

#include <Arduino.h>

void i2cSlaveBegin();
void i2cSlavePoll();
uint8_t i2cSlaveSelValue();
uint8_t i2cSlaveAddr7();
uint8_t i2cSlaveChannelNo();

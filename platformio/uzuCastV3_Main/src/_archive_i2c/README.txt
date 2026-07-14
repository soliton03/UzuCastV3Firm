I2C Master-Slave control (disabled by default)

Source preserved in:
  i2c_host.cpp / i2c_host.h
  i2c_protocol.h

Re-enable:
  1. platformio.ini: change -DUZU_ENABLE_I2C=0 to -DUZU_ENABLE_I2C=1
  2. Rebuild uzuCastV3_Main

See also: uzu_i2c_config.h, UzuCast_Master_Slave_I2C_Spec.md (repo docs)

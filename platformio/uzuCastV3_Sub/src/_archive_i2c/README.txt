I2C Master-Slave control (disabled by default)

Source preserved in:
  i2c_slave.cpp / i2c_slave.h
  i2c_slave_port.h
  i2c_protocol.h
  main.ino (block under #if UZU_ENABLE_I2C, "I2C slave port")

Re-enable:
  1. platformio.ini: change -DUZU_ENABLE_I2C=0 to -DUZU_ENABLE_I2C=1
  2. Rebuild uzuCastV3_Sub

When disabled, Sub uses direct BT connect (same as i2s_mp3test).

See also: uzu_i2c_config.h

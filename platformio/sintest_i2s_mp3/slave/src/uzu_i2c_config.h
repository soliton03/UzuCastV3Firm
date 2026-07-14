#pragma once

// Master-Sub I2C control (UzuCast_Master_Slave_I2C_Spec.md)
// Re-enable: platformio.ini build_flags += -DUZU_ENABLE_I2C=1
#ifndef UZU_ENABLE_I2C
#define UZU_ENABLE_I2C 0
#endif

#if UZU_ENABLE_I2C
#define I2C_BT_PENDING_CLEAR() g_i2c_bt_pending = false
#else
#define I2C_BT_PENDING_CLEAR() ((void)0)
#endif

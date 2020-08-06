// Copyright 2020 The Pigweed Authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.

#include <cinttypes>
#include <cstring>

#include "pw_bytes/endianness.h"
#include "pw_i2c/i2c_register_device.h"
#include "pw_i2c_baremetal_stm32f429/i2c_baremetal.h"
#include "pw_log/log.h"

// Address of the touch screen
// 7-bit address is 1000001, left shift to fit in the 8-bit data register
constexpr uint8_t ts_addr = 0b1000001 << 1;

int main(void) {
  PW_LOG_DEBUG("I2C demo app");

  pw::i2c::stm32f429_i2c3.Enable();

  uint8_t device_id_MSB;
  uint8_t device_id_LSB;

  pw::i2c::I2cRegisterDevice touch_screen{pw::i2c::stm32f429_i2c3,
                                          ts_addr,
                                          pw::IntSize::kUint8,
                                          pw::IntSize::kUint8,
                                          pw::ByteOrder::kLittleEndian};

  uint8_t device_id_MSB_reg = 0;
  uint8_t device_id_LSB_reg = 1;

  touch_screen.ReadRegister(&device_id_MSB_reg, &device_id_MSB);
  touch_screen.ReadRegister(&device_id_LSB_reg, &device_id_LSB);

  PW_LOG_DEBUG("Device ID: 0x%x", device_id_MSB << 8 | device_id_LSB);

  uint8_t clock_config_reg = 0x4;
  uint8_t clock_enable_mask = 0;

  uint8_t temp_sensor_config_reg = 0x60;
  uint8_t temp_sensor_enable_mask = 7;

  // Enable the touchscreen clock
  touch_screen.WriteRegister(&clock_config_reg, &clock_enable_mask);

  // Enable temperature sensor, acquire temperature
  touch_screen.WriteRegister(&temp_sensor_config_reg, &temp_sensor_enable_mask);

  uint8_t temp_MSB_reg = 0x61;
  uint8_t temp_LSB_reg = 0x62;

  uint8_t temp_MSB;
  uint8_t temp_LSB;

  touch_screen.ReadRegister(&temp_MSB_reg, &temp_MSB);
  touch_screen.ReadRegister(&temp_LSB_reg, &temp_LSB);

  PW_LOG_DEBUG("Temperature: %f", (float)(temp_MSB << 8 | temp_LSB) / 7.51);

  return 0;
}

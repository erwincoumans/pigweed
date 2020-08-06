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
#pragma once

#include <cinttypes>
#include <cstring>

#include "pw_i2c/i2c_bus.h"

namespace pw::i2c {

struct I2CBlock;
struct SdaScl;

class Stm32f429I2c : public I2cBus {
 public:
  Stm32f429I2c(SdaScl& sda,
               SdaScl& scl,
               volatile I2CBlock& i2c_hw,
               int32_t kI2cEnable);

  pw::Status Enable() override;

  pw::Status Disable() override;

  pw::Status WriteRead(I2cAddress address,
                       std::span<const std::byte> tx_buffer,
                       std::span<std::byte> rx_buffer) override;

 private:
  SdaScl& sda_;
  SdaScl& scl_;
  volatile I2CBlock& i2c_hw_;
  uint32_t kI2cEnable_;
};

// The I2C3 singleton
extern Stm32f429I2c stm32f429_i2c3;

}  // namespace pw::i2c
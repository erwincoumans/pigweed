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
#include <cstddef>
#include "pw_bytes/span.h"
#include "pw_status/status.h"

namespace pw::i2c {

typedef uint16_t I2cAddress;

// The I2cBus virtual interface for the master/controller mode that
// represents a single bus. This class should be inherited by the platform
// and provide an implementation to the methods included.
class I2cBus {
 public:
  virtual ~I2cBus() = default;

  virtual pw::Status Enable() = 0;

  virtual pw::Status Disable() = 0;

  // TODO: A timeout or a deadline should be added here later
  virtual pw::Status WriteRead(I2cAddress address,
                               ConstByteSpan tx_buffer,
                               ByteSpan rx_buffer) = 0;
};

}

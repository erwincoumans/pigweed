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

#include <cstddef>

#include "pw_assert/assert.h"
#include "pw_bytes/endianness.h"
#include "pw_i2c/i2c_bus.h"
#include "pw_preprocessor/util.h"

namespace pw::i2c {

// Represents an I2C slave register device.
// Instead of using a template class, command_type and register_type are taken
// as parameters during the construction time. By doing this, the object is
// evaluated in run-time instead of compile time
// enum class IntSize { kUint8, kUint16, kUint32, kUint64 };

// All values to read/write will be passed as pointer.
// Check demo_app_main.cc for sample usage
class I2cRegisterDevice {
 public:
  constexpr I2cRegisterDevice(I2cBus& i2c_bus,
                              I2cAddress i2c_addr,
                              pw::IntSize command_type,
                              pw::IntSize register_type,
                              pw::ByteOrder kEndianness)
      : command_type(command_type),
        register_type(register_type),
        kEndianness(kEndianness),
        i2c_bus_(i2c_bus),
        i2c_addr_(i2c_addr) {}

  // Write the command to the I2C bus, then read rx_buffer.size() bytes from
  // the bus.
  pw::Status Read(void* command, std::span<std::byte> rx_buffer) {
    const size_t command_size = pw::GetIntSize(command_type);
    std::byte command_buffer[command_size];

    pw::ConvertToEndian(command,
                        ByteSpan(command_buffer, command_size),
                        command_type,
                        kEndianness);

    return i2c_bus_.WriteRead(
        i2c_addr_, std::span(command_buffer, command_size), rx_buffer);
  }

  // Read the value in register_id into rx_buffer
  pw::Status ReadRegister(void* register_id, void* rx_buffer) {
    const size_t register_size = pw::GetIntSize(register_type);
    std::byte return_buffer[register_size];

    pw::Status read_status =
        Read(register_id, std::span(return_buffer, register_size));

    if (!read_status.ok()) {
      return read_status;
    }

    pw::GetEndian(ConstByteSpan(return_buffer, register_size),
                  rx_buffer,
                  register_type,
                  kEndianness);

    return pw::Status::OK;
  }

  // Write tx_buffer to the I2C bus.
  pw::Status Write(std::span<const std::byte> tx_buffer) {
    return i2c_bus_.Write(i2c_addr_, tx_buffer);
  }

  // Write the value to register_id
  pw::Status WriteRegister(void* register_id, void* value) {
    const size_t command_size = pw::GetIntSize(command_type);
    const size_t register_size = pw::GetIntSize(register_type);
    std::byte write_buffer[command_size + register_size];

    pw::ConvertToEndian(register_id,
                        ByteSpan(write_buffer, command_size),
                        command_type,
                        kEndianness);

    pw::ConvertToEndian(value,
                        ByteSpan(write_buffer + command_size, register_size),
                        register_type,
                        kEndianness);

    return i2c_bus_.Write(
        i2c_addr_, std::span(write_buffer, command_size + register_size));
  }

  I2cBus& GetI2cBus() { return i2c_bus_; }

  pw::IntSize command_type;
  pw::IntSize register_type;
  pw::ByteOrder kEndianness;

 private:
  I2cBus& i2c_bus_;
  I2cAddress i2c_addr_;
};

// Represents a register modification within a class that supports
// register reads and writes. Typical sequence of operations on such devices
// is:
//   1. Load existing values
//   2. Modify some bits as defined by the data sheet
//   3. Write the updated value
// Usage:
//    RegisterModification  m(device, 0x1234);
//    m.Bit<0>(true).SetBits<1,4>(3).Commit();
//    which is the same as:
//    m.SetBit(0, true).SetBitmask(0x1e, 3 << 1).Commit();
// Endianess:
//    Device read/write register handles endianess conversions. Assume
//    everything is in native endianess (bit 0 is 0x01, bit 4 is 0x10, bit
//    6 is 0x40, bit 11 is 0x800 and so on).
//
// Or if the device returns a modification:
//   Status s = device.ModifyRegister(0x1234).SetBits<4,8>(3).Commit();
//
// Commit returns ok only if both the Read (within Modify) and Write
// succeeds. If the read fails, no  write is attempted and Commit() will return
// the read failure code.
//
class RegisterModification {
 public:
  // Initialize a RegisterModification, load the current value of register
  RegisterModification(I2cRegisterDevice& device, void* register_id)
      : device_(device), register_id_(register_id) {
    load_status_ = device_.ReadRegister(register_id_, value_);
  }

  // Apply all bit operations.
  pw::Status Commit() {
    if (!load_status_.ok()) {
      return load_status_;
    }

    return device_.WriteRegister(register_id_, value_);
  }

  // Modify a subset of bits within the stored value.
  // The parameters lowBit/highBit are 0-inded bit indices.
  // Examples:
  //   SetBits<0, 3>(foo) translates into "value = (value & ~0x0F) | foo"
  //   SetBits<2, 3>(foo) translates into "value = (value & ~0x0C) | (foo << 2)"
  template <int lowBit, int highBit>
  RegisterModification& SetBits(int value) {
    PW_CHECK_INT_LE(lowBit, highBit);
    PW_CHECK_INT_GE(lowBit, 0);
    PW_CHECK_INT_LT(highBit, 8 * GetIntSize(register_type));

    constexpr int bit_count = highBit - lowBit + 1;

    if (command_type == pw::IntSize::kUint8) {
      uint8_t& value_casted = *reinterpret_cast<uint8_t*>(value_);
      uint8_t mask = (1 << bit_count) - 1;
      PW_CHECK_INT_EQ((value_casted & (~mask)), 0);  // check no overflow
      value_casted &= ~(mask << lowBit);
      value_casted |= (value & mask) << lowBit;
    } else if (command_type == pw::IntSize::kUint16) {
      uint16_t& value_casted = *reinterpret_cast<uint16_t*>(value_);
      uint16_t mask = (1 << bit_count) - 1;
      PW_CHECK_INT_EQ((value_casted & (~mask)), 0);  // check no overflow
      value_casted &= ~(mask << lowBit);
      value_casted |= (value & mask) << lowBit;
    } else if (command_type == pw::IntSize::kUint32) {
      uint32_t& value_casted = *reinterpret_cast<uint32_t*>(value_);
      uint32_t mask = (1 << bit_count) - 1;
      PW_CHECK_INT_EQ((value_casted & (~mask)), 0);  // check no overflow
      value_casted &= ~(mask << lowBit);
      value_casted |= (value & mask) << lowBit;
    } else if (command_type == pw::IntSize::kUint64) {
      uint64_t& value_casted = *reinterpret_cast<uint64_t*>(value_);
      uint64_t mask = (1 << bit_count) - 1;
      PW_CHECK_INT_EQ((value_casted & (~mask)), 0);  // check no overflow
      value_casted &= ~(mask << lowBit);
      value_casted |= (value & mask) << lowBit;
    }

    return *this;
  }

  // Set register value according to mask. Value must be already shifted to
  // align with mask
  RegisterModification& SetBitmask(void* mask, void* set_value) {
    if (command_type == pw::IntSize::kUint8) {
      uint8_t& value_casted = *reinterpret_cast<uint8_t*>(value_);
      uint8_t& mask_casted = *reinterpret_cast<uint8_t*>(mask);
      uint8_t& set_value_casted = *reinterpret_cast<uint8_t*>(set_value);
      value_casted &= ~mask_casted;
      value_casted |= (mask_casted & set_value_casted);
    } else if (command_type == pw::IntSize::kUint16) {
      uint16_t& value_casted = *reinterpret_cast<uint16_t*>(value_);
      uint16_t& mask_casted = *reinterpret_cast<uint16_t*>(mask);
      uint16_t& set_value_casted = *reinterpret_cast<uint16_t*>(set_value);
      value_casted &= ~mask_casted;
      value_casted |= (mask_casted & set_value_casted);
    } else if (command_type == pw::IntSize::kUint32) {
      uint32_t& value_casted = *reinterpret_cast<uint32_t*>(value_);
      uint32_t& mask_casted = *reinterpret_cast<uint32_t*>(mask);
      uint32_t& set_value_casted = *reinterpret_cast<uint32_t*>(set_value);
      value_casted &= ~mask_casted;
      value_casted |= (mask_casted & set_value_casted);
    } else if (command_type == pw::IntSize::kUint64) {
      uint64_t& value_casted = *reinterpret_cast<uint64_t*>(value_);
      uint64_t& mask_casted = *reinterpret_cast<uint64_t*>(mask);
      uint64_t& set_value_casted = *reinterpret_cast<uint64_t*>(set_value);
      value_casted &= ~mask_casted;
      value_casted |= (mask_casted & set_value_casted);
    }

    return *this;
  }

  // Set or clear bit based on given value
  RegisterModification& SetBit(uint8_t bitNumber, bool value) {
    PW_CHECK_INT_LT(bitNumber, 8 * GetIntSize(register_type));
    if (command_type == pw::IntSize::kUint8) {
      uint8_t mask = 1 << bitNumber;
      uint8_t value_shifted = value ? 1 << bitNumber : 0;
      return SetBitmask(&mask, &value_shifted);
    } else if (command_type == pw::IntSize::kUint16) {
      uint16_t mask = 1 << bitNumber;
      uint16_t value_shifted = value ? 1 << bitNumber : 0;
      return SetBitmask(&mask, &value_shifted);
    } else if (command_type == pw::IntSize::kUint32) {
      uint32_t mask = 1 << bitNumber;
      uint32_t value_shifted = value ? 1 << bitNumber : 0;
      return SetBitmask(&mask, &value_shifted);
    } else if (command_type == pw::IntSize::kUint64) {
      uint64_t mask = 1 << bitNumber;
      uint64_t value_shifted = value ? 1 << bitNumber : 0;
      return SetBitmask(&mask, &value_shifted);
    }
  }

  // Set or clear bit based on given value
  template <int bitNumber>
  RegisterModification& Bit(bool value) {
    return SetBit(bitNumber, value);
  }

  pw::IntSize command_type;
  pw::IntSize register_type;

 private:
  I2cRegisterDevice& device_;
  void* register_id_;
  void* value_;
  pw::Status load_status_;
};

}  // namespace pw::i2c

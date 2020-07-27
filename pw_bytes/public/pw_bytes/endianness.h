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
#include <cstring>
#include <span>

#include "pw_bytes/byte_builder.h"

namespace pw {

enum class IntSize { kUint8, kUint16, kUint32, kUint64 };

size_t GetIntSize(IntSize int_size) {
  if (int_size == IntSize::kUint8) {
    return sizeof(uint8_t);
  } else if (int_size == IntSize::kUint16) {
    return sizeof(uint16_t);
  } else if (int_size == IntSize::kUint32) {
    return sizeof(uint32_t);
  } else if (int_size == IntSize::kUint64){
    return sizeof(uint64_t);
  } else {
    return 0;  // Indicate an error
  }
}

void GetEndian(ConstByteSpan bytes,
               void* value,
               IntSize int_size,
               ByteOrder kEndianness) {
  if (int_size == IntSize::kUint8) {
    ByteBuffer<1> bb;
    bb.append(bytes);
    auto it = bb.begin();
    *static_cast<uint8_t*>(value) = it.ReadUint8();
  } else if (int_size == IntSize::kUint16) {
    ByteBuffer<2> bb;
    bb.append(bytes);
    auto it = bb.begin();
    *static_cast<uint16_t*>(value) = it.ReadUint16(kEndianness);
  } else if (int_size == IntSize::kUint32) {
    ByteBuffer<4> bb;
    bb.append(bytes);
    auto it = bb.begin();
    *static_cast<uint32_t*>(value) = it.ReadUint32(kEndianness);
  } else if (int_size == IntSize::kUint64) {
    ByteBuffer<8> bb;
    bb.append(bytes);
    auto it = bb.begin();
    *static_cast<uint64_t*>(value) = it.ReadUint64(kEndianness);
  }
}

void ConvertToEndian(void* value,
                     ByteSpan bytes,
                     IntSize int_size,
                     ByteOrder kEndianness) {
  if (int_size == IntSize::kUint8) {
    ByteBuffer<1> bb;
    bb.append(value, 1);
    auto it = bb.begin();
    uint8_t value_converted = it.ReadUint8();
    std::memcpy(bytes.data(), &value_converted, sizeof(uint8_t));
  } else if (int_size == IntSize::kUint16) {
    ByteBuffer<2> bb;
    bb.append(value, 2);
    auto it = bb.begin();
    uint16_t value_converted = it.ReadUint16(kEndianness);
    std::memcpy(bytes.data(), &value_converted, sizeof(uint16_t));
  } else if (int_size == IntSize::kUint32) {
    ByteBuffer<4> bb;
    bb.append(value, 4);
    auto it = bb.begin();
    uint32_t value_converted = it.ReadUint32(kEndianness);
    std::memcpy(bytes.data(), &value_converted, sizeof(uint32_t));
  } else if (int_size == IntSize::kUint64) {
    ByteBuffer<8> bb;
    bb.append(value, 8);
    auto it = bb.begin();
    uint64_t value_converted = it.ReadUint64(kEndianness);
    std::memcpy(bytes.data(), &value_converted, sizeof(uint64_t));
  }
}

}  // namespace pw
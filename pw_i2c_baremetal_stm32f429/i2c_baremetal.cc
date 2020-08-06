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

#include "pw_i2c_baremetal_stm32f429/i2c_baremetal.h"

#include "pw_preprocessor/compiler.h"
#include "pw_preprocessor/util.h"

namespace pw::i2c {

// Reset/clock configuration block (RCC).
// `reserved` fields are unimplemented features, and are present to ensure
// proper alignment of registers that are in use.
PW_PACKED(struct) RccBlock {
  uint32_t reserved1[12];
  uint32_t ahb1_config;
  uint32_t reserved2[3];
  uint32_t apb1_config;
  uint32_t apb2_config;
};

// I2C register block definition.
// Only the lower 16 bits of each register are used
struct I2CBlock {
  uint16_t control1;  // I2C_CR1
  uint16_t reserved1;
  uint16_t control2;  // I2C_CR2
  uint16_t reserved2;
  uint16_t own_address1;  // I2C_OAR1
  uint16_t reserved3;
  uint16_t own_address2;  // I2C_OAR2
  uint16_t reserved4;
  uint16_t data_register;  // I2C_DR
  uint16_t reserved5;
  uint16_t status1;  // I2C_SR1
  uint16_t reserved6;
  uint16_t status2;  // I2C_SR2
  uint16_t reserved7;
  uint16_t clock_control;  // I2C_CCR
  uint16_t reserved8;
  uint16_t TRISE;  // I2C_TRISE
  uint16_t reserved9;
  uint16_t FLTR;  // I2C_FLTR
  uint16_t reserved10;
};

// GPIO register block definition.
PW_PACKED(struct) GpioBlock {
  uint32_t modes;
  uint32_t out_type;
  uint32_t out_speed;
  uint32_t pull_up_down;
  uint32_t input_data;
  uint32_t output_data;
  uint32_t gpio_bit_set;
  uint32_t port_config_lock;
  uint32_t alt_low;
  uint32_t alt_high;
};

volatile RccBlock& platform_rcc =
    *reinterpret_cast<volatile RccBlock*>(0x40023800U);

volatile GpioBlock& gpio_a =
    *reinterpret_cast<volatile GpioBlock*>(0x40020000U);

volatile GpioBlock& gpio_c =
    *reinterpret_cast<volatile GpioBlock*>(0x40020800U);

volatile I2CBlock& i2c3 = *reinterpret_cast<volatile I2CBlock*>(0x40005C00U);

// Masks for ahb1_config (AHB1ENR) to enable the "A" and "C" GPIO pins.
constexpr uint32_t kGpioAEnable = 0x1u;
constexpr uint32_t kGpioCEnable = 0x4u;

// Constants related to GPIO output type open-drain
constexpr uint32_t kGpioOutputTypeOpenDrain = 1;
constexpr uint32_t kGpio9OutputTypePos = 9;
constexpr uint32_t kGpio8OutputTypePos = 8;

// Constants related to GPIO pull up/down resistor type masks
constexpr uint32_t kGpioPullTypeNone = 0;
constexpr uint32_t kGpio9PullTypePos = 18;
constexpr uint32_t kGpio8PullTypePos = 16;

// Constants related to GPIO mode register masks
constexpr uint32_t kGpioPortModeAlternate = 2;
constexpr uint32_t kGpio9PortModePos = 18;
constexpr uint32_t kGpio8PortModePos = 16;

// Constants related to GPIO alternate function mode
constexpr uint32_t kGpioAlternateFunctionI2C3 = 0x4u;
constexpr uint32_t kGpio9AltFuncHighPos = 4;
constexpr uint32_t kGpio8AltFuncHighPos = 0;

// Mask for apb2_config (APB1ENR) to enable I2C3.
constexpr uint32_t kI2C3Enable = 0x00800000u;

// Masks for i2c_cr1 to software reset
constexpr uint16_t kI2CSwReset = 0x8000u;

// Masks for i2c_cr2 frequency
constexpr uint16_t kI2CCr2Freq = 0x003fu;

// Mask for i2c_trise maximum rase time
constexpr uint8_t kI2CTRISE = 0x3fu;

//// Masks for I2C interrupts
// constexpr uint16_t kI2CErrIrq = 0x0100u; // error irq
// constexpr uint16_t kI2CEvtIrq = 0x0200u; // event irq
// constexpr uint16_t kI2CBufIrq = 0x0400u; // buffer irq

// Default core clock and CCR value. These are technically not constants,
// but since this app doesn't change the system clock constants will suffice.
constexpr uint32_t kSystemCoreClock = 16000000;
constexpr uint16_t kCCRStdMode = 0x32u;

// Masks for flags on CR1
constexpr uint16_t kCR1Pe = 0x0001u;     // PE
constexpr uint16_t kCR1Start = 0x0100u;  // START
constexpr uint16_t kCR1Stop = 0x0200u;   // STOP
constexpr uint16_t kCR1Ack = 0x0400u;    // ACK

// Masks for the flags on SR1
constexpr uint16_t kSR1Sb = 0x1u;     // SB
constexpr uint16_t kSR1Addr = 0x2u;   // ADDR
constexpr uint16_t kSR1Btf = 0x4u;    // ADDR
constexpr uint16_t kSR1Txe = 0x80u;   // TXE
constexpr uint16_t kSR1Rxne = 0x40u;  // RXNE

struct SdaScl {
  volatile GpioBlock& gpio;
  uint32_t kGpioEnable;
  uint32_t kGpioOutputTypePos;
  uint32_t kGpioPullTypePos;
  uint32_t kGpioPortModePos;
  uint32_t kGpioAltFuncHighPos;
  uint32_t kGpioAlternateFunction;

  SdaScl(volatile GpioBlock& gpio,
         uint32_t kGpioEnable,
         uint32_t kGpioOutputTypePos,
         uint32_t kGpioPullTypePos,
         uint32_t kGpioPortModePos,
         uint32_t kGpioAltFuncHighPos,
         uint32_t kGpioAlternateFunction)
      : gpio(gpio),
        kGpioEnable(kGpioEnable),
        kGpioOutputTypePos(kGpioOutputTypePos),
        kGpioPullTypePos(kGpioPullTypePos),
        kGpioPortModePos(kGpioPortModePos),
        kGpioAltFuncHighPos(kGpioAltFuncHighPos),
        kGpioAlternateFunction(kGpioAlternateFunction) {}
};

Stm32f429I2c::Stm32f429I2c(SdaScl& sda,
                           SdaScl& scl,
                           volatile I2CBlock& i2c_hw,
                           int32_t kI2cEnable)
    : sda_(sda), scl_(scl), i2c_hw_(i2c_hw), kI2cEnable_(kI2cEnable) {}

pw::Status Stm32f429I2c::Enable() {
  // Enable 'A' & 'C' GPIO clocks.
  platform_rcc.ahb1_config |= sda_.kGpioEnable;
  platform_rcc.ahb1_config |= scl_.kGpioEnable;

  // Enable the SDA (PC9 for I2C3_SDA):
  // open-drain, no pull-up/down ,alternate function mode, alternate function
  // i2c
  sda_.gpio.out_type |= kGpioOutputTypeOpenDrain << sda_.kGpioOutputTypePos;
  sda_.gpio.pull_up_down |= kGpioPullTypeNone << sda_.kGpioPullTypePos;
  sda_.gpio.modes |= kGpioPortModeAlternate << sda_.kGpioPortModePos;
  sda_.gpio.alt_high |= sda_.kGpioAlternateFunction << sda_.kGpioAltFuncHighPos;

  // Enable the SCL (PA8 for I2C3_SCL):
  // open-drain, no pull-up/down ,alternate function mode, alternate function
  // i2c
  scl_.gpio.out_type |= kGpioOutputTypeOpenDrain << scl_.kGpioOutputTypePos;
  scl_.gpio.pull_up_down |= kGpioPullTypeNone << scl_.kGpioPullTypePos;
  scl_.gpio.modes |= kGpioPortModeAlternate << scl_.kGpioPortModePos;
  scl_.gpio.alt_high |= scl_.kGpioAlternateFunction << scl_.kGpioAltFuncHighPos;

  // Initialize I2C3. Reset and clear all peripherals
  platform_rcc.apb1_config |= kI2C3Enable;
  i2c_hw_.control1 = kI2CSwReset;
  i2c_hw_.control1 = 0;

  // Enable peripheral clock
  i2c_hw_.control2 &= ~kI2CCr2Freq;
  i2c_hw_.control2 |= kSystemCoreClock / 1000000;

  // Initiate speed (standard mode up to 100kHz)
  i2c_hw_.clock_control = kCCRStdMode;

  // Initiate Max rise time
  i2c_hw_.TRISE = (kSystemCoreClock / 1000000 + 1) & kI2CTRISE;

  // Enable interrupts
  // i2c3.control2 |= (kI2CErrIrq | kI2CEvtIrq | kI2CBufIrq);

  // Enable the I2C peripheral
  i2c_hw_.control1 = kCR1Pe;

  return pw::Status::OK;
}

pw::Status Stm32f429I2c::Disable() {
  // Disable the I2C peripheral
  i2c3.control1 &= ~kCR1Pe;

  return pw::Status::OK;
}

pw::Status Stm32f429I2c::WriteRead(I2cAddress address,
                                   std::span<const std::byte> tx_buffer,
                                   std::span<std::byte> rx_buffer) {
  if (tx_buffer.size() == 0 && rx_buffer.size() == 0) {
    return pw::Status::OK;
  }

  // Generate start condition
  i2c_hw_.control1 |= kCR1Start;

  // Wait until SB is set
  while (!(i2c_hw_.status1 & kSR1Sb))
    ;

  if (tx_buffer.size() > 0) {
    // Send the slave address
    i2c_hw_.data_register = address & 0xff;

    // Wait until ADDR is set
    while (!(i2c_hw_.status1 & kSR1Addr))
      ;

    // Clear ADDR flag by reading SR1 and SR2
    i2c_hw_.status1;
    i2c_hw_.status2;

    for (size_t i = 0; i < tx_buffer.size(); i++) {
      // Wait until TXE is set
      while (!(i2c_hw_.status1 & kSR1Txe))
        ;

      // Send the data
      i2c_hw_.data_register = (uint16_t)tx_buffer[i];

      // Wait until BTF is set
      while (!(i2c_hw_.status1 & kSR1Btf))
        ;
    }
  }

  if (rx_buffer.size() > 0) {
    // Generate restart
    i2c_hw_.control1 |= kCR1Start;

    // Wait until SB is set
    while (!(i2c_hw_.status1 & kSR1Sb))
      ;

    // Send the slave address, set the last bit to 1 since it's a READ
    i2c_hw_.data_register = (address & 0xff) | 1;

    // Wait until ADDR is set
    while (!(i2c_hw_.status1 & kSR1Addr))
      ;

    // Clear ADDR flag by reading SR1 and SR2
    i2c_hw_.status1;
    i2c_hw_.status2;

    for (size_t i = 0; i < rx_buffer.size(); i++) {
      // Wait until RXNE bit is set
      while (!(i2c_hw_.status1 & kSR1Rxne))
        ;

      rx_buffer[i] = (std::byte)(i2c_hw_.data_register & 0xff);
    }
  }

  // Generate stop
  i2c_hw_.control1 |= kCR1Stop;

  return pw::Status::OK;
}

SdaScl pc9{gpio_c,
           kGpioCEnable,
           kGpio9OutputTypePos,
           kGpio9PullTypePos,
           kGpio9PortModePos,
           kGpio9AltFuncHighPos,
           kGpioAlternateFunctionI2C3};

SdaScl pa8{gpio_a,
           kGpioAEnable,
           kGpio8OutputTypePos,
           kGpio8PullTypePos,
           kGpio8PortModePos,
           kGpio8AltFuncHighPos,
           kGpioAlternateFunctionI2C3};

// The I2C3(use PC9 as SDA, PA8 as SCL) singleton
Stm32f429I2c stm32f429_i2c3{pc9, pa8, i2c3, kI2C3Enable};

}  // namespace pw::i2c
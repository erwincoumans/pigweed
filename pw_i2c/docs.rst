.. _chapter-pw-i2c:

.. default-domain:: cpp

.. highlight:: sh

---------
pw_i2c
---------
pw_i2c contains interface and utility functions for a register device on the I2C bus.

Compatibility
=============
C++17

Dependencies
============
* ``pw_preprocessor``
* ``pw_span``
* ``pw_bytes``

Features
========

pw::i2c::I2cBus
-----------------
The common interface for an I2C bus, which should be implemented by platform.

pw::i2c::I2cRegisterDevice
---------------------------
Represent an I2C register device on an I2C bus with address.
Utility functions are provided for easy read/write to registers

Future work
^^^^^^^^^^^
- deadline or timeout for WriteRead in I2cBus
- A virtual RegisterDevice interface for both I2C and SPI
- Clock related functions
- unstick() in I2cBus

.. _chapter-pw-i2c-baremetal-stm32f429:

.. default-domain:: cpp

.. highlight:: sh

-----------------------------
pw_i2c_baremetal_stm32f429
-----------------------------

``pw_i2c_baremetal_stm32f429`` implements the ``pw_i2c`` interface.

The baremetal I2C contains a simple WriteRead, no mutex or timeout
provided. The clock frequency is fixed.

Module usage
============
See the demo app ``demo_app.cc`` for an example.
After building an executable that utilizes this backend, flash the
produced .elf binary to the development board.

Dependencies
============
  * ``pw_bytes``
  * ``pw_i2c``
  * ``pw_preprocessor``

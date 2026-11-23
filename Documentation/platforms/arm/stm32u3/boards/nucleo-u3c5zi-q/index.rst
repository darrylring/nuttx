======================
ST NUCLEO-U3C5ZI-Q
======================

.. tags:: arch:arm, chip:stm32u3, part:stm32u3c5, vendor:st

The `NUCLEO-U3C5ZI-Q <https://www.st.com/en/evaluation-tools/nucleo-u3c5zi-q.html>`_
is an STM32 Nucleo-144 board featuring an STM32U3C5ZI microcontroller with
2 MiB of Flash and 640 KiB of SRAM.  The board includes an ST-LINK debugger
and programmer with a virtual COM port.

Clocking
========

NuttX runs at 96 MHz from the internal MSIRC0 oscillator.  The initial port
runs as a flat, nonsecure image with TrustZone disabled.

Configurations
==============

nsh
---

Configures the NuttShell with USART1 as the serial console.

fdcan1
------

Configures the NuttShell with FDCAN1 (PB8 RX, PB9 TX) in CAN FD internal
loopback mode, registered as ``/dev/can0``, and the ``can`` example.  FDCAN1
is connected to the on-board MCP2562FD transceiver, whose STBY pin (PD7) is
driven low at boot.  Disable ``CONFIG_STM32_FDCAN1_LOOPBACK`` to use a real
bus.

jumbo
-----

Enables a broad set of NuttX features to provide additional build and runtime
coverage.

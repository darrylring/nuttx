==========
ST STM32H5
==========

This is a port of the STM32H5 family.
The STM32H5 is a chip based on the ARM Cortex-M33.
Most code is adapted from legacy STM32 and STM32H7.

Board ports are available for STM32H503, STM32H533, STM32H563 and
STM32H573 devices.  Peripheral support varies by board configuration.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H503     Yes
STM32H523     No
STM32H533     Yes
STM32H562     No
STM32H563     Yes
STM32H573     Yes
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
ADC         Yes
ETH         Yes
DTS         Yes      Software trigger only.
FLASH       Yes      Hardware defines only.
FDCAN       Yes
GPDMA       Yes
GPIO        Yes
I2C         Yes
ICACHE      Yes      Uses the MPU to keep OTP, RO and EDATA flash uncached.
RCC         Yes
USART       Yes
LPUART      Yes
OCTOSPI     Yes      Serial flash (QSPI) or HyperBus memory.
PWR         Yes      Partial.
SPI         Yes
TIM         Yes
USB_FS      Yes      USB Device and Host Support.

AES         No
CEC         No
CORDIC      No
CRC         Yes
CRS         No
DAC         Yes
DBG         No
DCACHE      Yes      DCACHE1, external memory only.
DCMI        No
DLYB        Yes      Used by the HyperBus driver.
EXTI        Yes
FMAC        No
FSMC        No
GTZC        No
HASH        No
I3C         No
IWDG        Yes
LPTIM       Yes
OTFDEC      No
PKA         No
PSSI        No
RAMCFG      No
SBS         No
SDMMC       No
RNG         No
RTC         Yes
SAES        No
SAI         No
TAMP        No
UCPD        No
VREFBUF     No
WWDG        Yes
OTP         Yes

==========  =======  =====

ICACHE
------

With ``CONFIG_STM32_ICACHE`` the OTP, read-only (UID, flash size, package) and
EDATA flash areas (0x08fff000-0x09017fff) are mapped non-cacheable with an MPU
region, so they can be read like any other memory.

The MPU is not applied in the HardFault and NMI handlers (``HFNMIENA=0``), so
these areas must not be read from NMI or HardFault context: with the ICACHE
enabled such a read raises a bus fault.

USB FS Host
-----------

STM32 USB FS Host Driver Support. The STM32H5 is equipped with a Dual Role USB device
capable of operating as a device or host. 

Pre-requisites:

- CONFIG_USBHOST         - Enable USB host support
- CONFIG_STM32_USBFS_HOST  - Enable the STM32 USB OTG FS block in host mode

USB host requires a stable 48MHz clock. This should come from a PLL driven by the HSE.
HSI48 cannot be reliably used in host mode due to drift. It can only be used in device mode.

Options:

- STM32H5_USBDRD_NCHANNELS - Number of host channels. Default 8

- STM32H5_USBDRD_DESCSIZE - Maximum size of a descriptor.  Default: 128

Timer Capture
-------------

The STM32H5 timer capture driver measures the frequency and the duty cycle of
a PWM input signal using the PWM input mode of the timer. It is registered as
``/dev/cap0``, ``/dev/cap1``, ... (see ``include/nuttx/timers/capture.h``).

Only the timers with two input channels can be used: TIM1, TIM2, TIM3, TIM4,
TIM5, TIM8, TIM12 and TIM15.

Pre-requisites:

- ``CONFIG_CAPTURE`` - Enable the upper-half capture driver
- ``CONFIG_STM32_TIMn`` and ``CONFIG_STM32_TIMn_CAP`` - Reserve timer ``n`` for capture

Options:

- ``CONFIG_STM32_TIMn_CHANNEL`` - The input channel, 1 or 2. The signal is measured
  on the pin of that channel. The other channel is mapped internally to the same input.
- ``CONFIG_STM32_TIMn_CLOCK`` - The counter frequency. It limits the resolution
  of the measurement and the lowest frequency that can be measured (the counter must not
  overflow within a period of the input signal).

The board must define the pin of the selected channel in ``board.h``, e.g.
``GPIO_TIM4_CH1IN`` (see ``GPIO_TIMn_CHmIN_x`` in the pin map) and register the drivers
with ``stm32_cap_initialize()`` and ``cap_register_multiple()``, see the nucleo-h563zi
board for an example.

OTP
---

STM32H5 parts have a 2 KiB one-time programmable (OTP) area. It's organized
into 32 blocks of 32 16-bit words. Each word can be successfully programmed once.
Each block may be permanently locked at any point. Written words may be read.

There are two APIs for it: a block-oriented API with locking, and a lower-level
word API. An optional eFuse character device is built on top of the word API.

Block API
~~~~~~~~~

This API is organized into 32 blocks of 32 16-bit words. Each word can be
successfully programmed once. Each block may be permanently locked at any
point. Written words may be read.

Writing the same word more than once is unsupported. Doing so may cause corruption.
Reading an unwritten word raises an exception.
To simplify the programming model, the OTP API
locks blocks after any part is written. The user may check which blocks are locked.
Blocks are tagged as "written" using this lock status
without the need for out-of-band metadata. Some of the words within a locked
block may be left unwritten, so the exception may still be raised if an unwritten word
within a locked block is read.

.. code:: c

   int stm32_otp_write(const uint16_t *data, uint16_t len, uint32_t offset);
   int stm32_otp_read(uint16_t *data, uint16_t len, uint32_t offset);
   uint32_t stm32_otp_getlockstatus(void);

The API allows cross-block reads/writes that don't necessarily start/end at block boundaries.
Any block affected by ``stm32_otp_write`` will be locked. The user should be aware
of the block size and count when partitioning the OTP area for their needs.
``len`` is the number of bytes - not words. It has no alignment requirement. ``offset`` is
the offset in bytes. It must be a multiple of 4.

Word API
~~~~~~~~

``CONFIG_STM32H5_OTP_WORD`` builds direct 16- or 32-bit word access to the
OTP area, independent of the block API above -- there is no locking, and
no relation between a "word" index here and the block API's byte
``offset``:

.. code:: c

   int stm32_otp_word_read16(uint32_t word, uint16_t *value);
   int stm32_otp_word_read32(uint32_t word, uint32_t *value);

Unlike the block API, reading a blank (never programmed) word does not
raise an exception: it legitimately reads back as
``0xffff``/``0xffffffff``, which is returned through ``*value`` either
way. Since whether a word has been written is known, that is reported
through the return value: ``-ENODATA`` for a blank word, ``OK`` for one
that holds real data. ``-EIO`` is returned only when a word's ECC does
not check out at all, i.e. it is neither blank nor the value its own
program operation wrote.

With ``CONFIG_STM32H5_OTP_WRITE`` also set:

.. code:: c

   int stm32_otp_word_write16(uint32_t word, uint16_t value);
   int stm32_otp_word_write32(uint32_t word, uint32_t value);

Each word may be programmed once. Writing a word that already holds
exactly the value requested is a harmless no-op that returns ``OK``.
Writing a word that already holds a different value returns ``-EEXIST``.
A hardware programming failure, or a post-write readback mismatch,
returns ``-EIO``.

eFuse Character Device
~~~~~~~~~~~~~~~~~~~~~~

``CONFIG_STM32H5_EFUSE`` (which selects ``CONFIG_STM32H5_OTP_WORD``)
registers the OTP area as a NuttX efuse character device, by default
``/dev/efuse``, built on the word API above. See
:doc:`/os/drivers/character/efuse` for the ``EFUSEIOC_READ_FIELD``/
``EFUSEIOC_WRITE_FIELD`` ioctl interface. Field bit offsets index into the
flat bit space of the OTP area at 16 bits per word, the same as the word
API's ``word`` index. Writing a field requires ``CONFIG_STM32H5_OTP_WRITE``;
without it, writes are refused with ``-EPERM``.

OCTOSPI HyperBus Memory
-----------------------

With ``CONFIG_STM32_HYPERBUS`` OCTOSPI1 drives a HyperBus memory such as a
HyperRAM, mapped for reads and writes at ``0x90000000``
(``STM32_HYPERBUS_BASE``).  OCTOSPI1 then cannot be used by the serial flash
driver: ``CONFIG_STM32_HYPERBUS`` depends on ``!CONFIG_STM32_QSPI1``.

Options:

- ``CONFIG_STM32_HYPERBUS_SIZE`` - Size of the memory in bytes, a power of
  two (default 8 MiB).  For a multi-chip package give the total size.
- ``CONFIG_STM32_HYPERBUS_MPU`` - Map the memory with an MPU region: Normal
  memory, read/write for all, outer shareable.  It is write-back cacheable
  when ``CONFIG_STM32_DCACHE1`` is set and non-cacheable otherwise (default
  y).
- ``CONFIG_STM32_HYPERBUS_HEAP`` - Add the whole memory to the heap as one
  more region.  Raise ``CONFIG_MM_REGIONS`` by one.
- ``CONFIG_STM32_DCACHE1`` - Enable DCACHE1, see below.

The board does the following:

1. Select the OCTOSPI1 kernel clock with ``STM32_RCC_CCIPR4_OCTOSPI1SEL`` in
   ``board.h``.
2. Define the pins in ``board.h``: ``GPIO_OCTOSPI1_IO0`` to
   ``GPIO_OCTOSPI1_IO7``, ``GPIO_OCTOSPI1_CLK``, ``GPIO_OCTOSPI1_NCS`` and
   ``GPIO_OCTOSPI1_DQS``.  ``GPIO_OCTOSPI1_NCLK`` is optional; it is only
   used by 1.8 V memories.  The driver configures these pins.
3. Reset the memory through its ``RESET#`` pin, if it has one, and wait for
   the time given in its data sheet.
4. Call ``stm32_hyperbus_initialize()``.  With
   ``CONFIG_STM32_HYPERBUS_HEAP`` the call must be made from
   ``stm32_boardinitialize()``, before the heap is set up; the function
   busy-waits and can run there.  If it fails, the memory is not added to
   the heap.

``struct stm32_hyperbus_config_s`` (``stm32_hyperbus.h``) gives the timings
of the memory.  ``tcshi`` (minimum ``CS#`` high time) and ``tcsm`` (maximum
``CS#`` low time, for the memory refresh) are in ns; the driver converts them
to CLK cycles and rounds to the safe side.  ``tacc`` is the initial latency
in CLK cycles and must match the latency set in the memory (its CR0
register).  ``fixedlat`` must match the fixed-latency setting of the memory.
HyperBus transfers data on both clock edges, so ``prescaler`` must be even
(50% duty cycle), or 1 with a kernel clock of 50% duty cycle.  With
``tcsm`` set, the driver also releases ``CS#`` after 16 idle CLK cycles
(OCTOSPI timeout counter), so that a stalled prefetch cannot hold ``CS#``
low longer than ``tcsm``.

With ``usedlyb`` the driver calibrates the OCTOSPI1 delay block on a free
running clock (RM0481 27.4.4) and delays DQS by ``dlybsel`` unit delays.
``STM32_HYPERBUS_DLYBSEL_AUTO`` picks a quarter of the clock period.  Enable
memory manager debug messages (``CONFIG_DEBUG_MM_INFO``) to see the
calibrated values.

``iocomp`` enables the VDDIO I/O compensation cell and applies its code
(SBS_CCCSR EN1 and CS1).  The cell needs the CSI oscillator, which the driver
turns on, and VDDIO between 2.7 V and 3.6 V or between 1.62 V and 2 V.

``stm32_hyperbus_readreg()`` and ``stm32_hyperbus_writereg()`` access the
register space of the memory, for example ``HYPERRAM_ID0`` and
``HYPERRAM_CR0``.  They stop memory-mapped mode while interrupts are
disabled: no DMA may use the memory meanwhile.  After a write to CR0 that
changes the latency, call ``stm32_hyperbus_initialize()`` again with the new
timings.

Example for an 8 MiB, 3.3 V HyperRAM at 125 MHz (250 MHz kernel clock from
PLL2R).  ``board.h``:

.. code:: c

   #define STM32_RCC_CCIPR4_OCTOSPI1SEL RCC_CCIPR4_OCTOSPI1SEL_PLL2RCK

   #define GPIO_OCTOSPI1_DQS  (GPIO_OCTOSPI1_DQS_2  | GPIO_SPEED_100MHZ) /* PB2  */
   #define GPIO_OCTOSPI1_CLK  (GPIO_OCTOSPI1_CLK_5  | GPIO_SPEED_100MHZ) /* PF10 */
   #define GPIO_OCTOSPI1_NCLK (GPIO_OCTOSPI1_NCLK_1 | GPIO_SPEED_100MHZ) /* PB5  */
   #define GPIO_OCTOSPI1_NCS  (GPIO_OCTOSPI1_NCS_1  | GPIO_SPEED_100MHZ) /* PE11 */
   #define GPIO_OCTOSPI1_IO0  (GPIO_OCTOSPI1_IO0_5  | GPIO_SPEED_100MHZ) /* PF8  */
   #define GPIO_OCTOSPI1_IO1  (GPIO_OCTOSPI1_IO1_4  | GPIO_SPEED_100MHZ) /* PF9  */
   #define GPIO_OCTOSPI1_IO2  (GPIO_OCTOSPI1_IO2_3  | GPIO_SPEED_100MHZ) /* PE2  */
   #define GPIO_OCTOSPI1_IO3  (GPIO_OCTOSPI1_IO3_4  | GPIO_SPEED_100MHZ) /* PF6  */
   #define GPIO_OCTOSPI1_IO4  (GPIO_OCTOSPI1_IO4_4  | GPIO_SPEED_100MHZ) /* PH2  */
   #define GPIO_OCTOSPI1_IO5  (GPIO_OCTOSPI1_IO5_4  | GPIO_SPEED_100MHZ) /* PE8  */
   #define GPIO_OCTOSPI1_IO6  (GPIO_OCTOSPI1_IO6_4  | GPIO_SPEED_100MHZ) /* PE9  */
   #define GPIO_OCTOSPI1_IO7  (GPIO_OCTOSPI1_IO7_3  | GPIO_SPEED_100MHZ) /* PE10 */

   /* HyperRAM RESET#, a plain output, low (reset) at configuration */

   #define GPIO_HYPERRAM_RESET (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                GPIO_SPEED_2MHZ | GPIO_OUTPUT_CLEAR | \
                                GPIO_PORTE | GPIO_PIN12)

``stm32_boardinitialize()``:

.. code:: c

   static const struct stm32_hyperbus_config_s g_hyperram =
   {
     .kerfreq   = STM32_PLL2R_FREQUENCY, /* 250 MHz */
     .prescaler = 2,                     /* 125 MHz CLK */
     .tacc      = 7,
     .trwr      = 7,
     .tcshi     = 6,
     .tcsm      = 2000,
     .dlybsel   = 3,
     .fixedlat  = true,
     .iocomp    = true,
     .usedlyb   = true,
   };

   uint16_t id0;

   stm32_configgpio(GPIO_HYPERRAM_RESET);
   up_mdelay(1);
   stm32_gpiowrite(GPIO_HYPERRAM_RESET, true);
   up_udelay(200);

   if (stm32_hyperbus_initialize(&g_hyperram) == OK &&
       stm32_hyperbus_readreg(HYPERRAM_ID0, &id0) == OK)
     {
       syslog(LOG_INFO, "HyperRAM ID0 %04x\n", id0);
     }

Instead of ``CONFIG_STM32_HYPERBUS_HEAP``, a board can keep the memory as a
separate heap, initialized later from ``stm32_bringup()``:

.. code:: c

   struct mm_heap_s *heap;

   heap = mm_initialize("hyperram", (void *)STM32_HYPERBUS_BASE,
                        STM32_HYPERBUS_SIZE);

``ramtest`` (``CONFIG_TESTING_RAMTEST``) checks the memory:
``ramtest -w -a 0x90000000 -s 8388608``.

DCACHE1
-------

DCACHE1 is a 4 KiB cache on the Cortex-M33 S-AHB bus.  It only caches the
external memory windows (OCTOSPI1 and FMC, RM0481 9.3), and only accesses
that the MPU marks cacheable.  ``CONFIG_STM32_DCACHE1`` builds
``stm32_dcache.c``; the HyperBus driver enables the cache after the memory
is mapped.  Boards with other external memory call
``stm32_dcache_enable()`` themselves.

DCACHE1 is not managed by ``up_invalidate_dcache()`` and the other
``up_*_dcache()`` functions, which handle the CPU cache.  Before a DMA reads
data that the CPU wrote to cached external memory, call
``stm32_dcache_clean()``; before the CPU reads data that a DMA wrote, call
``stm32_dcache_invalidate()``.  These functions work on 16-byte lines, so
keep DMA buffers line aligned.

Clocks
------

``STM32_BOARD_HSIKERON_ENABLE`` can be defined in board.h to keep HSI running in
STOP mode. This can be used to keep a peripheral clocked by HSI running in
STOP mode.

``STM32_RCC_CCIPR1_U[S]ARTxSEL`` (e.g. ``STM32_RCC_CCIPR1_USART3SEL``) can be defined as one of

- ``RCC_CCIPR1_U[S]ARTxSEL_RCCPCLK1``
- ``RCC_CCIPR1_U[S]ARTxSEL_PLL2QCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_PLL3QCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_HSIKERCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_CSIKERCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_LSECK``

E.g. ``RCC_CCIPR1_USART3SEL_HSIKERCK`` in board.h to select the clock source for that USART.
The clock source is set in RCC initialization. Only stm32_serial.c is aware of this setting.
TODO: Make stm32_lowputc.c aware of this clock source setting too.

References
=================
[RM0481] Reference Manual: STM32H523/33xx, STM32H562/63xx, and STM32H573xx Arm® -based 32-bit MCUs

Support
=================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

==========
CRC Driver
==========

The CRC driver class exposes hardware CRC engines as character devices
(``/dev/crc0``, ``/dev/crc1``, ...). It follows the usual NuttX
upper-half/lower-half split:

-  **Upper half**: ``drivers/crc/crc_upperhalf.c``. Implements the
   character device, per-open sessions, locking, and the software parts of
   the CRC (output reflection and final XOR).
-  **Lower half**: architecture-specific, e.g.
   ``arch/arm/src/stm32h5/stm32_crc.c``. Drives the actual hardware.

Header file: ``include/nuttx/crc/crc.h``.

Enable the driver with ``CONFIG_CRC``. It depends on
``CONFIG_ARCH_HAVE_CRC``, which is selected by chips that provide a lower
half.

CRC profile
===========

A CRC algorithm is described by a ``struct crc_config_s``, following the
usual "Rocksoft" parameter model:

.. code-block:: c

   struct crc_config_s
   {
     uint8_t  width;   /* 32, 16, 8, or 7 */
     uint32_t poly;    /* MSB-first, implicit leading 1 */
     uint32_t init;    /* Seed */
     bool     refin;   /* Reflect each input byte */
     bool     refout;  /* Reflect the final result */
     uint32_t xorout;  /* XORed with the (possibly reflected) result */
   };

For example, standard CRC-32 (Ethernet, zlib) is ``width = 32``,
``poly = 0x04c11db7``, ``init = 0xffffffff``, ``refin = refout = true``,
``xorout = 0xffffffff``.

Reflection
----------

-  ``refin`` is performed by the hardware. It is programmed into the engine
   by the lower half's ``configure`` operation.
-  ``refout`` is always performed in software by the upper half, when the
   result is read. The upper half reverses the low ``width`` bits of the
   value and then applies ``xorout``.

``refout`` is deliberately not delegated to a hardware "reverse output"
feature. The upper half stores the engine's raw running value between
``write()`` calls and reloads it to resume the stream. If the hardware
reversed that value, resumed multi-chunk streams would be corrupted. Keeping
the stored state raw also means the result can be read mid-stream without
disturbing it.

Sessions
========

Every ``open()`` creates an independent session with its own profile and
running CRC value. Multiple sessions may stream concurrently; they
time-share the single hardware engine, which the upper half serializes with
a mutex and reprograms (profile and checkpoint) on every ``write()``. The
engine is enabled when the first session opens and disabled when the last
one closes.

User interface
==============

Configure the session with ``CRCIOC_CONFIG``, feed data with ``write()``,
and obtain the result with ``read()`` or ``CRCIOC_RESULT``. All operations
other than ``CRCIOC_CONFIG`` fail with ``-EINVAL`` until the session has
been configured.

``write()``
   Feeds the buffer into the CRC. Returns the number of bytes written.

``read()``
   Returns the finalized CRC (``refout`` and ``xorout`` applied) as a
   native-endian ``uint32_t`` (a shorter buffer receives the leading bytes).
   Does not alter the running state.

``CRCIOC_CONFIG``
   Argument: ``const struct crc_config_s *``. Sets the profile and seeds the
   running value with ``init``. May be issued again to switch profiles.

``CRCIOC_RESET``
   Argument: none. Re-seeds the running value with ``init`` while keeping
   the profile, to start a new stream on the same file descriptor.

``CRCIOC_RESULT``
   Argument: ``uint32_t *``. Stores the finalized CRC. Does not alter the
   running state, so more data may be written afterwards.

Example
-------

.. code-block:: c

   #include <fcntl.h>
   #include <unistd.h>
   #include <sys/ioctl.h>
   #include <nuttx/crc/crc.h>

   static const struct crc_config_s crc32 =
   {
     .width  = 32,
     .poly   = 0x04c11db7,
     .init   = 0xffffffff,
     .refin  = true,
     .refout = true,
     .xorout = 0xffffffff,
   };

   uint32_t result;
   int fd = open("/dev/crc0", O_RDWR);

   ioctl(fd, CRCIOC_CONFIG, (unsigned long)&crc32);
   write(fd, "123456789", 9);
   ioctl(fd, CRCIOC_RESULT, (unsigned long)&result);   /* 0xcbf43926 */
   close(fd);

Lower half interface
====================

A lower half fills in a ``struct crc_lowerhalf_s`` (``cl_ops`` and
``cl_priv``) and calls ``crc_register("/dev/crc0", lower)``. All members of
``struct crc_ops_s`` are mandatory:

``setup`` / ``shutdown``
   Enable and disable the engine (first open / last close).

``configure``
   Program polynomial, width and input reflection. Must not touch the
   running value. Should return an error for profiles the hardware cannot
   handle; the upper half does not validate ``width`` or emulate ``refin``.

``checkpoint_load``
   Load a value as the engine's current running value.

``write``
   Feed a buffer and return the engine's *raw* running value, with no output
   reflection or XOR. Whether DMA is used is entirely up to the lower half.

The upper half calls ``configure``, ``checkpoint_load`` and ``write`` with
its mutex held, so the lower half need not do its own locking against other
sessions.

Supported hardware
==================

-  **STM32H5**: ``CONFIG_STM32_CRC`` (``arch/arm/src/stm32h5/stm32_crc.c``).
   Optional GPDMA feeding for large buffers via ``CONFIG_STM32_CRC_DMA`` and
   ``CONFIG_STM32_CRC_DMA_THRESHOLD``.

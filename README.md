Overview
========

Sometimes it's a pain to get the kernel modules loaded for SPIDev on Raspberry Pi.

Sometimes you just need a quick way to access SPI devices.

This is a library for accessing SPI devices quickly, but with little regard to playing
nice with other processes/peripherals.  The access to the SPI peripheral
is mmap()'d, without any provosion for mutual exclusion of the peripheral.

Initializiation will update GPIO registers for MUXing the pins to SPI functions.
So don't use this with anything that uses the Linux GPIO drivers on the same
pins as the SPI pins.

Interrupts are not implemented.

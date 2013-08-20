MCP2210 driver for Linux
========================

This is a Linux device driver for the mcp2210 USB-to-SPI bridge.  It includes a userland utility for testing, configuration and control and currently builds as an out-of-tree module.

This is a USB interface driver, NOT a usbhid driver!  The MCP2210 isn't a device that interfaces with humans, and they only choose the HID interface to avoid the hassle of writing and maintaining drivers for every other platform. However, it means that you use this whole layer of software, protocols, drivers, etc that you don't really need.  Therefore, this driver skips that layer and communicates directly with the device via standard interrupt URBs (not hid reports).

Unlike other libraries, this driver connects the SPI devices directly to the spi sub-system, allowing you to choose whatever spi protocol driver you wish, with the default being spidev (which allows you to interact with the SPI device from userspace).

RTFM
----
God loves people who rtfm. It's fun, healthy and besides, everybody's doing it.

* [Home page](http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en556614)
* [Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/22288A.pdf)
* [spidev](https://www.kernel.org/doc/Documentation/spi/spidev)


Building, installing, etc.
==========================

If you have your kernel sources installed and all is right with the world, building and installing is as simple as:

```
make all && make install
```

To explicitly say where your sources are, use:

```
KERNELDIR=/usr/scr/abcdefg make
```

Cross-Compiling
---------------

I'm not going to tell you how to setup a cross-compiling toolchain, but here is how I build for my Raspberry Pi.  Please note that the USB drivers for the Pi are currently problematic (especially prior to a commit late last month), so note the `MCP2210_QUIRKS=1` cpp variable.

```bash
#!/bin/bash

export KERNELDIR=/home/daniel/proj/kernel/rpi.3.10
export ARCH=arm
export CROSS_COMPILE=/usr/bin/armv6j-hardfloat-linux-gnueabi-
export CPPFLAGS="-DMCP2210_QUIRKS=1"

CFLAGS="-march=armv6j -mfpu=vfp -mfloat-abi=hard" make -j4 -C user &&
make -j4 "$@" &&
make mcp2210.s &&
scp -p mcp2210.ko user/libmcp2210.so user/mcp2210 root@pi:bin/
```

At the very least, your KERNELDIR should have a valid .config and you should run make modules_prepare.  You'll need to enable the following in your .config:

* CONFIG_SPI - if you want to use the MCP2210 as an spi master
* CONFIG_SPI_SPIDEV - if you want to use the kernel's handy-dandy spidev driver and the userspace utility's spi functionality
* CONFIG_GPIOLIB - if you want to use the gpio interface which doesn't exist yet :)

You don't need to make `mcp2210.s` if you don't care to examine the disassembly. The `CFLAGS` supplied when building the userspace program aren't inferred if not supplied (although it really doesn't contain any floating point calculations at the moment).  I use `-j4` because I have a quad core processor, tune to your preferences.  Finally, there is a lot of hard-coded crap in the `Makefile` (The `Kconfig` only exists for future integration into the mainline kernel tree).

Configuration and Setup
=======================

Between the driver & userland utility, there are two mechanisms for configuring your MCP2210 board.

Userland
--------
Basically, you run `user/mcp2210 set config <mask>`

For mask, you OR together the values for the configuration option(s) you want to set:

<table>
<tr><th>Bit</th><th>Option</th><th>Section</th></tr>
<tr><td>1</td><td>chip settings (current)</td><td>3.2.4</td></tr>
<tr><td>2</td><td>chip settings (power-up)</td><td>3.1.1</td></tr>
<tr><td>4</td><td>spi transfer settings (current)</td><td>3.2.2</td></tr>
<tr><td>8</td><td>spi transfer settings (power-up)</td><td>3.1.2</td></tr>
<tr><td>16</td><td>key parameters (power-up)</td><td>3.1.3</td></tr>
<tr><td>31</td><td>board config</td><td>n/a</td></tr>
</table>

"But where the hell do you specify the settings!?" I hear you ask.  Yeah, well currently, you have to modify [`user/settings.h`](blob/master/user/mcp2210-user.c) and recompile.  If you don't like it, then please write an xml / json / something interface and send me a patch.  This is pretty straight forward, consisting of simple static const structs initialized with named (C99) initializers.

WARNING: Modifying `nvram_access_control` can permanently lock you chip!  Don't change it unless you really mean to do this and understand what you're doing.

Creek (Auto-Configure)
----------------------

So this driver contains support for an auto-configuration scheme called Creek. It works by reading the power-up chip settings (section 3.1.1) and the power-up spi transfer settings (section 3.1.2) that the MCP2210 stores in its internal EEPROM and then reading the user area of its EEPROM (section 3.3.1) to obtain wiring and other configuration information.  Since this area is only 256 bytes, it uses a compression/encoding scheme dubbed Creek.

If Creek support is enabled (via CONFIG_MCP2210_CREEK), then at probe time, the first four bytes of the user EEPROM area are read to see if they match a magic number.  If it does, the remainder is read and decompressed into a `struct mcp2210_board_info` object which contains all of the information (timings for each SPI device, name of the spi protocol driver, label, etc.) to allow probing the spi_master.

TODO: All support for creating this encoding from you `struct mcp2210_board_config` in `user/settings.h` is in the userspace utility program and will eventually get documented here.

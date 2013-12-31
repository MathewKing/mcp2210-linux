MCP2210 driver for Linux
========================

This is a Linux device driver for the [Microchip mcp2210 USB-to-SPI bridge](http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en556614).  It currently builds as an out-of-tree module and includes a userland utility for testing, configuration, control and spi messaging via spidev. (There are other userland utilities for doing spi through [spidev](https://www.kernel.org/doc/Documentation/spi/spidev)).

This is a USB interface driver, NOT a usbhid driver!  The MCP2210 isn't a device that interfaces with humans, and they only choose the HID interface to avoid the hassle of writing and maintaining drivers for every other platform. However, it means that you use this whole layer of software, protocols, drivers, etc that you don't really need.  Therefore, this driver skips that layer and communicates directly with the device via standard interrupt URBs (not hid reports).

Unlike other libraries, this driver connects the SPI devices directly to the spi sub-system, allowing you to choose whatever spi protocol driver you wish, with the default being spidev (which allows you to interact with the SPI device from userspace).

RTFM
----
God loves people who rtfm. It's fun, healthy and besides, everybody's doing it.

* [MCP2210 Product page](http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en556614)
* [MCP2210 Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/22288A.pdf)
* [spidev](https://www.kernel.org/doc/Documentation/spi/spidev)


Configuring, Building and Installing
====================================

If you have your kernel sources installed and all is right with the world, building and installing (with a default configuration) is as simple as:

```
make all && make install
```

To explicitly say where your sources are, use:

```
KERNELDIR=/usr/scr/abcdefg make
```

This is an out-of-tree module, so if you don't fancy the default configuration, run `make out-of-tree-autoconf.h` to have it generated from the [template](out-of-tree-autoconf.h.template) and edit to your needs.  You can refer to [`Kconfig`](Kconfig) for descriptions of the various config options, but there is currently no mechanism to use the `Kconfig` for a `make menuconfig`; the `Kconfig` exists primarily for mainline integration.

Note that at this time, there is no `install` target for the userland utilities and to run `mcp2210-util`, you must explicitly specify an `LD_LIBRARY_PATH`. Example:

```bash
/home/daniel/proj/mcp2210$ make -j4
.... make output.....
/home/daniel/proj/mcp2210$ LD_LIBRARY_PATH=`pwd`/user user/mcp2210-util --help
```

Cross-Compiling
---------------

I'm not going to tell you how to setup a cross-compiling toolchain, but here is how I build for my Raspberry Pi.  Please note that the USB drivers for the Pi are currently problematic, especially prior the commit "[USB fix using a FIQ to implement split transactions](https://github.com/raspberrypi/linux/commit/db4fad7380c83b6e1b62dfb3fd61e55d031a04fc)", which has greatly improved things. So the below `MCP2210_QUIRKS=1` cpp variable should be omitted if you aren't using a pi.

```bash
#!/bin/bash

export KERNELDIR=/home/daniel/proj/kernel/rpi.3.10
export ARCH=arm
export CROSS_COMPILE=/usr/bin/armv6j-hardfloat-linux-gnueabi-
export CPPFLAGS="-DMCP2210_QUIRKS=1"

CFLAGS="-O2 -g3 -pipe -march=armv6j -mfpu=vfp -mfloat-abi=hard" make -j4 -C user &&
make -j4 "$@" &&
make mcp2210.s &&
scp -p mcp2210.ko user/libmcp2210.so user/mcp2210 root@pi:bin/
```

At the very least, your `KERNELDIR` should have a valid `.config` and you should run `make modules_prepare`.  You'll need to enable the following in your .config (either modules or built-ins will do):

* `CONFIG_SPI` - if you want to use the MCP2210 as an spi master
* `CONFIG_SPI_SPIDEV` - if you want to use the kernel's handy-dandy spidev driver and the userspace utility's spi functionality.  If you will only be using spi protocol drivers specific to your peripherals, then you do not need this option.
* `CONFIG_GPIOLIB` - if you want to use the gpio interface or use gpios for chip selects (as opposed to Microchip's mechanism, which is faster if it serves your needs)

You don't need to make `mcp2210.s` if you don't care to examine the disassembly. The `CFLAGS` supplied when building the userspace program aren't inferred if not supplied (although it really doesn't contain any floating point calculations at the moment).  I use `-j4` because I have a quad core processor, tune to your preferences.

Installing the Driver
=====================
Aside from the usual modprobe/insmod, this part is unfortunately a pain in the ass. This is because the hid-generic driver uses these values to select its devices:

```
static const struct hid_device_id hid_table[] = {
    { HID_DEVICE(HID_BUS_ANY, HID_GROUP_GENERIC, HID_ANY_ID, HID_ANY_ID) },
    { }
};
```

This prevents the mcp2210 driver from being selected as a candidate, even if the vid/pid explicitly match.  Currently, the work-around is to run the script [`user/mcp2210_bind.sh`](user/mcp2210_bind.sh) as root (you must have sysfs mounted).  This uses sysfs files to tell the usbhid driver to unbind from the mcp2210 device so that the mcp2210 driver can probe it.  If you know of a way to do this via udev rules, please notify me! (just create an issue via the issue tracker).

Configuration & Setup
=====================
As you may be aware, SPI does not offer a mechanism to automatically detect and configure peripherals. Typically, a particular chip is hard-wired to an SPI master and the driver for that master knows whats conntect to it.  In Linux, SPI slave devices are configred via [`struct spi_board_info`](https://www.kernel.org/doc/htmldocs/device-drivers/API-struct-spi-board-info.html) objects.  But for USB-to-SPI protocol bridges like the MCP2210, we can't know how its board is wired -- we need that information to come from somewhere else.

The MCP2210 has an embedded EEPROM (most likely 512 bytes in size), part of which it uses for its own internal settings and 256 bytes which it exposes as "user-EEPROM". In its internal settings (3.1.1 through 3.1.11 in the datasheet), the MCP2210 stores its inital "power up" values for all parameters:

* Mode of each pin (GPIO, SPI or dedicated)
* Direction and value (for outputs) of each GPIO
* The idle and active values of all chip select lines (only affects pins configured as SPI)
* SPI mode, bitrate, and timing parameters
* Some access control settings (for changing the persistent settings)
* USB power control: self or host powered, requested power
* USB vid, pid, mfg name, product name, etc.

While this is a pleasntly comprehensive feature set, it is still not enough to fully configure & probe any board wiring because:

1. We only have transfer settings for a single SPI device.
2. We still aren't telling the USB host what chip is connected to each chip select.

The mcp2210 driver offers two separate mechanisms to provide that information and configure the device: one from userland and an auto-configuration mechanism that utilizes the user area of the on-chip EEPROM.

Configuring from Userland (Manual)
----------------------------------
When ioctl control is enabled (`CONFIG_MCP2210_IOCTL`), the mcp2210 driver will create a `/dev` node upon a successful probe with the name `spi2usb_bridge<x>` (where x is a number) which can be used to interact with the driver. Userspace configuration is fairly straight-forward:

```
user/mcp2210-util set config <mask>
```

For mask, you OR together the values for the configuration option(s) you want to set:

<table>
<tr>
	<th>Bit</th>
	<th>Option</th>
	<th>Datasheet Section</th>
	<th>struct</th>
</tr><tr>
	<td>1</td>
	<td>chip settings (current)</td>
	<td>3.2.4</td>
	<td><tt><a href="mcp2210.h#L408">struct mcp2210_chip_settings</a></tt></td>
</tr><tr>
	<td>2</td>
	<td>chip settings (power-up)</td>
	<td>3.1.1</td>
	<td><tt><a href="mcp2210.h#L408">struct mcp2210_chip_settings</a></tt></td>
</tr><tr>
	<td>4</td>
	<td>spi transfer settings (current)</td>
	<td>3.2.2</td>
	<td><tt><a href="mcp2210.h#L441">struct mcp2210_spi_xfer_settings</a></tt></td>
</tr><tr>
	<td>8</td>
	<td>spi transfer settings (power-up)</td>
	<td>3.1.2</td>
	<td><tt><a href="mcp2210.h#L441">struct mcp2210_spi_xfer_settings</a></tt></td>
</tr><tr>
	<td>16</td>
	<td>key parameters (power-up)</td>
	<td>3.1.3</td>
	<td><tt><a href="mcp2210.h#L453">struct mcp2210_usb_key_params</a></tt></td>
</tr><tr>
	<td>32</td>
	<td>board config</td>
	<td>n/a</td>
	<td><tt><a href="mcp2210.h#L546">struct mcp2210_board_config</a></tt></td>
</tr><tr>
</table>

"But where the hell do you specify the settings!?" I hear you ask.  Yeah, well currently, you have to modify `user/settings.h`, which is auto-generated from [`user/settings-example.h`](user/settings-example.h) the first time you run make and recompile.  (If you don't like this, then please write an xml / json / something interface and send me a patch or pull request!)  This is pretty straight forward, consisting of simple static const structs initialized with named (C99) initializers.

WARNING: Modifying `nvram_access_control` can permanently lock you chip!  Don't change it unless you really mean to do this and understand what you're doing.

Automatic Configuration
-----------------------
When the driver probes, it queries the device for (among other things) its power-up chip settings (section 3.1.1) and the power-up spi transfer settings (section 3.1.2). If Creek support is enabled (via `CONFIG_MCP2210_CREEK`), the first four bytes of the user EEPROM area are also read. If these match a "magic number", then the remainder of the user-EEPROM is read and decoded into a [`struct mcp2210_board_config`](mcp2210.h#L546) object which contains all of the information (timings for each SPI device, name of the spi protocol driver, label, etc.) to allow probing the spi_master.

For details on the encoding format, see the comments in [`mcp2210-creek.h`](mcp2210-creek.h#L29) (see also [`creek_encode()`](mcp2210-lib.c#L618) and [`creek_decode()`](mcp2210-lib.c#L373))

NOTICE: At this time, the Creek binary format is unstable and subject to change without a version bump.

Storing and Viewing Auto-Configure Data
---------------------------------------
All support for creating this encoding from your [`struct mcp2210_board_config`](mcp2210.h#L546) in `user/settings.h` is in the userspace utility program.

1. Edit `user/settings.h` to your needs and recomple `mcp2210-util`
2. Run the following command to encode to config.dat:

```
mcp2210-util encode > config.dat
```

3. `ls -l config.dat` to determine its size (yes, the utility should eventually do this for you)
4. Run the following command to store it to the user-EEPROM (replace size with the size of the file)

```
mcp2210-util eeprom write size=<size> addr=0 < config.dat
```

Decoding is the inverse.  However, the creek data doesn't include any data available in the "chip settings" (because this would be a redundant waste of limited space) so your chip settings in settings.h (or at least the pin modes) must match what was used when the encoding was generated.

1. Make sure my_chip_settings in `user/settings.h` is correct.
2. Run the following command:

```
mcp2210-util decode < config.dat
```


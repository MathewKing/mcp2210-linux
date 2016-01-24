/*
 *  MCP2210 driver
 *
 *  Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
 *                2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 * TODO:
 * Incomplete list:
 * - major number from maintainer?
 * - honor cs_gpio and allow SPI where idle_cs == active_cs
 * - missing mechanism to notify external entities of changes to gpio and
 *   the dedicated "interrupt counter" line.
 * - honor cs_change
 *
 * Fix list:
 * - While we wont see it on x86 & little-endian arm, the flipping
 *   ctl_complete_urb() may be broken when processing data from the request
 *   buffer.
 * - Using the packed version of protocol structs is bloating code horribly,
 *   even worse than I had guessed it would!! So we need to have packed and
 *   unpacked versions of these structs replace the is_mcp_endianness and
 *   "flipping" functionality with conversions from packed (protocol-correct)
 *   little-endian structs to unpacked native-endianness structs.
 * - When a command dies and can't be retried, we need to do something more
 *   than just log it.  Re-implement device failure?
 * - Change ioctl to netlink.
 * - ioctl interface not friendly with ABIs where kernel & user space have
 *   different sized ptrs (x32, sparc64, etc.) (Maybe fix this when we convert
 *   to netlink)
 * - obey struct spi_transfer cs_change
 *
 * Tweak list:
 * - examine locking and possibly (hopefully) refine
 * - better prediction of how long SPI transfers will take and appropriate
 *   scheduling so that we don't unnecessarily request a status (update: this
 *   has some stuff now, but it may need an audit)
 * - include/linux/spi/spi.h: extend mode (to u16), spi_device, et. al. to
 *   accommodative missing features (timing, drop cs between words, etc.) (note
 *   current work with DUAL and QUAD on linux-spi)
 *   - update spidev driver & userspace to support
 *   - update mcp2210 driver to use them.
 *
 * @file
 *
 * Product page:
 *    https://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en556614
 * Datasheet:
 *    https://www.microchip.com/downloads/en/DeviceDoc/22288A.pdf
 *
 * Background:
 *
 * The MCP2210 is a USB-to-SPI protocol converter with GPIO by Microchip.  It
 * uses a basic 64-byte request/response protocol for all operations, advertises
 * its self as a generic HID device and can be used via the hid-generic driver
 * from userspace using the hidraw /dev node.  A more friendly interface is
 * available via Kerry Wong's MCP2210-Library
 * (https://github.com/kerrydwong/MCP2210-Library) which is built on top of
 * Signal 11's hidapi library (http://www.signal11.us/oss/hidapi) which in turn
 * uses libusb.  All together, it gives you a pretty user-friendly userland
 * interface to to the device, but carries the following drawbacks:
 *
 * - Extensive overhead: hid-core driver -> usbhid driver -> (hid-generic
 *   driver) -> libusb -> hidapi -> MCP2210-Library means lots of memory & CPU.
 *   (Note however that hid-generic is virtually non-existent.) Additionally,
 *   an embedded system will have to enable the input and HID layers that it
 *   otherwise may not require.
 * - No way to use SPI protocol drivers to manage peripherals on the other side
 *   of the MCP2210.
 * - No mechanism for auto-configuration outside of userland.
 *
 * This driver attempts to solve these shortcomings by eliminating all of these
 * layers, including the kernel's input, hid, hid-core, usbhid, etc.,
 * communicating with the MCP2210 directly via interrupt URBs and allowing your
 * choice of spi protocol driver for each SPI device, with the standard option
 * of spidev for interacting with the SPI device from userspace.  In addition,
 * it supplies an (optional) auto-configuration scheme which utilizes the
 * devices 256 bytes of user EEPROM to store the wiring information for the
 * board and a fairly complete ioctl interface for userland configuration,
 * testing and such.
 *
 * Theory of Operation:
 * ===================
 *
 * Commands and The Command Queue
 * ------------------------------
 * The MCP2210 driver is a queue-driven USB interface driver.  All commands are
 * executed (or given the opportunity) serially, in FIFO order.  Only one
 * command may execute at a time.  Each command represents a logically atomic
 * operation (e.g., send SPI transfer or query settings) and typically exchanges
 * at least one message with the device via interrupt URBs.
 *
 * Commands are objects of either struct mcp2210_cmd or a derived-type using
 * pseudo-inheritance -- embedding a struct mcp2210_cmd object as its first
 * member and using a type field to specify the command type.  There are three
 * different types of "normal" commands that interact with the device directly.
 * Each populate the cmd->type with a pointer to an appropriate struct
 * mcp2210_cmd_type object:
 *
 * 1. Transfer SPI messages (mcp2210-spi.c)
 * 2. Read/write to/from the device's user-EEPROM area (mcp2210-eeprom.c)
 * 3. Query and change settings (mcp2210-ctl.c)
 *
 * General-purpose commands have have a NULL command type, but will populate
 * cmd->complete and cmd->context to defer some work, possibly to execute in a
 * non-atomic context.  Currently, these are only used to probe spi and gpio,
 * neither of which can be probed at the time the USB interface is probed
 * because the information to do so is not yet available.  This occurs either
 * when the user-EEPROM area has been read and decoded (when
 * CONFIG_MCP2210_CREEK is enabled) or when the configure ioctl command is
 * called from userspace.
 *
 * Delayed & Non-Atomic Commands
 * -----------------------------
 * Due to the nature of the device, having:
 *
 * 1. a single USB interface,
 * 2. potentially multiple SPI peripherals connected to it,
 * 3. the need to poll its gpio values & interrupt counter, and
 * 4. late arrival of wiring information to perform a complete probe (with SPI
 *    & GPIO),
 *
 * the driver requires a slightly more sophisticated mechanism for processing
 * commands than a simple FIFO queue processed by URB completion handlers.
 * Polling commands need to be delayed.  SPI Transfers (especially on slower
 * chips) often need to have delays between URBs to give the device time to
 * complete transfers to the chip.  Performing the configure (probing spi_master
 * and gpio_chip) must occur in a context that can sleep.  If a command needs to
 * be delayed, its cmd->delayed bit is set and cmd->delay_until will specify a
 * time (in jiffies) that the command is to run at.  If the command must be able
 * to sleep, it's cmd->nonatomic bit will be set.
 *
 * Note that there is no mechanism (at this time) to assure that a delayed
 * command will execute at its requested time, although it is guaranteed not
 * to execute prior to it.  Delayed commands who's cmd->delay_until time has
 * been reached still have to wait their turn in the queue to be eligible for
 * execution.  If a delayed command is popped from the queue but its
 * cmd->delay_until time has not yet arrived, it is moved to the
 * dev->delayed_list and an appropriate mechanism (either timer or
 * delayed_work) is set to pick it up.  If another command is executing when
 * the timer expires or delayed_work runs, the delayed command is (effectively)
 * moved to the head of the queue and will execute next.
 *
 * process_commands():
 * ------------------
 * Messages in the queue are processed via process_commands().  If dev->cur_cmd
 * is NULL, it will attempt to retrieve a command from the queue, marking its
 * state as new.  How it processes commands differs between "normal" commands
 * (which interact with the MCP2210) and general-purpose commands (which are
 * simply some delayed work)
 *
 * -- Normal Commands
 * If there is a normal command in the new state, submit_urbs() is called and
 * its state is set to "submitted".  Completion handlers for the in and out URBs
 * will eventually be called (if the usb host driver behaves) marking the
 * command as "completed" and calling process_commands() to have the completed
 * command finalized.  Finalization is performed by calling struct mcp2210_cmd's
 * complete function (if non-NULL) and freeing it.  Then, the next message is
 * retrieved, and so forth.
 *
 * -- General-Purpose Commands
 * For general-purpose commands, submit/complete_urbs() is bypassed and it is
 * instead immediately marked as completed having its complete() function
 * called, allowing any arbitrary work to be performed.
 *
 * -- Deferred Work
 * A command may be deferred because:
 * a. it needs to wait before starting,
 * c. it needs to run again, but needs to wait before doing so, or
 * b. it needs to run in a non-atomic state.
 *
 * submit_urbs():
 * -------------
 * This function will first call the command type's submit_prepare() function,
 * which should perform any needed initialization, populate the 64-byte request
 * message and return non-zero unless it has an error.  However, there are two
 * exceptions to this: submit_prepare() may return -ERESTARTSYS to have the
 * command deleted w/o further processing or -EAGAIN to have the message
 * restarted and submit_urbs() called again).  It will then submit the URBS and
 * change the command's state to "submitted".
 *
 * complete_urb():
 * This function manages both the in & out URBs.
 *
 * Once both URBs have completed,
 * the command type's complete_urb() function is called and the command's
 * state is set to "completed".
 *
#if 0
out of date info:
 * When the in URB completes, complete_urb_in() performs the following:
 * - basic sanity checks & URB status via check_urb_status()
 * - sets command status to "sent"
 * - calls the command type's complete_tx() function (if it has one)
 *
 * complete_urb_out():
 * Similarly, when the out URB completes, complete_urb_out() performs
 * the following:
 * - calls check_urb_status() and exits upon error
 * - checks for and handles out-of-order responses
 * - sets the command status to "done"
 * - checks the status field and general validity of the response message via
 *   check_response().
 * - calls complete_rx()
 * - if complete_rx() returns -EAGAIN, the command's state is reset to "new" so
 *   that the next call to process_commands will run it again.
 * - calls process_commands() (in GFP_ATOMIC mode) to process the next command.
#endif
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
//#include <linux/completion.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"

#ifdef CONFIG_MCP2210_CREEK
# include "mcp2210-creek.h"
#endif /* CONFIG_MCP2210_CREEK */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
# define HAVE_USB_ALLOC_COHERENT 1
#endif

static void mcp2210_delete(struct kref *kref);
static int mcp2210_probe(struct usb_interface *intf,
			 const struct usb_device_id *id);
static void mcp2210_disconnect(struct usb_interface *intf);

static int report_status_error(struct mcp2210_device *dev, u8 status);
static bool reschedule_delayed_work(struct mcp2210_device *dev,
				    unsigned long _jiffies);
static inline bool reschedule_delayed_work_ms(struct mcp2210_device *dev,
					      unsigned int ms);
static void delayed_work_callback(struct work_struct *work);
static void timer_callback(unsigned long context);

static int unlink_urbs(struct mcp2210_device *dev);
static void kill_urbs(struct mcp2210_device *dev, unsigned long *irqflags);

static void complete_urb(struct urb *urb);
static int submit_urbs(struct mcp2210_cmd *cmd, gfp_t gfp_flags);

struct mcp2210_cmd_type mcp2210_cmd_types[MCP2210_CMD_TYPE_MAX];

/******************************************************************************
 * Module parameters
 */

int debug_level	  = CONFIG_MCP2210_DEBUG_INITIAL;
int creek_enabled = IS_ENABLED(CONFIG_MCP2210_CREEK);
int dump_urbs	  = IS_ENABLED(CONFIG_MCP2210_DEBUG);
int dump_commands = IS_ENABLED(CONFIG_MCP2210_DEBUG_VERBOSE);
uint pending_bytes_wait_threshold = 45;

module_param(debug_level,	int, 0664);
module_param(creek_enabled,	int, 0664);
module_param(dump_urbs,		int, 0664);
module_param(dump_commands,	int, 0664);
module_param(pending_bytes_wait_threshold, uint, 0664);

/******************************************************************************
 * USB Driver structs
 */

static const struct usb_device_id mcp2210_devices[] = {
    { USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MCP2210)},
    { }
};

static struct usb_driver mcp2210_driver = {
	.name			= "mcp2210",
	.probe			= mcp2210_probe,
	.disconnect		= mcp2210_disconnect,
	.suspend		= NULL,
	.resume			= NULL,
	.reset_resume		= NULL,
	.id_table		= mcp2210_devices,
	.supports_autosuspend	= IS_ENABLED(CONFIG_MCP2210_AUTOPM),
};

/******************************************************************************
 * Module functions
 */

static int __init mcp2210_init(void)
{
	int ret;
	printk("mcp2210_init\n");
	msg_validate_size();

	ret = usb_register_driver(&mcp2210_driver, THIS_MODULE, "mcp2210");
	if (ret)
		printk(KERN_ERR "can't register mcp2210 driver\n");

	return ret;
}

static void __exit mcp2210_exit(void)
{
    printk("mcp2210_exit\n");
    usb_deregister(&mcp2210_driver);
}

module_init(mcp2210_init);
module_exit(mcp2210_exit);
MODULE_AUTHOR("Daniel Santos <daniel.santos@pobox.com>");
MODULE_DESCRIPTION("Microchip MCP2210 USB-to-SPI protocol converter with GPIO");
MODULE_LICENSE("GPL");

/* TODO: I have no idea what this should say, this shouldn't be a "platform:"
 * device if I understand correctly */
MODULE_ALIAS("mcp2210");

/******************************************************************************
 * USB Class Driver & file ops
 */

static inline struct mcp2210_device *mcp2210_kref_to_dev(struct kref *kref)
{
	return container_of(kref, struct mcp2210_device, kref);
}

#ifdef CONFIG_MCP2210_IOCTL
static int mcp2210_open(struct inode *inode, struct file *file)
{
	struct mcp2210_device *dev;
	struct usb_interface *intf;
	int subminor = iminor(inode);
	int ret;

	if (!(intf = usb_find_interface(&mcp2210_driver, subminor))) {
		pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		return -ENODEV;
	}

	if (!(dev = usb_get_intfdata(intf)))
		return -ENODEV;

	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM)) {
		ret = usb_autopm_get_interface(intf);
		if (ret && ret != -EACCES) {
			mcp2210_err("usb_autopm_get_interface() failed:%de", ret);
			return ret;
		}
	}

	kref_get(&dev->kref);
	file->private_data = dev;

	return 0;
}

static int mcp2210_release(struct inode *inode, struct file *file)
{
	struct mcp2210_device *dev = file->private_data;

	if (!dev)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM) && dev->intf)
		usb_autopm_put_interface(dev->intf);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, mcp2210_delete);
	return 0;
}

static int mcp2210_flush (struct file *file, fl_owner_t id)
{
	return 0;
}

static loff_t mcp2210_no_llseek (struct file *file, loff_t offset, int i)
{
	return -EPERM;
}

static ssize_t mcp2210_no_read_write (struct file *file,
				      const char __user *data, size_t size,
				      loff_t *offset)
{
	return -EPERM;
}

const struct file_operations mcp2210_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	mcp2210_ioctl,
	.compat_ioctl =		mcp2210_ioctl,
	.llseek =		mcp2210_no_llseek,
	.read =			(void*)mcp2210_no_read_write,
	.write =		mcp2210_no_read_write,
	.open =			mcp2210_open,
	.flush =		mcp2210_flush,
	.release =		mcp2210_release,
};

static struct usb_class_driver mcp2210_class = {
	.name =		"usb2spi_bridge%d",
	.fops =		&mcp2210_fops,
	.minor_base =	0, /* FIXME: need a minor base from USB maintainer? */
};

#endif /* CONFIG_MCP2210_IOCTL */

/******************************************************************************
 * USB Driver functions
 */

static inline void reset_endpoint(struct mcp2210_endpoint *ep)
{
	ep->state = MCP2210_STATE_NEW;
	ep->kill = 0;
}

/* init_endpoint - Initialize an endpoint proxy struct
 *
 * may sleep
 *
 * This is only called from one place now but the code is cleaner with it
 * separated out in another function, so I'm marking inlining it for the
 * possible reduction in size.  (It's only called in probe, so performance isn't
 * a big deal here.)
 */
static __always_inline int init_endpoint(struct mcp2210_device *dev,
					 const struct usb_host_endpoint *ep)
{
	int is_dir_in = !!usb_endpoint_dir_in(&ep->desc);
	struct mcp2210_endpoint *dest = &dev->eps[is_dir_in];
	unsigned int pipe;
	struct urb *urb;

	if (is_dir_in)
		pipe = usb_rcvintpipe(dev->udev, ep->desc.bEndpointAddress);
	else
		pipe = usb_sndintpipe(dev->udev, ep->desc.bEndpointAddress);

	/* just another struct size sanity check */
	BUILD_BUG_ON(sizeof(*dest->buffer) != 64);

	if (unlikely(dest->ep)) {
		mcp2210_warn("Unexpected: more than one interrupt %s endpoint "
			     "discovered, ingoring them\n",
			     urb_dir_str[is_dir_in]);
		return 0;
	}

	dest->is_dir_in = is_dir_in;
	dest->ep = ep;
	reset_endpoint(dest);

	 //= ATOMIC_INIT(0);
	atomic_set(&dest->unlink_in_process, 1);
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (unlikely(!urb)) {
		mcp2210_err("Failed to alloc %s URB\n", urb_dir_str[is_dir_in]);
		return -ENOMEM;
	}

	/* If we have usb_alloc_coherent, we'll use that.  Otherwise, we'll let
	 * the usb host controller driver allocate a DMA buffer if it needs one
	 */
#ifdef HAVE_USB_ALLOC_COHERENT
	dest->buffer = usb_alloc_coherent(dev->udev, 64, GFP_KERNEL,
					  &urb->transfer_dma);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
#else
	dest->buffer = kzalloc(64, GFP_KERNEL);
#endif
	if (unlikely(!dest->buffer)) {
		usb_free_urb(urb);
		mcp2210_err("Failed to alloc %s URB DMA buffer\n",
			    urb_dir_str[is_dir_in]);
		return -ENOMEM;
	}

	dest->urb = urb;
	usb_fill_int_urb(urb,
			dev->udev,
			pipe,
			dest->buffer,
			64,
			complete_urb,
			dev,
			ep->desc.bInterval);
	/* well, this isn't really helpful, but usb_host_endpoint ptr will
	 * eventually replace pipe and usb_submit_urb() will have to
	 * populate this later anyway */
	urb->ep = (struct usb_host_endpoint*)ep;

#if 0
	/* apparently, this isn't reliable on all host controllers, so we'll
	 * just check acutal_length ourselves */
	if (is_dir_in)
		urb->transfer_flags |= URB_SHORT_NOT_OK;
#endif

	return 0;
}

static void mcp2210_delete(struct kref *kref)
{
	struct mcp2210_device *dev = mcp2210_kref_to_dev(kref);
	struct mcp2210_endpoint *ep;

	for (ep = dev->eps; ep < &dev->eps[2]; ++ep) {
		if (ep->urb) {
			if (ep->buffer)
#ifdef HAVE_USB_ALLOC_COHERENT
				usb_free_coherent(dev->udev, 64, ep->buffer,
						  ep->urb->transfer_dma);
#else
				kfree(ep->buffer);
#endif

			usb_free_urb(ep->urb);
		}
	}

	if (dev->config)
		kfree(dev->config);

	kfree(dev);
}

#ifdef CONFIG_MCP2210_CREEK

static int creek_configure(struct mcp2210_cmd *cmd, void *context) {
	struct mcp2210_device *dev = cmd->dev;
	struct mcp2210_board_config *board_config;
	int ret;

	board_config = mcp2210_creek_probe(dev, GFP_KERNEL);

	BUG_ON(dev->config);
	if (IS_ERR(board_config)) {
		mcp2210_err("Failed to decode board config from MCP2210's user-EEPROM: %de", (int)PTR_ERR(board_config));
		return 0;
	}

	ret = mcp2210_configure(dev, board_config);

	/*
	spin_lock_irqsave(&dev->eeprom_spinlock, irqflags);
	magic = le32_to_cpu(*((u32*)dev->eeprom_cache));
	spin_unlock_irqrestore(&dev->eeprom_spinlock, irqflags);
	*/
	return 0;
}

static int eeprom_read_complete(struct mcp2210_cmd *cmd_head, void *context)
{
	struct mcp2210_device *dev = context;
	struct mcp2210_cmd_eeprom *cmd = (void *)cmd_head;
	unsigned long irqflags;
	int ret;

	mcp2210_debug();

	BUG_ON(!cmd);
	BUG_ON(!dev);
	BUG_ON(!cmd_head->type);
	BUG_ON(cmd_head->type->id != MCP2210_CMD_TYPE_EEPROM);

	/* deal with fuck-ups */
	if (cmd->head.status) {
		mcp2210_err("RPi USB otg drivers are shit, trying "
			    "a-fucking-gain...");

		ret = mcp2210_eeprom_read(dev, NULL, 0, cmd->size,
					  eeprom_read_complete, dev,
					  GFP_ATOMIC);

		if (ret && ret != -EINPROGRESS)
			mcp2210_err("Adding eeprom command failed with %de, fuck it", ret);

		return 0;
	}

	/* We can get 0xfa (write failure) for writing to EEPROM, but we
	 * supposedly can't get a failure for reading, so we pertty much should
	 * always have a zero for the mcpstatus
	 */
	if (cmd->head.mcp_status) {
		mcp2210_err("Unexpected failure of EEPROM read: 0x%02x",
			    cmd->head.mcp_status);
		return 0;
	}

	if (cmd->size == 4) {
		u8 magic[4];

		spin_lock_irqsave(&dev->eeprom_spinlock, irqflags);
		memcpy(magic, dev->eeprom_cache, 4);
		//magic = le32_to_cpu(*((u32*)dev->eeprom_cache));
		spin_unlock_irqrestore(&dev->eeprom_spinlock, irqflags);

		if (memcmp(magic, CREEK_CONFIG_MAGIC, 4)) {
			mcp2210_notice("creek magic not detected: 0x%08x != "
				       "0x%08x", *(u32*)magic, *(u32*)CREEK_CONFIG_MAGIC);
			return 0;
		}

		mcp2210_notice("creek magic detected, reading config from EEPROM");
		ret = mcp2210_eeprom_read(dev, NULL, 0, 0x100, eeprom_read_complete, dev, GFP_ATOMIC);

		if (ret && ret != -EINPROGRESS)
			mcp2210_err("Adding eeprom command failed with %de",
				    ret);

		return 0;
	} else if (cmd->size == 0x100) {
		struct mcp2210_cmd *cmd;

		if (!dev->s.have_power_up_chip_settings) {
			mcp2210_err("Don't have power up chip settings, aborting probe... :(");
			return 0;
		}

		cmd = mcp2210_alloc_cmd(dev, NULL, sizeof(struct mcp2210_cmd), GFP_ATOMIC);
		if (!cmd) {
			mcp2210_err("mcp2210_alloc_cmd failed (ENOMEM)");
			return 0;
		}

		cmd->complete = creek_configure;
		cmd->nonatomic = 1;

		ret = mcp2210_add_cmd(cmd, true);
		if (ret)
			return 0;
	}

#if 0

	if (ret && ret != -EINPROGRESS) {
		mcp2210_err("Adding eeprom command failed with %de", ret);
		goto error2;
	}
	CREEK_CONFIG_MAGIC
	le32_to_cpu(creek_get_bits(&bs, 32));
	/* HACK: This is a hack for when URBs fail on the rpi, since we're not
	 * really even using these yet */
	dev->have_power_up_chip_settings = 1;
	dev->have_power_up_spi_settings = 1;
	dev->do_spi_probe = 1;
#endif
	return 0;
}
#else
static inline int eeprom_read_complete(struct mcp2210_cmd *cmd_head,
				       void *context)
{
	return 0;
}
#endif /* CONFIG_MCP2210_CREEK */

/* mcp2210_probe
 * can sleep, but keep it minimal as the USB core uses a single thread to probe
 * & remove all USB devices.
 */
int mcp2210_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *intf_desc = intf->cur_altsetting;
	struct mcp2210_device *dev;
	struct usb_host_endpoint *ep, *ep_end;
	int ret = -ENODEV;

	printk("mcp2210_probe\n");

	if (!udev)
		return -ENODEV;

	dev = kzalloc(sizeof(struct mcp2210_device), GFP_KERNEL);
	if (unlikely(!dev)) {
		mcp2210_err("failed to allocate struct usb_device (%u bytes)",
			    (unsigned)sizeof(struct mcp2210_device));
		return -ENOMEM;
	}

	dev->intf = intf;
	dev->udev = udev;
	kref_init(&dev->kref);
	INIT_LIST_HEAD(&dev->cmd_queue);
	INIT_LIST_HEAD(&dev->delayed_list);
	spin_lock_init(&dev->dev_spinlock);
	spin_lock_init(&dev->queue_spinlock);
#ifdef CONFIG_MCP2210_EEPROM
	spin_lock_init(&dev->eeprom_spinlock);
#endif
	mutex_init(&dev->io_mutex);
	ctl_cmd_init(dev, &dev->ctl_cmd, 0, 0, NULL, 0, false);
	INIT_DELAYED_WORK(&dev->delayed_work, delayed_work_callback);
	init_timer(&dev->timer);
	dev->timer.function = timer_callback;
	dev->timer.data = (unsigned long)dev;
#ifdef CONFIG_MCP2210_DEBUG
	atomic_set(&dev->manager_running, 0);
#endif

#ifdef CONFIG_MCP2210_GPIO
#endif

#ifdef CONFIG_MCP2210_SPI
	/* mark current spi config as uninitialized */
	dev->s.cur_spi_config = -1;
#endif

	/* Set up interrupt endpoint information. */
	ep_end = &intf_desc->endpoint[intf_desc->desc.bNumEndpoints];
	for (ep = intf_desc->endpoint; ep != ep_end; ++ep) {

		if (!usb_endpoint_xfer_int(&ep->desc))
			continue;

		if ((ret = init_endpoint(dev, ep)))
			goto error_kref_put;
	}

	if (unlikely(!dev->eps[EP_OUT].ep || !dev->eps[EP_IN].ep)) {
		mcp2210_err("could not find in and/or out interrupt endpoints");
		goto error_kref_put;
	}

	usb_set_intfdata(intf, dev);

	/* no sleeping until we're done with all of our probing */
	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM))
		usb_autopm_get_interface_no_resume(intf);

#ifdef CONFIG_MCP2210_IOCTL
	/* TODO: Do I need a "major number" from maintainer?
	 * https://www.kernel.org/doc/htmldocs/usb/API-usb-register-dev.html
	 */
	ret = usb_register_dev(intf, &mcp2210_class);
	if (unlikely(ret)) {
		mcp2210_err("failed to register device %de\n", ret);
		goto error_autopm_put;
	}
#endif /* CONFIG_MCP2210_IOCTL */

	/* TODO: set USB power to max until we know how much we need? */
	dump_dev(KERN_INFO, 0, "This is the initial device state: ", dev);

	/* Submit initial commands & queries.  First, we have to cancel any SPI
	 * messages that were started (and not finished) prior the host machine
	 * rebooting since the chip fails to do this upon USB reset. */
	if ((ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_SPI_CANCEL, 0,
				       NULL, 0, false, GFP_KERNEL))
	/* current chip configuration */
	 || (ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_GET_CHIP_CONFIG, 0,
				       NULL, 0, false, GFP_KERNEL))
	/* current SPI configuration */
	 || (ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_GET_SPI_CONFIG, 0,
				       NULL, 0, false, GFP_KERNEL))
	/* power-up chip configuration */
	 || (ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_GET_NVRAM,
				       MCP2210_NVRAM_SPI,
				       NULL, 0, false, GFP_KERNEL))
	/* power-up SPI configuration */
	 || (ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_GET_NVRAM,
				       MCP2210_NVRAM_CHIP,
				       NULL, 0, false, GFP_KERNEL))
	/* user key parameters */
	 || (ret = mcp2210_add_ctl_cmd(dev, MCP2210_CMD_GET_NVRAM,
				       MCP2210_NVRAM_KEY_PARAMS,
				       NULL, 0, false, GFP_KERNEL))) {
		mcp2210_err("Adding some command failed with %de", ret);
		goto error_deregister_dev;
	}

	if (IS_ENABLED(CONFIG_MCP2210_CREEK) && creek_enabled) {
		/* read the first 4 bytes to see if we have our magic number */
		ret = mcp2210_eeprom_read(dev, NULL, 0, 4, eeprom_read_complete, dev, GFP_KERNEL);
		if (ret && ret != -EINPROGRESS) {
			mcp2210_err("Adding eeprom command failed with %de", ret);
			goto error_deregister_dev;
		}
	}

	/* start up background cleanup thread */
	schedule_delayed_work(&dev->delayed_work, msecs_to_jiffies(1000));

	/* Submit the first command's URB */
	ret = process_commands(dev, false, true);

	if (ret < 0)
		goto error_deregister_dev;


	mcp2210_info("success");

	return 0;

error_deregister_dev:
	if (IS_ENABLED(CONFIG_MCP2210_IOCTL))
		usb_deregister_dev(intf, &mcp2210_class);

error_autopm_put:
	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM))
		usb_autopm_put_interface(intf);
	usb_set_intfdata(intf, NULL);

error_kref_put:
	kref_put(&dev->kref, mcp2210_delete);

	return ret;
}


#if 0
static void fail_device(struct mcp2210_device *dev, int error)
{
	mcp2210_err("failing mcp2210 with error %de", error);
	*((volatile int *)&dev->dead) = 1;
	unlink_urbs(dev);
	cancel_delayed_work(&dev->delayed_work);

	dev->cur_cmd->status = error;
}
#endif


/**
 * eats new_config
 */
int mcp2210_configure(struct mcp2210_device *dev, struct mcp2210_board_config *new_config)
{
	unsigned i;
	struct mcp2210_chip_settings chip_settings;
	int ret;

	might_sleep();

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		BUG_ON(dev->spi_master);
		BUG_ON(dev->s.is_gpio_probed);
		BUG_ON(dev->config);
		BUG_ON(!dev->s.have_chip_settings);
	}
	if (dev->config)
		mcp2210_err("ERROR: already have dev->config!");
	if (dev->spi_master)
		mcp2210_err("ERROR: already have dev->spi_master!");
	if (dev->s.is_gpio_probed)
		mcp2210_err("ERROR: gpio already probed");
	if (!dev->s.have_chip_settings)
		mcp2210_err("ERROR: don't have dev->s.have_chip_settings!");

	if (dev->config || dev->spi_master || dev->s.is_gpio_probed || !dev->s.have_chip_settings)
		return -EPERM;

	dev->s.cur_spi_config = -1;
	dev->s.have_config = 1;
	dev->config = new_config;

	memcpy(&chip_settings, &dev->s.chip_settings, sizeof(chip_settings));

	for (i = 0; i < MCP2210_NUM_PINS; ++i) {
		u8 mode = dev->config->pins[i].mode;

		chip_settings.pin_mode[i] = mode;
		dev->names[i] = dev->config->pins[i].name;
	}

	mcp2210_add_ctl_cmd(dev, MCP2210_CMD_SET_CHIP_CONFIG, 0, &chip_settings,
			    sizeof(chip_settings), false, GFP_KERNEL);

	if (IS_ENABLED(CONFIG_MCP2210_GPIO)) {
		mcp2210_info("----------probing GPIO----------\n");
		ret = mcp2210_gpio_probe(dev);
		if (ret) {
			mcp2210_err("gpio probe failed: %de", ret);
			return 0;
		}
	}

	if (IS_ENABLED(CONFIG_MCP2210_IRQ)) {
		mcp2210_info("----------probing IRQ controller----------\n");
		ret = mcp2210_irq_probe(dev);
		if (ret) {
			mcp2210_err("IRQ probe failed: %de", ret);
			return 0;
		}
	}

	if (IS_ENABLED(CONFIG_MCP2210_SPI)) {
		mcp2210_info("----------probing SPI----------\n");
		ret = mcp2210_spi_probe(dev);
		if (ret) {
			mcp2210_err("spi probe failed: %de", ret);
			return 0;
		}
	}

	if (!dev->cur_cmd)
		process_commands(dev, false, true);

	/* Allow the device to auto-sleep now */
	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM))
		usb_autopm_put_interface(dev->intf);

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		mcp2210_info("----------new dump----------\n");
		dump_dev(KERN_INFO, 0, "", dev);
	}

	mcp2210_notice("Configure successful");
	return 0;
}

// can sleep, like probe
void mcp2210_disconnect(struct usb_interface *intf)
{
	unsigned long irqflags;
	int ret;
	struct mcp2210_device *dev = usb_get_intfdata(intf);

	printk("mcp2210_disconnect\n");

	*((volatile int *)&dev->dead) = -ESHUTDOWN;
	spin_lock_irqsave(&dev->dev_spinlock, irqflags);
	cancel_delayed_work(&dev->delayed_work);
	del_timer(&dev->timer);
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);

	/* disable interrupt generation early */
	mcp2210_irq_remove(dev);

	/* unlink any URBs in route and wait for their completion handlers to be
	 * called */
	mcp2210_info("unlinking and killing all URBs...");
	kill_urbs(dev, NULL);


	mcp2210_info("emptying command queue...");
	spin_lock_irqsave(&dev->queue_spinlock, irqflags);
	while (!list_empty(&dev->cmd_queue)) {
		struct mcp2210_cmd *cmd = list_first_entry(&dev->cmd_queue,
							   struct mcp2210_cmd,
							   node);
		list_del(dev->cmd_queue.next);

		if (cmd->complete) {
			cmd->status = -ESHUTDOWN;
			spin_unlock(&dev->queue_spinlock);
			mcp2210_debug("calling completion handler for %p", cmd);
			ret = cmd->complete(cmd, cmd->context);
			mcp2210_debug("done");
			spin_lock(&dev->queue_spinlock);
		} else
			ret = 0;
		if (ret != -EINPROGRESS)
			kfree(cmd);
	};
	spin_unlock_irqrestore(&dev->queue_spinlock, irqflags);

	/* always let completion handler free dev->cur_cmd */


	/* TODO: interrupt (? taken care of by interrupting URBs?) any SPI
	 *       transfers & clear the queues */
	/* TODO: put USB port power back to whatever "normal" is */

#ifdef CONFIG_MCP2210_SPI
	if (dev->spi_master)
		mcp2210_spi_remove(dev);
#endif

#ifdef CONFIG_MCP2210_GPIO
	if (dev->s.is_gpio_probed)
		mcp2210_gpio_remove(dev);
#endif

	/* TODO: free GPIO resources here */

#ifdef CONFIG_MCP2210_IOCTL
	usb_deregister_dev(intf, &mcp2210_class);
#endif
	/* if never configured, make sure we release this */
	if (IS_ENABLED(CONFIG_MCP2210_AUTOPM) && !dev->config)
		usb_autopm_put_interface(dev->intf);
	usb_set_intfdata(intf, NULL);
	kref_put(&dev->kref, mcp2210_delete);

	printk("mcp2210_remove\n");
}

static inline int u16_get_bit(unsigned bit, u16 raw)
{
	BUILD_BUG_ON(bit >= sizeof(raw) * 8);

	return 1 & (raw << bit);
}

/*
 * Moves delayed commands that are due from the delayed_list to the head of the
 * queue and stores the command that will be due next (or NULL if none) in
 * dev->delayed_cmd.
 */
static void process_delayed_list(struct mcp2210_device *dev)
{
	struct mcp2210_cmd *cmd;
	struct mcp2210_cmd *n;
	struct mcp2210_cmd *next = NULL;
	unsigned long now;

	mcp2210_debug();

	if (list_empty(&dev->delayed_list))
		goto done;

	now = jiffies;

	list_for_each_entry_safe(cmd, n, &dev->delayed_list, node) {
		/* if due, move to head of queue and clear delayed flag */
		if (time_after_eq(now, cmd->delay_until)) {
			list_del(&cmd->node);
			list_add(&cmd->node, &dev->cmd_queue);
			cmd->delayed = 0;

		/* of those remaining, determine which is due the soonest */
		} else if (!next || time_before(cmd->delay_until,
					       next->delay_until))
			next = cmd;
	}

done:
	dev->delayed_cmd = next;
	mcp2210_debug("next = %p", next);
}

/* must hold dev_spinlock */
static void push_delayed_cmd(struct mcp2210_device *dev)
{
	unsigned long irqflags;

	BUG_ON(!dev->cur_cmd || !dev->cur_cmd->delayed);

	spin_lock_irqsave(&dev->queue_spinlock, irqflags);
	list_add_tail(&dev->cur_cmd->node, &dev->delayed_list);
//	process_delayed_list(dev); // not needed, will be done in the process_commands loop
	spin_unlock_irqrestore(&dev->queue_spinlock, irqflags);
	dev->cur_cmd = NULL;
}

/*
 * schedule either a delayed_work or timer to pick up a delayed command later
 */
static void schedule_delayed_cmd(struct mcp2210_device *dev, long *pdelay)
{
	struct mcp2210_cmd *cmd;

	cmd = dev->cur_cmd ? dev->cur_cmd : dev->delayed_cmd;
	if (cmd) {
		BUG_ON(!cmd->delayed);
		if (cmd->nonatomic) {
			long delay;

			/* avoid re-reading jiffies if we can */
			if (!pdelay) {
				delay = jiffdiff(jiffies, cmd->delay_until);
				pdelay = &delay;
			}

			/* we better code for that unlikely scenario that could
			 * really fuck things up */
			if (unlikely(*pdelay < 0))
				*pdelay = 0;
			reschedule_delayed_work(dev, (unsigned long)*pdelay);
		} else
			mod_timer(&dev->timer, cmd->delay_until);
	}
}


int process_commands(struct mcp2210_device *dev, const bool lock_held, const bool can_sleep)
{
	unsigned long irqflags = irqflags;
	struct mcp2210_cmd *cmd;
	int ret = 0;

	mcp2210_info();

	/* you can't say that we can sleep if you call this func while holding
	 * dev_spinlock, although you can call w/o lock_held, and can_sleep = false */
	BUG_ON(lock_held && can_sleep);

	if (!lock_held)
		spin_lock_irqsave(&dev->dev_spinlock, irqflags);

restart:
	if (dev->dead) {
		ret = -ESHUTDOWN;
		goto exit_unlock;
	}

	/* use a local for brevity */
	cmd = dev->cur_cmd;

	/* if the current command is dead or done, then free it */
	if (cmd && cmd->state >= MCP2210_STATE_COMPLETE) {
		ret = 0;

		if (cmd->complete) {
			if (cmd->nonatomic) {
				/* this should now be taken care of after
				 * picking up the comand, so should never
				 * happen */
				BUG_ON(!can_sleep);
				spin_unlock_irqrestore(&dev->dev_spinlock,
						       irqflags);
			}

			ret = cmd->complete(cmd, cmd->context);

			if (cmd->nonatomic)
				spin_lock_irqsave(&dev->dev_spinlock, irqflags);
		}

		dev->cur_cmd = NULL;

		/* a completion handler that returns -EINPROGRESS will either
		 * free the cmd later or for commands that shouldn't be freed
		 */
		if (ret != -EINPROGRESS) {
			mcp2210_info("freeing cur_cmd\n");
			kfree(cmd);
		}
	}

	cmd = dev->cur_cmd;

	/* fetch a command, if needed */
	if (!dev->cur_cmd) {
		spin_lock(&dev->queue_spinlock);

		process_delayed_list(dev);

		if (!list_empty(&dev->cmd_queue)) {
			unsigned i;

			dev->cur_cmd = list_entry(dev->cmd_queue.next, struct mcp2210_cmd, node);
			list_del(dev->cmd_queue.next);
			dev->cur_cmd->time_started = jiffies;

			for (i = 0; i < 2; ++i) {
				reset_endpoint(&dev->eps[i]);
				dev->eps[i].retry_count = 0;
			}
		}

		spin_unlock(&dev->queue_spinlock);

		/* If queue was empty then we're done */
		if (!dev->cur_cmd) {
			mcp2210_debug("nothing to do");
			schedule_delayed_cmd(dev, NULL);
			goto exit_unlock;
		}
	}

	cmd = dev->cur_cmd;

	if (cmd->repeat_count) {
		mcp2210_info("running command %p again (repeat_count = %d)",
			     cmd, cmd->repeat_count);
	} else {
		mcp2210_info("got a command (%p)", cmd);
	}

	if (cmd->state == MCP2210_STATE_NEW) {
		if (cmd->delayed) {
			unsigned long cur_time = jiffies;
			long diff = jiffdiff(cmd->delay_until, cur_time);

			if (diff > 0) {
				mcp2210_info("delaying (%satomic) execution "
					     "for aprx. %ums",
					     cmd->nonatomic ? "non-" : "",
					     jiffies_to_msecs(diff));

				if (cmd->non_preemptable) {
					/* if non-preemptable then we just
					 * schedule somebody to pick it up
					 * later */
					schedule_delayed_cmd(dev, &diff);
					goto exit_unlock;
				} else {
					/* if preemptable, then push it onto
					 * the delayed_list and start over */
					push_delayed_cmd(dev);
					goto restart;
				}
			}

			/* otherwise, it is now due */
			cmd->delayed = 0;
		}

		/* defer non-atomic commands if we can't sleep */
		if (unlikely(cmd->nonatomic) && !can_sleep) {
			mcp2210_info("deferring execution");
			reschedule_delayed_work(dev, 0);
			goto exit_unlock;
		}

		/* mark general-purpose commands complete immediately and call
		 * completion handler. */
		if (!cmd->type) {
			cmd->state = MCP2210_STATE_COMPLETE;
			goto restart;
		}

		ret = submit_urbs(cmd, GFP_ATOMIC);

		if (ret) {
			/* the command wasn't ready to run */
			if (ret == -EAGAIN)
				cmd->state = MCP2210_STATE_NEW;

			/* the command is already done */
			else if (ret == -EALREADY) {
				cmd->state = MCP2210_STATE_COMPLETE;
				cmd->status = 0;

			} else {
				cmd->state = MCP2210_STATE_DEAD;
				cmd->status = ret;
				mcp2210_err("submit_urbs() failed: %de, here are "
					"the dead URBs\n", ret);
				mcp2210_dump_urbs(dev, KERN_ERR, 3);

				/* FIXME: do better than just dumping the command & removing it */

				// fail_device(dev, ret);
				// goto exit_unlock;
				/* grab the next request */
			}
			goto restart;
		}

		cmd->state = MCP2210_STATE_SUBMITTED;
	}

exit_unlock:
	mcp2210_info("exit_unlock");

	/* unlock the device */
	if (!lock_held)
		spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);

	return ret;
}


/* We don't want this considered as a candidate for inlining, since it's
 * unlikely to be called and it would just bloat the text */
/* TODO: replace with slower, more compact struct & loop */
static noinline int report_status_error(struct mcp2210_device *dev, u8 status)
{
	const char *desc;
	int ret;

	switch (status) {
	case MCP2210_STATUS_SPI_NOT_OWNED:
		desc = "SPI bus not currently owned";
		ret = -EBUSY;
		break;

	case MCP2210_STATUS_BUSY:
		desc = "device busy";
		ret = -EBUSY;
		break;

	case MCP2210_STATUS_WRITE_FAIL:
		desc = "EEPROM write failure";
		ret = -EIO;
		break;

	case MCP2210_STATUS_NO_ACCESS:
		desc = "no access";
		ret = -EACCES;
		break;

	case MCP2210_STATUS_PERM_LOCKED:
		desc = "device permenantly locked";
		ret = -EACCES;
		break;

	case MCP2210_STATUS_BAD_PASSWD:
		desc = "bad password";
		ret = -EACCES;
		break;

	case MCP2210_STATUS_UNKNOWN_CMD:
		desc = "unknown command";
		ret = -EPROTO;
		break;

	default:
		desc = "invalid status code, comm error?";
		ret = -EPROTO;
		break;
	}

	mcp2210_err("status 0x%02hhx: %s\n", status, desc);
	return ret;
}

/******************************************************************************
 * Background thread
 */

static bool reschedule_delayed_work(struct mcp2210_device *dev,
				    unsigned long _jiffies)
{
	if (!dev->dead)
		return false;

	cancel_delayed_work(&dev->delayed_work);
	return schedule_delayed_work(&dev->delayed_work, _jiffies);
}

static inline bool reschedule_delayed_work_ms(struct mcp2210_device *dev,
					      unsigned int ms)
{
	return reschedule_delayed_work(dev, msecs_to_jiffies(ms));
}

static inline int all_ubrs_done(struct mcp2210_device *dev)
{
	return dev->eps[EP_OUT].state >= MCP2210_STATE_COMPLETE
	    && dev->eps[EP_IN].state >= MCP2210_STATE_COMPLETE;
}

/**
 * unlink_urbs - unlink URBs
 *
 * must hold dev->dev_spinlock
 *
 * returns non-zero if both in and out URBs are killed, zero if their
 * completion handlers have not yet been called/finished.
 */
static int unlink_urbs(struct mcp2210_device *dev)
{
	struct mcp2210_endpoint *ep;

	for (ep = dev->eps; ep != &dev->eps[2]; ++ep) {
		if (ep->state != MCP2210_STATE_SUBMITTED)
			continue;

		ep->kill = 1;

		mcp2210_debug("ep->is_dir_in = %d", ep->is_dir_in);

		if(!atomic_dec_and_test(&ep->unlink_in_process)) {
			mcp2210_crit("fuck");
			BUG();
		}

		usb_unlink_urb(ep->urb);

		if(atomic_inc_and_test(&ep->unlink_in_process)) {
			mcp2210_crit("fuck");
			BUG();
		}
	}

	return all_ubrs_done(dev);
}

/* Must hold dev->dev_spinlock, plus pass irqflags.
 * Returns zero if command re-locked, 1 if current command changed or device
 * is failed (dev->dev_spinlock will always be relocked). */
static void kill_urbs(struct mcp2210_device *dev, unsigned long *irqflags)
{
	struct mcp2210_endpoint *out = &dev->eps[EP_OUT];
	struct mcp2210_endpoint *in = &dev->eps[EP_IN];

	mcp2210_warn();

	if (unlink_urbs(dev))
		return;

	if (irqflags)
		spin_unlock_irqrestore(&dev->dev_spinlock, *irqflags);

	if (out->state == MCP2210_STATE_SUBMITTED)
		usb_kill_urb(out->urb);
	if (in->state == MCP2210_STATE_SUBMITTED)
		usb_kill_urb(in->urb);

//	BUG_ON(out->state < MCP2210_STATE_COMPLETE);
//	BUG_ON(in->state < MCP2210_STATE_COMPLETE);

	if (irqflags)
		spin_lock_irqsave(&dev->dev_spinlock, *irqflags);
}


/* must hold dev->dev_spinlock */
__cold noinline static void fail_command(struct mcp2210_device *dev, int error,
					 u8 mcp_error, int reschedule_dw)
{
	struct mcp2210_cmd *cmd = dev->cur_cmd;

	cmd->state = MCP2210_STATE_DEAD;
	cmd->status = error;
	cmd->mcp_status = mcp_error;
	unlink_urbs(dev);

	if (reschedule_dw) {
		/* reschedule the delayed_work-er for now */
		mcp2210_info("rescheduling delayed_work to run now");
		reschedule_delayed_work(dev, 0);
	}
}

static void timer_callback(unsigned long context)
{
	struct mcp2210_device *dev = (void*)context;

	mcp2210_debug();
	process_commands(dev, false, false);
}

/* work queue callback to take care of hung URBs and other misc bottom half
 * stuff
 *
 * verify only one instance running
 * - lock dev?
 * check current command
 * - age of endpoints
 */
static void delayed_work_callback(struct work_struct *work)
{
	struct mcp2210_device *dev = container_of((struct delayed_work *)work,
						  struct mcp2210_device,
						  delayed_work);
	struct mcp2210_cmd *cmd;
	unsigned long irqflags;
	struct mcp2210_endpoint *ep;
	unsigned long start_time = jiffies;
	int ret;

	/* static const */ long TIMEOUT_URB = msecs_to_jiffies(750) + 1;
	/* static const */ long TIMEOUT_HUNG = msecs_to_jiffies(8000);

#ifdef CONFIG_MCP2210_DEBUG
	if (atomic_inc_and_test(&dev->manager_running)) {
		atomic_dec(&dev->manager_running);
		mcp2210_crit("BUG: two instances of delayed work running!******");

		BUG();
		return;
	}
#endif

	spin_lock_irqsave(&dev->dev_spinlock, irqflags);

	if (dev->dead) {
		mcp2210_warn("dead");
		goto exit_unlock_dev;
	}

	if(IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		if (!dev->cur_cmd) {
			if (dev->debug_chatter_count < 12) {
				/* limit spam to 12 lines when idle */
				mcp2210_debug("nothing to do");
				++dev->debug_chatter_count;
			} else if (dev->debug_chatter_count == 12) {
				mcp2210_debug("nothing to do, shutting up...");
				++dev->debug_chatter_count;
			}
		} else
			dev->debug_chatter_count = 0;
	}

	if (!dev->cur_cmd)
		goto exit_reschedule;

	cmd = dev->cur_cmd;

	mcp2210_debug("cmd: %p, cmd->state: %d", cmd, cmd->state);

	/* run any deferred work that is now due */
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);
	process_commands(dev, false, true);
	spin_lock_irqsave(&dev->dev_spinlock, irqflags);

	/* check for stalled URBs */
	for (ep = dev->eps; ep != &dev->eps[2]; ++ep) {
		// validate  cmd / ep states
		long age = jiffdiff(start_time, ep->submit_time);

		if (ep->state == MCP2210_STATE_NEW)
			continue;

		mcp2210_info("  %s URB in state %d is %d mS old",
			     urb_dir_str[ep->is_dir_in], ep->state,
			     jiffies_to_msecs(age));

		if (ep->state == MCP2210_STATE_SUBMITTED) {
			/* this is the state we'll normally see if dev->cur_cmd is not
			 * null.  Make sure the command hasn't timed out and then
			 * exit. */
			if (likely(age < TIMEOUT_URB)) {
				goto exit_reschedule;
			} else {
#ifdef CONFIG_MCP2210_USB_QUIRKS
#else
#endif
				/* timeout sending request, just fail the command */
				mcp2210_err("URB timed out -- %s->retry_count = %u", urb_dir_str[ep->is_dir_in], ep->retry_count);
				cmd->status = -EIO;
				ret = unlink_urbs(dev);
				goto exit_reschedule;
			}
		} else if (unlikely(age > TIMEOUT_HUNG)) {
			mcp2210_err("Command timed-out (age = %ld, TIMEOUT_HUNG = %ld)", age, TIMEOUT_HUNG);
			fail_command(dev, -EIO, 0, false);
			goto exit_reschedule;
		}
	}

exit_reschedule:
	/* reschedule as long as the device isn't dead and the delayed_work
	 * wasn't already re-armed by process_commands() */
	if (!dev->dead && !timer_pending(&dev->delayed_work.timer))
		schedule_delayed_work(&dev->delayed_work, msecs_to_jiffies(4000));

exit_unlock_dev:
#ifdef CONFIG_MCP2210_DEBUG
	atomic_dec(&dev->manager_running);
#endif
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);
}


/******************************************************************************
 * Base command & URB functions
 */

/**
 * mcp2210_alloc_cmd -- allocate a new (generic) command
 *
 */
struct mcp2210_cmd *mcp2210_alloc_cmd(struct mcp2210_device *dev,
				      const struct mcp2210_cmd_type *type,
				      size_t size, gfp_t gfp_flags)
{
	struct mcp2210_cmd *cmd;

	BUG_ON(size < sizeof(struct mcp2210_cmd));

	cmd = kzalloc(size, gfp_flags);
	if (!cmd)
		return NULL;

	cmd->dev = dev;
	cmd->type = type;
	cmd->status = -EINPROGRESS;
	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		cmd->node.next = LIST_POISON1;
		cmd->node.prev = LIST_POISON2;
	}

	/*  Already zeroed....
	cmd->time_queued	= 0;
	cmd->time_started	= 0;
	cmd->delay	= 0;
	cmd->mcp_status		= 0;
	cmd->state		= MCP2210_STATE_NEW;
	cmd->can_retry		= 0;
	cmd->nonatomic		= 0;
	cmd->repeat_count	= 0;
	cmd->complete		= NULL;
	cmd->context		= NULL;
	*/

	return cmd;
}

/* may have device or command locked, but not the queue */
int mcp2210_add_cmd(struct mcp2210_cmd *cmd, bool free_if_dead)
{
	struct mcp2210_device *dev = cmd->dev;
	unsigned long irqflags;
	int ret = 0;

	if (!cmd)
		return -ENOMEM;

	spin_lock_irqsave(&dev->queue_spinlock, irqflags);
	if (dev->dead && free_if_dead) {
		kfree(cmd);
		ret = -ESHUTDOWN;
	} else
		list_add_tail(&cmd->node, &dev->cmd_queue);
	spin_unlock_irqrestore(&dev->queue_spinlock, irqflags);

	cmd->time_queued = jiffies;
	return ret;
}

/* Perform basic checks on a response received from the device.  (Do command and
 * sub-command codes match? etc.)
 */
static int check_response(struct mcp2210_device *dev)
{
	struct mcp2210_msg *req = dev->eps[EP_OUT].buffer;
	struct mcp2210_msg *rep = dev->eps[EP_IN].buffer;
	u8 sub_cmd;
	u8 mcp_status = rep->head.rep.status;

	dev->cur_cmd->mcp_status = mcp_status;

	if (dev->dead)
		return -ESHUTDOWN;

	if (unlikely(req->cmd != rep->cmd)) {
		mcp2210_err("mis-matched command codes (sent 0x%02hhx, "
			    "received 0x%02hhx)", req->cmd,
			    rep->cmd);
		return -EPROTO;
	}

//	if (unlikely(rep->head.req.status != MCP2210_STATUS_SUCCESS))
//		return report_status_error(dev, rep->head.req.status);

	if (unlikely(mcp_status != MCP2210_STATUS_SUCCESS)) {
		mcp2210_err("mcp2210 command failed (status = 0x%02hhx)",
			    mcp_status);
		return report_status_error(dev, mcp_status);
	}


	switch (rep->cmd) {
	case MCP2210_CMD_GET_NVRAM:
	case MCP2210_CMD_SET_NVRAM:
		sub_cmd = req->head.req.xet.sub_cmd;

		if (likely(sub_cmd == rep->head.rep.xet.sub_cmd))
			break;

		mcp2210_err("MCP2210_CMD_%cET_NVRAM: sub-command mis-match. "
			    "Sent 0x%02hhx, received 0x%02hhx",
			    rep->cmd == MCP2210_CMD_GET_NVRAM ? 'G' : 'S',
			    sub_cmd,
			    rep->head.rep.xet.sub_cmd);
		return -EPROTO;
	}

	return 0;
}


/* complete_urb_bad - handles a bad condition in URB completion callback
 *
 * must hold dev->dev_spinlock
 *
 * populates cmd->status with an appropriate error
 * -ESHUTDOWN if the driver is shutting down or we've marked it as dead
 * -EPROTO if something screwy is happening (like wrong message size)
 * otherwise, the status returned in the URB
 *
 * Returns non-zero if all URBs are dead, zero otherwise.
 */
__cold noinline static int
complete_urb_bad(struct mcp2210_device *dev, const int is_dir_in)
{
	struct mcp2210_cmd *cmd = dev->cur_cmd;
	struct mcp2210_endpoint *ep = &dev->eps[is_dir_in];
	int status = ep->urb->status;

	mcp2210_warn();
	ep->state = MCP2210_STATE_DEAD;

	if (dev->dead) {
		cmd->status = -ESHUTDOWN;
		goto unlink_urbs;
	}

	if (ep->urb->actual_length) {
		mcp2210_warn("Failed %s URB has %u bytes of data:",
			     urb_dir_str[is_dir_in], ep->urb->actual_length);
		mcp2210_dump_urbs(dev, KERN_WARNING, 1 << is_dir_in);
	}

	if (status) {
		mcp2210_warn("%s URB status is %de\n",
			     urb_dir_str[is_dir_in], status);

		switch (status) {
		case -ENOENT:
		case -ECONNRESET:
		case -ESHUTDOWN:
			cmd->status = -ESHUTDOWN;
			break;

		default:
			cmd->status = status;
		}
	}

	/* all mcp2210 messages are 64 bytes */
	if (!status && ep->urb->actual_length != 64) {
		mcp2210_warn("MCP2210 returned a message of the wrong size "
			     "(ep->urb->actual_length = %d)",
			     ep->urb->actual_length);
		cmd->status = -EPROTO;
	}

unlink_urbs:
	return unlink_urbs(dev);
}


static void complete_urb(struct urb *urb)
{
#if 0
	/* alternatively, we can set the mcp2210_endpoint as the urb context */
	struct mcp2210_endpoint *ep = urb->context;
	const int is_dir_in = ep->is_dir_in;
	struct mcp2210_device *dev = container_of(ep, struct mcp2210_device,
						  eps[is_dir_in]);
#else
	struct mcp2210_device *dev = urb->context;
	const int is_dir_in = !!usb_endpoint_dir_in(&urb->ep->desc);
	struct mcp2210_endpoint *ep;
	struct mcp2210_endpoint *other_ep;
#endif
	struct mcp2210_cmd *cmd;
	unsigned long irqflags;
	int ret;
	int lock_held;

	if (!dev->cur_cmd) {
		mcp2210_crit("I see the RPi usb host controller is fucking up "
			     "again.  I suggest you reboot.");
		return;
	}

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		BUG_ON(!dev);
		BUG_ON(!dev->cur_cmd);
		BUG_ON(!dev->cur_cmd->type);
		BUG_ON(dev->cur_cmd->dev != dev);
	}

	cmd = dev->cur_cmd;
	ep = &dev->eps[is_dir_in];
	other_ep = &dev->eps[!is_dir_in];

	/* This is a crappy mechanism for dealing with recursion.  The problem
	 * is that usb_unlink_urb can call the completion handlers */
	lock_held = !atomic_read(&ep->unlink_in_process);

	mcp2210_debug("%s, state: %#02x urb->status: %de, lock_held: %d, kill: %d", urb_dir_str[is_dir_in], ep->state, urb->status, lock_held, ep->kill);

	if (!lock_held)
		spin_lock_irqsave(&dev->dev_spinlock, irqflags);

	if (unlikely(ep->kill)) {
		ep->state = MCP2210_STATE_DEAD;
		mcp2210_warn("kill complete for %s URB",
			     urb_dir_str[is_dir_in]);

		if (other_ep->state >= MCP2210_STATE_COMPLETE)
			goto bad;
		else
			goto exit_unlock;
	}

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		/* make sure that we came up with is_dir_in correctly */
		BUG_ON(ep->urb != urb);
		BUG_ON(cmd->state != MCP2210_STATE_SUBMITTED);
		BUG_ON(ep->state != MCP2210_STATE_SUBMITTED);
	}

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG) && dump_urbs && is_dir_in) {
		mcp2210_debug("-------------RESPONSE------------");
		mcp2210_dump_urbs(dev, KERN_DEBUG, 2);
	}


	/* check for unlikely conditions and call cold function to handle them
	 * if they occur */
	if (dev->dead || urb->status || urb->actual_length != 64) {
		if (complete_urb_bad(dev, is_dir_in))
			goto bad;
		else /* need to wait for other URB to complete */
			goto exit_unlock;
	}

	ep->state = MCP2210_STATE_COMPLETE;

	/* when receiving, we'll have a buffer that's in the endianness of the
	 * chip */
	if (is_dir_in)
		ep->is_mcp_endianness = 1;

	switch (other_ep->state) {
	case MCP2210_STATE_COMPLETE: {
		const struct mcp2210_cmd_type *type = cmd->type;

		cmd->status = ret = check_response(dev);

		if (ret) {
			if (cmd->mcp_status && type->mcp_error) {
				ret = type->mcp_error(cmd);
				if (ret == -EAGAIN) {
					mcp2210_warn("ignoring mcp_status error and repeating");
					dev->eps[EP_OUT].retry_count = 0;
					dev->eps[EP_IN].retry_count = 0;
					cmd->status = -EINPROGRESS;
					goto restart;
				}
			}

			cmd->status = ret;
			goto bad;
		}

		cmd->state = MCP2210_STATE_COMPLETE;
		if (type->complete_urb) {
			ret = type->complete_urb(cmd);
			if (IS_ENABLED(CONFIG_MCP2210_DEBUG_VERBOSE) && dump_commands) {
				mcp2210_debug("--------FINAL COMMAND STATE--------");
				dump_cmd(KERN_DEBUG, 0, "cmd: ", cmd);
			}
		} else {
		}

		if (ret == -EAGAIN) {
			dev->eps[EP_OUT].retry_count = 0;
			dev->eps[EP_IN].retry_count = 0;
			goto restart;
		}

		break;
	}
	case MCP2210_STATE_DEAD:
bad:
		mcp2210_err("Command is now dead");
		cmd->state = MCP2210_STATE_DEAD;
		if (unlikely(!cmd->status || cmd->status == -EINPROGRESS)) {
			mcp2210_err("******** missing error code *******");
			cmd->status = -EINVAL;
		}

		if (!cmd->can_retry)
			break;

#ifdef CONFIG_MCP2210_USB_QUIRKS
# define RETRY_COUNT 8
#else
# define RETRY_COUNT 1
#endif
		if (dev->eps[EP_OUT].retry_count < RETRY_COUNT) {
			mcp2210_warn("retrying................");
			++dev->eps[EP_OUT].retry_count;
			++dev->eps[EP_IN].retry_count;
restart:
			cmd->state = MCP2210_STATE_NEW;
			++cmd->repeat_count;
			dev->eps[EP_OUT].state = 0;
			dev->eps[EP_IN].state = 0;
			dev->eps[EP_OUT].kill = 0;
			dev->eps[EP_IN].kill = 0;
			goto process_commands;
		}
		break;

	default:
		goto exit_unlock;
	}

//	if (cmd->complete)
//		cmd->complete(cmd, cmd->context);

process_commands:
	process_commands(dev, true, false);

exit_unlock:
	/* FIXME: more crap */
	BUG_ON(lock_held == atomic_read(&ep->unlink_in_process));

	if (!lock_held)
		spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);
}

static int submit_urb(struct mcp2210_device *dev, struct mcp2210_endpoint *ep,
		      gfp_t gfp_flags)
{
	int ret;

	mcp2210_info("%s", urb_dir_str[ep->is_dir_in]);
	reset_endpoint(ep);

	ret = usb_submit_urb(ep->urb, gfp_flags);
	if (unlikely(ret)) {
//		const u8 is_dir_in = ep->is_dir_in;
		ep->state = MCP2210_STATE_DEAD;
		dev->cur_cmd->state = MCP2210_STATE_DEAD;

		mcp2210_err("usb_submit_urb failed for %s URB %de",
			    urb_dir_str[ep->is_dir_in], ret);
		return ret;
	}
	ep->submit_time = jiffies;
	ep->state = MCP2210_STATE_SUBMITTED;

	return 0;
}

static int submit_urbs(struct mcp2210_cmd *cmd, gfp_t gfp_flags)
{
	struct mcp2210_device *dev;
	int ret = 0;

	BUG_ON(!cmd || !cmd->dev);

	dev = cmd->dev;

	mcp2210_info();

	if (dev->dead)
		return -ESHUTDOWN;

	BUG_ON(!cmd->type || !cmd->type->submit_prepare);

	ret = cmd->type->submit_prepare(cmd);
	if (ret) {
		if (ret != -EALREADY && ret != -EAGAIN)
			mcp2210_err("submit_prepare failed with %de", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG_VERBOSE))
		dump_cmd(KERN_DEBUG, 0, "submit_urbs: cmd = ", cmd);

	if (unlikely(cmd->state != MCP2210_STATE_NEW))
		mcp2210_warn("unexpected: cmd->state is %u", cmd->state);

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG_VERBOSE)) {
		mcp2210_debug("----------SUBMITTING----------\n");
		mcp2210_dump_urbs(dev, KERN_DEBUG, 1);
	}

	ret = submit_urb(dev, &dev->eps[EP_IN], gfp_flags);
	if (unlikely(ret))
		return ret;

	ret = submit_urb(dev, &dev->eps[EP_OUT], gfp_flags);
	if (unlikely(ret)) {
		unlink_urbs(dev);
		return ret;
	}

	return ret;
}

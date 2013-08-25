/*
 *  MCP2210 gpio support
 *
 *  Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
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
 */

#ifdef CONFIG_MCP2210_GPIO

#include <linux/gpio.h>
#include <linux/completion.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"


//static int  request	    (struct gpio_chip *chip, unsigned offset);
//static void free	    (struct gpio_chip *chip, unsigned offset);
static int  get_direction   (struct gpio_chip *chip, unsigned offset);
static int  direction_input (struct gpio_chip *chip, unsigned offset);
static int  get		    (struct gpio_chip *chip, unsigned offset);
static int  direction_output(struct gpio_chip *chip, unsigned offset, int value);
//static int  set_debounce    (struct gpio_chip *chip, unsigned offset, unsigned debounce);
static void set		    (struct gpio_chip *chip, unsigned offset, int value);
//static int  to_irq	    (struct gpio_chip *chip, unsigned offset);
//static void dbg_show	    (struct seq_file *s, struct gpio_chip *chip);

static inline struct mcp2210_device *chip2dev(struct gpio_chip *chip) {
	return container_of(chip, struct mcp2210_device, gpio);
}

/******************************************************************************
 * probe & remove
 */

int mcp2210_gpio_probe(struct mcp2210_device *dev)
{
	struct gpio_chip *gpio = &dev->gpio;
	int ret;
	int is_gpio_probed = dev->s.is_gpio_probed; /* unalias */

	mcp2210_info();

	BUG_ON(!dev);
	BUG_ON(!dev->config);
	BUG_ON(is_gpio_probed);

	if (!dev || !dev->config || is_gpio_probed)
		return -EINVAL;

	gpio->label		= "mcp2210";
	gpio->dev		= &dev->udev->dev;
	gpio->owner		= THIS_MODULE;
	INIT_LIST_HEAD(&gpio->list);

//	gpio->request		= request;
//	gpio->free		= free;
	gpio->get_direction	= get_direction;
	gpio->direction_input	= direction_input;
	gpio->get		= get;
	gpio->direction_output	= direction_output;
//	gpio->set_debounce	= set_debounce;
	gpio->set		= set;
//	gpio->to_irq		= to_irq;
//	gpio->dbg_show		= dbg_show;

	gpio->base		= -1; /* request dynamic ID allocation */
	gpio->ngpio		= MCP2210_NUM_PINS;
	/* private: gpio->desc */
	gpio->names		= dev->names;
	gpio->can_sleep		= 1; /* we have to make them sleep because we
					need to do an URB */
	gpio->exported		= 0; /* wtf is this anyway? */


	ret = gpiochip_add(gpio);
	if (ret) {
		mcp2210_err("Failed to register GPIOs: %de", ret);
		return ret;
	}

	mcp2210_info("registered GPIOs from %d to %d", gpio->base,
		     gpio->base + gpio->ngpio - 1);

	return 0;
}

void mcp2210_gpio_remove(struct mcp2210_device *dev)
{
	int ret;

	mcp2210_info();
	ret = gpiochip_remove(&dev->gpio);
	if (ret) {
		mcp2210_err("gpiochip_remove() failed with %de", ret);
		return;
	}
	dev->s.is_gpio_probed = 0;
}


/******************************************************************************
 * struct gpio_chip functions
 */

struct gpio_completion {
	struct completion completion;
	u16 gpio_val_dir;
	int status;
	u8 mcp_status;
};

static int complete_cmd_chip(struct mcp2210_cmd *cmd_head, void *context)
{
	struct gpio_completion *c = context;
	struct mcp2210_device *dev = cmd_head->dev;
	struct mcp2210_cmd_ctl *cmd = (void *)cmd_head;

	switch (cmd->req.cmd) {
	case MCP2210_CMD_GET_PIN_DIR:
	case MCP2210_CMD_GET_PIN_VALUE:
		c->gpio_val_dir = le16_to_cpu(dev->eps[EP_IN].buffer->body.gpio);
		break;
	case MCP2210_CMD_SET_CHIP_CONFIG:
	case MCP2210_CMD_SET_PIN_DIR:
	case MCP2210_CMD_SET_PIN_VALUE:
		break;
	default:
		MCP_ASSERT(0);
	}

	c->status     = cmd_head->status;
	c->mcp_status = cmd_head->mcp_status;
	complete_all(&c->completion);
	return 0;
}

static int do_gpio_cmd(struct mcp2210_device *dev, u8 cmd_code, void *body,
		       size_t size)
{
	struct gpio_completion c;
	struct mcp2210_cmd_ctl *cmd;
	int ret;

	cmd = mcp2210_alloc_ctl_cmd(dev, cmd_code, 0, body, size, false, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	memset(&c, 0, sizeof(c));
	init_completion(&c.completion);

	switch (cmd_code) {
	case MCP2210_CMD_SET_CHIP_CONFIG:
	case MCP2210_CMD_GET_PIN_DIR:
	case MCP2210_CMD_SET_PIN_DIR:
	case MCP2210_CMD_GET_PIN_VALUE:
	case MCP2210_CMD_SET_PIN_VALUE:
		break;
	default:
		MCP_ASSERT(0);
	}

	cmd->head.complete = complete_cmd_chip;
	cmd->head.context = &c;

	ret = mcp2210_add_or_free_cmd(&cmd->head);
	if (ret)
		return ret;

	process_commands(dev, GFP_KERNEL, 0);

	wait_for_completion(&c.completion);

	if (c.status) {
		MCP_ASSERT(c.status < 0);
		return c.status;
	}

	return c.gpio_val_dir;
}

static int get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct mcp2210_device *dev = chip2dev(chip);
	unsigned long irqflags;
	int ret;

	BUG_ON(offset >= MCP2210_NUM_PINS);

	mcp2210_debug();
	spin_lock_irqsave(&dev->dev_spinlock, irqflags);
		ret = 1 & (dev->s.chip_settings.gpio_direction >> offset);
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);

	return ret;
}

static int set_dir_and_value(struct gpio_chip *chip, unsigned pin, int dir,
			     int value)
{
	struct mcp2210_device *dev = chip2dev(chip);
	unsigned long irqflags;
	int ret = 0;
	u8 pin_mode;
	u16 pin_vals;
	u16 pin_dirs;
	u16 new_vals;
	u16 new_dirs;
	u8 set_vals = 0;
	u8 set_dirs = 0;

	BUG_ON(pin >= MCP2210_NUM_PINS);

	mcp2210_debug("chip:%p, pin: %u, dir: %d, value: %d", chip, pin, dir,
		     value);
	spin_lock_irqsave(&dev->dev_spinlock, irqflags);
		pin_vals = dev->s.chip_settings.gpio_value;
		pin_dirs = dev->s.chip_settings.gpio_direction;
		pin_mode = dev->config->pins[pin].mode;
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);

	if (pin_mode != MCP2210_PIN_GPIO && pin_mode != MCP2210_PIN_UNUSED) {
		mcp2210_err("pin %u not gpio", pin);
		return -EPERM;
	}

	switch (dir) {
	/* If not changing direction and current dir is input, then error */
	case MCP2210_GPIO_NO_CHANGE:
		if (pin_dirs & 1 << pin)
			return -EROFS;
		/* intentional fall-through */

	case MCP2210_GPIO_OUTPUT:
		new_vals = (pin_vals & ~(1 << pin)) | value << pin;
		if (new_vals != pin_vals)
			set_vals = 1;

		if (dir == MCP2210_GPIO_NO_CHANGE)
			break;
		/* intentional fall-through */

	case MCP2210_GPIO_INPUT:
		new_dirs = (pin_dirs & ~(1 << pin)) | dir << pin;
		if (new_dirs != pin_dirs)
			set_dirs = 1;
		break;

	default:
		BUG();
	}

	mcp2210_debug("pin_vals = 0x%04hx, pin_dirs = 0x%04hx\n"
		      "new_vals = 0x%04hx, new_dirs = 0x%04hx\n"
		      "set_vals = %hhu  , new_dirs = %hhu",
		      pin_vals, pin_dirs, new_vals, new_dirs,
		      set_dirs, set_vals);
	if (set_vals & set_dirs) {
		/* I'm not aware of any other mechanism to change the direction
		 * to output and the value at the same time, and I don't think
		 * this device supports changing the direction to output, but
		 * leaving it's actual state in high-z until you give it a
		 * value, so we'll just set the full chip settings.
		 */
		struct mcp2210_chip_settings cs = dev->s.chip_settings;

		cs.gpio_value	  = new_vals;
		cs.gpio_direction = new_dirs;
		return do_gpio_cmd(dev, MCP2210_CMD_SET_CHIP_CONFIG, &cs,
				  sizeof(cs));
	}

	if (set_vals) {
		ret = do_gpio_cmd(dev, MCP2210_CMD_SET_PIN_VALUE, &new_vals,
				  sizeof(new_vals));
		if (ret < 0)
			return ret;
	}

	if (set_dirs) {
		ret = do_gpio_cmd(dev, MCP2210_CMD_SET_PIN_DIR, &new_dirs,
				  sizeof(new_dirs));
	}

	return ret;
}

static int direction_input(struct gpio_chip *chip, unsigned offset)
{
	return set_dir_and_value(chip, offset, MCP2210_GPIO_INPUT, 0);
}

static int direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return set_dir_and_value(chip, offset, MCP2210_GPIO_OUTPUT, value);
}

static int get(struct gpio_chip *chip, unsigned offset)
{
	struct mcp2210_device *dev = chip2dev(chip);
	unsigned long irqflags;
	int val;
	int dir;
	int ret;

	BUG_ON(offset >= MCP2210_NUM_PINS);

	mcp2210_debug();

	spin_lock_irqsave(&dev->dev_spinlock, irqflags);
		val = 1 & (dev->s.chip_settings.gpio_value >> offset);
		dir = 1 & (dev->s.chip_settings.gpio_direction >> offset);
	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);

	/* TODO: we need to add the ability to return the last-known value
	 * returned by polling rather than querying across USB each time */
	if (dir == MCP2210_GPIO_OUTPUT)
		return val;

	ret = do_gpio_cmd(dev, MCP2210_CMD_GET_PIN_VALUE, NULL, 0);

	if (ret < 0)
		return ret;
	else
		return 1 & ret >> offset;
}

static void set(struct gpio_chip *chip, unsigned offset, int value)
{
	set_dir_and_value(chip, offset, MCP2210_GPIO_NO_CHANGE, value);
}

#endif /* CONFIG_MCP2210_GPIO */

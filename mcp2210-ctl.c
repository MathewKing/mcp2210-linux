/*
 *  MCP 2210 driver for linux
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

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/export.h>
#include <linux/workqueue.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"

static int ctl_submit_prepare(struct mcp2210_cmd *cmd);
static int ctl_complete_urb(struct mcp2210_cmd *cmd);

const struct mcp2210_cmd_type mcp2210_cmd_type_ctl = {
	.id = MCP2210_CMD_TYPE_CTL,
	.submit_prepare = ctl_submit_prepare,
	.complete_urb = ctl_complete_urb,
	.dump = dump_cmd_ctl,
	.desc = "ctl",
};

//struct mcp2210_cmd_type mcp2210_cmd_types[MCP2210_CMD_TYPE_CTL] = mcp2210_cmd_type_ctl;

/******************************************************************************
 * Control Command Functions
 */

static inline void flip_chip_settings(struct mcp2210_msg *msg) {
	struct mcp2210_chip_settings *body = &msg->body.chip;
	body->gpio_value	= cpu_to_le16(body->gpio_value);
	body->gpio_direction	= cpu_to_le16(body->gpio_direction);
}

static inline void flip_gpio(struct mcp2210_msg *msg) {
	msg->body.gpio = cpu_to_le16(msg->body.gpio);
}

static inline void flip_spi_settings(struct mcp2210_msg *msg) {
	struct mcp2210_spi_xfer_settings *body = &msg->body.spi;
	body->bitrate		    = cpu_to_le32(body->bitrate);
	body->idle_cs		    = cpu_to_le16(body->idle_cs);
	body->active_cs		    = cpu_to_le16(body->active_cs);
	body->cs_to_data_delay	    = cpu_to_le16(body->cs_to_data_delay);
	body->last_byte_to_cs_delay = cpu_to_le16(body->last_byte_to_cs_delay);
	body->delay_between_bytes   = cpu_to_le16(body->delay_between_bytes);
	body->bytes_per_trans	    = cpu_to_le16(body->bytes_per_trans);
}

static inline void flip_get_usb_params(struct mcp2210_msg *msg) {
	msg->body.get_usb_params.vid = cpu_to_le16(msg->body.get_usb_params.vid);
	msg->body.get_usb_params.pid = cpu_to_le16(msg->body.get_usb_params.pid);
}

static inline void flip_set_usb_params(struct mcp2210_msg *msg) {
	msg->body.set_usb_params.vid = cpu_to_le16(msg->body.set_usb_params.vid);
	msg->body.set_usb_params.pid = cpu_to_le16(msg->body.set_usb_params.pid);
}

static inline void flip_interrupt_event_counter(struct mcp2210_msg *msg) {
	msg->body.interrupt_event_counter = cpu_to_le16(msg->body.interrupt_event_counter);
}

/* Keep this a no-inline, monolithic function */
static __attribute__((flatten)) noinline void flip_msg(struct mcp2210_msg *msg)
{
	switch (msg->cmd) {
	case MCP2210_CMD_SET_NVRAM:
	case MCP2210_CMD_GET_NVRAM:
		switch (msg->head.req.xet.sub_cmd) {
		case MCP2210_NVRAM_SPI:
			goto spi_config;

		case MCP2210_NVRAM_CHIP:
			goto chip_config;

		case MCP2210_NVRAM_KEY_PARAMS:
			if (msg->cmd == MCP2210_CMD_SET_NVRAM)
				flip_set_usb_params(msg);
			else
				flip_get_usb_params(msg);
			break;

		case MCP2210_NVRAM_PROD:
		case MCP2210_NVRAM_MFG:
		default:
			break;
		};
		break;

	case MCP2210_CMD_GET_SPI_CONFIG:
	case MCP2210_CMD_SET_SPI_CONFIG:
spi_config:
		flip_spi_settings(msg);
		break;

	case MCP2210_CMD_GET_CHIP_CONFIG:
	case MCP2210_CMD_SET_CHIP_CONFIG:
chip_config:
		flip_chip_settings(msg);
		break;

	case MCP2210_CMD_GET_PIN_DIR:
	case MCP2210_CMD_SET_PIN_DIR:
	case MCP2210_CMD_GET_PIN_VALUE:
	case MCP2210_CMD_SET_PIN_VALUE:
		flip_gpio(msg);
		break;

	case MCP2210_CMD_GET_INTERRUPTS:
		flip_interrupt_event_counter(msg);
		break;

	case MCP2210_CMD_READ_EEPROM:
	case MCP2210_CMD_WRITE_EEPROM:
	case MCP2210_CMD_SPI_TRANSFER:
	case MCP2210_CMD_SPI_CANCEL:
	case MCP2210_CMD_SPI_RELEASE:
	case MCP2210_CMD_SEND_PASSWD:
	default:
		break;
	};
}

static inline void flip_ep_buffer(struct mcp2210_endpoint *ep, unsigned to_mcp)
{
	to_mcp = !!to_mcp;
	printk(KERN_INFO "flip_ep_buffer: to_mcp = %u, ep->is_mcp_endianness = %hhu", to_mcp, ep->is_mcp_endianness);

	if (ep->is_mcp_endianness != to_mcp) {
		printk(KERN_INFO "flipping...\n");
		flip_msg(ep->buffer);
		ep->is_mcp_endianness = to_mcp;
	}
}

static inline void flip_ctl_cmd_req(struct mcp2210_cmd_ctl *cmd, unsigned to_mcp)
{
	to_mcp = !!to_mcp;
	printk(KERN_INFO "flip_ctl_cmd_req:to_mcp = %u, cmd->is_mcp_endianness = %hhu", to_mcp, cmd->is_mcp_endianness);

	if (cmd->is_mcp_endianness != to_mcp) {
		printk(KERN_INFO "flipping...\n");
		flip_msg(&cmd->req);
		cmd->is_mcp_endianness = to_mcp;
	}
}

static void store_chip_settings(struct mcp2210_device *dev,
				const struct mcp2210_chip_settings *cfg,
				int is_power_up)
{
	struct mcp2210_chip_settings *dest = is_power_up
					   ? &dev->s.power_up_chip_settings
					   : &dev->s.chip_settings;

	memcpy(dest, cfg, sizeof(*cfg));

	if (is_power_up)
		dev->s.have_power_up_chip_settings = 1;
	else
		dev->s.have_chip_settings = 1;
}

static void store_spi_settings(struct mcp2210_device *dev,
			       const struct mcp2210_spi_xfer_settings *cfg,
			       unsigned pin, int is_power_up)
{
	struct mcp2210_spi_xfer_settings *dest = is_power_up
					       ? &dev->s.power_up_spi_settings
					       : &dev->s.spi_settings;

	memcpy(dest, cfg, sizeof(*cfg));

	if (is_power_up)
		dev->s.have_power_up_spi_settings = 1;
	else {
		//struct mcp2210_spi_config *devcfg = &dev->config.pins[pin].body.spi;
		dev->s.have_spi_settings = 1;
		dev->s.cur_spi_config = pin;
		dev->s.idle_cs = cfg->idle_cs;
		dev->s.active_cs = cfg->active_cs;
	}
}

static void store_usb_key_params(struct mcp2210_device *dev,
				 const struct mcp2210_usb_key_params *params,
	const typeof((*(struct mcp2210_msg*)0).body.get_usb_params) *params_get)
{
	struct mcp2210_usb_key_params tmp;

	/* playing protocol-as-structs screws us here, but we can fix it */
	if (params_get) {
		tmp.vid			= params_get->vid;
		tmp.pid			= params_get->pid;
		tmp.chip_power_option	= params_get->chip_power_option;
		tmp.requested_power	= params_get->requested_power;
		params = &tmp;
	}

	memcpy(&dev->s.usb_key_params, params, sizeof(*params));
	dev->s.have_usb_key_params = 1;
}



/* if the command needs to be repeated, then just set the state to NEW before
 * calling process_commands */
static int ctl_complete_urb(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_device *dev = cmd_head->dev;
	struct mcp2210_cmd_ctl *cmd = (struct mcp2210_cmd_ctl *)cmd_head;

	struct mcp2210_msg *req = &cmd->req;//dev->eps[EP_OUT].buffer;
	struct mcp2210_msg *rep = dev->eps[EP_IN].buffer;
	u8 sub_cmd = req->head.req.xet.sub_cmd;

	/* First we need to convert the correct buffer to the endianness */
	switch (rep->cmd) {
	case MCP2210_CMD_GET_NVRAM:
	case MCP2210_CMD_GET_SPI_CONFIG:
	case MCP2210_CMD_GET_CHIP_CONFIG:
	case MCP2210_CMD_GET_PIN_DIR:
	case MCP2210_CMD_GET_PIN_VALUE:
		mcp2210_info("flip ep");
		flip_ep_buffer(&dev->eps[EP_IN], false);
		break;
	case MCP2210_CMD_SET_NVRAM:
	case MCP2210_CMD_SET_SPI_CONFIG:
	case MCP2210_CMD_SET_CHIP_CONFIG:
	case MCP2210_CMD_SET_PIN_DIR:
	case MCP2210_CMD_SET_PIN_VALUE:
		mcp2210_info("flip req");
		flip_ctl_cmd_req(cmd, false);
		break;
	default:
		break;
	};

	switch (rep->cmd) {
	/* When retrieving settings, we store a copy of the response body */
	case MCP2210_CMD_GET_NVRAM:
		switch (sub_cmd) {
		case MCP2210_NVRAM_SPI:
			mcp2210_info("MCP2210_CMD_GET_NVRAM: SPI xfer settings");
			store_spi_settings(dev, &rep->body.spi, cmd->pin, true);
			break;
		case MCP2210_NVRAM_CHIP:
			mcp2210_info("MCP2210_CMD_GET_NVRAM: chip settings");
			store_chip_settings(dev, &rep->body.chip, true);
			break;
		case MCP2210_NVRAM_KEY_PARAMS:
			mcp2210_info("MCP2210_CMD_GET_NVRAM: usb key params");
			store_usb_key_params(dev, 0, &rep->body.get_usb_params);
			break;
		case MCP2210_NVRAM_PROD:
		case MCP2210_NVRAM_MFG:
			/* we don't care about these */
		default:
			mcp2210_err("MCP2210_CMD_%cET_NVRAM: unprocessed sub "
				    "command! 0x%02hhx", 'G', sub_cmd);
			break;
		};
		break;

	/* When writing settings, we store a copy of the request body */
	case MCP2210_CMD_SET_NVRAM:
		switch (sub_cmd) {
		case MCP2210_NVRAM_SPI:
			mcp2210_info("MCP2210_CMD_SET_NVRAM: SPI xfer settings");
			store_spi_settings(dev, &req->body.spi, cmd->pin, true);
			break;
		case MCP2210_NVRAM_CHIP:
			mcp2210_info("MCP2210_CMD_SET_NVRAM: chip settings");
			store_chip_settings(dev, &req->body.chip, true);
			//dump_chip_settings(&rep->body.chip);
			break;
		case MCP2210_NVRAM_KEY_PARAMS:
			mcp2210_info("MCP2210_CMD_SET_NVRAM: usb key params");
			store_usb_key_params(dev, &req->body.set_usb_params, 0);
			break;
		case MCP2210_NVRAM_PROD:
		case MCP2210_NVRAM_MFG:
			/* we don't care about these */
		default:
			mcp2210_err("MCP2210_CMD_%cET_NVRAM: unprocessed sub "
				    "command! 0x%02hhx", 'S', sub_cmd);
			break;
		};
		break;

	case MCP2210_CMD_GET_SPI_CONFIG:
		mcp2210_info("MCP2210_CMD_GET_SPI_CONFIG");
		store_spi_settings(dev, &rep->body.spi, cmd->pin, false);
		break;
	case MCP2210_CMD_SET_SPI_CONFIG:
		mcp2210_info("MCP2210_CMD_SET_SPI_CONFIG");
		store_spi_settings(dev, &req->body.spi, cmd->pin, false);
		break;
	case MCP2210_CMD_GET_CHIP_CONFIG:
		mcp2210_info("MCP2210_CMD_GET_CHIP_CONFIG");
		store_chip_settings(dev, &rep->body.chip, false);
		break;
	case MCP2210_CMD_SET_CHIP_CONFIG:
		mcp2210_info("MCP2210_CMD_SET_CHIP_CONFIG");
		store_chip_settings(dev, &req->body.chip, false);
		break;
	case MCP2210_CMD_SPI_CANCEL:
		mcp2210_info("MCP2210_CMD_SPI_CANCEL");
		dev->spi_in_flight = 0;
		break;

	default:
		mcp2210_err("unprocessed command code: 0x%02hhx", rep->cmd);

	};

	return 0;
}

static int ctl_submit_prepare(struct mcp2210_cmd *cmd_head)
{
	const struct mcp2210_cmd_type *type = cmd_head->type;
	struct mcp2210_cmd_ctl *cmd = (struct mcp2210_cmd_ctl *)cmd_head;
	struct mcp2210_device *dev;

	if (!cmd_head || !cmd_head->dev || !type || type != &mcp2210_cmd_type_ctl) {
		trace_printk("something's fucked: %p, %p, %p, %d", cmd_head, cmd_head->dev, type, type != &mcp2210_cmd_type_ctl);
		return -EINVAL;
	}

	dev = cmd_head->dev;

	mcp2210_info();

	if (dev->dead)
		return -ESHUTDOWN;

	mcp2210_info("need MCP endianness...");
	flip_ctl_cmd_req(cmd, true);
	memcpy(dev->eps[EP_OUT].buffer, &cmd->req, sizeof(cmd->req));
	memset(dev->eps[EP_IN].buffer, 0, 64); /* FIXME: eventually remove, not really needed */

	mcp2210_info("cmd->is_mcp_endianness = %hhu", cmd->is_mcp_endianness);

	return 0;
}

void ctl_cmd_init(struct mcp2210_device *dev, struct mcp2210_cmd_ctl *cmd,
		  u8 cmd_code, u8 subcmd_code, void *body, size_t body_size,
		  u8 pin, u8 is_mcp_endianness)
{
	cmd->head.dev = dev;
	cmd->head.type = &mcp2210_cmd_type_ctl;
	mcp2210_init_msg(&cmd->req, cmd_code, subcmd_code, 0, body,
			 body_size, true);
	cmd->head.can_retry = 1;
	cmd->pin = pin;
	cmd->is_mcp_endianness = is_mcp_endianness;

	mcp2210_info("pin = %hhu, is_mcp_endianness = %hhu", cmd->pin, cmd->is_mcp_endianness);

}


/** mcp2210_add_ctl_cmd - add a new mcp2210 control command to the queue
 *
 * Adds the newly allocated and initialized control command to the queue and
 * returns its address.
 */
struct mcp2210_cmd_ctl *mcp2210_alloc_ctl_cmd(struct mcp2210_device *dev,
		u8 cmd_code, u8 subcmd_code, void *body, size_t body_size,
		u8 pin, u8 is_mcp_endianness, gfp_t gfp_flags)
{
	struct mcp2210_cmd_ctl *cmd = mcp2210_alloc_cmd_type(dev,
			struct mcp2210_cmd_ctl, &mcp2210_cmd_type_ctl,
			gfp_flags);

	if (cmd) {
		ctl_cmd_init(dev, cmd, cmd_code, subcmd_code, body, body_size, pin, is_mcp_endianness);
		/*
		mcp2210_init_msg(&cmd->req, cmd_code, subcmd_code, 0, body,
				 body_size, true);
		cmd->head.can_retry = 1;
		cmd->pin = pin;*/
	}

	return cmd;
}

int mcp2210_prepare_spi(struct mcp2210_device *dev,
			struct mcp2210_cmd_ctl *cmd, u8 pin,
			struct spi_device *spi, u16 len)
{
	const struct mcp2210_spi_config *cfg;
	struct mcp2210_spi_xfer_settings *dest = &cmd->req.body.spi;
	//struct mcp2210_spi_xfer_settings buf; /* 17 bytes, so probably 20 on the stack */
	u8 i;
	u16 active_cs = 0;
	u32 bitrate;

	BUG_ON(!dev);
	BUG_ON(!dev->config);
	BUG_ON(pin > 8);

	if (!dev->config)
		return -EPERM;

	ctl_cmd_init(dev, cmd, MCP2210_CMD_SET_SPI_CONFIG, 0, NULL, 0, pin, true);

	if (spi) {
		BUG_ON(dev->spi_master != spi->master);

		if (spi->chip_select != pin) {
			mcp2210_err("You can't change the chip select "
				    "(expected %u, got %u)", pin,
				    spi->chip_select);
			return -EINVAL;
		}
	}

	for (i = 0; i < 9; ++i) {
		if (dev->config->pins[i].mode == MCP2210_PIN_SPI) {
			u16 cs = !(dev->config->pins[i].body.spi.mode & SPI_CS_HIGH);
			if (pin == i) {
				if (spi)
					cs = !!(spi->mode & SPI_CS_HIGH);
				else
					cs = !cs;
			}

			active_cs |= cs << i;
		}
	}

	cfg = &dev->config->pins[pin].body.spi;

	if (spi) {
		bitrate = spi->max_speed_hz;
		/* "mode" for the mcp2210 means "spi mode" */
		dest->mode	= spi->mode & SPI_MODE_3;
	} else {
		bitrate = cfg->max_speed_hz;
		dest->mode	= cfg->mode;
	}
	if (bitrate < MCP2210_MIN_SPEED)
		bitrate = MCP2210_MIN_SPEED;
	else if (bitrate > MCP2210_MAX_SPEED)
		bitrate = MCP2210_MAX_SPEED;

	dest->bitrate		= cpu_to_le32(bitrate);
	dest->idle_cs		= cpu_to_le16(0x01ff & (~(active_cs)));
	dest->active_cs		= cpu_to_le16(active_cs);
	dest->cs_to_data_delay	= cpu_to_le16(cfg->cs_to_data_delay);
	dest->last_byte_to_cs_delay = cpu_to_le16(cfg->last_byte_to_cs_delay);
	dest->delay_between_bytes = cpu_to_le16(cfg->delay_between_bytes);
	dest->bytes_per_trans	= cpu_to_le16(len);

	return 0;
}

static u16 calculate_active_cs(const struct mcp2210_device *dev, const struct spi_device *spi, u8 pin)
{
	const struct mcp2210_pin_config *pin_cfg;
	u8 i;
	u16 active_cs = 0;

	for (i = 0; i < 9; ++i) {
		pin_cfg = &dev->config->pins[i];
		if (pin_cfg->mode == MCP2210_PIN_SPI) {
			u16 cs = !(pin_cfg->body.spi.mode & SPI_CS_HIGH);
			if (pin == i) {
				if (spi)
					cs = !!(spi->mode & SPI_CS_HIGH);
				else
					cs = !cs;
			}

			active_cs |= cs << i;
		}
	}

	return active_cs;
}

void calculate_spi_settings(struct mcp2210_spi_xfer_settings *dest,
			    const struct mcp2210_device *dev,
			    const struct spi_device *spi,
			    const struct spi_transfer *xfer, u8 pin)
{
	const struct mcp2210_spi_xfer_settings *cur_settings;
	u16 active_cs;

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		BUG_ON(!dev);
		BUG_ON(!dev->config);
		BUG_ON(!spi);
		BUG_ON(!xfer);
		BUG_ON(dev->spi_master != spi->master);
		BUG_ON(pin > 8);
	}

	cur_settings = (pin == dev->s.cur_spi_config) ? &dev->s.spi_settings : NULL;

	/* per-xfer takes precedence */
	if (xfer->speed_hz)
		dest->bitrate = xfer->speed_hz;
	else
		dest->bitrate = spi->max_speed_hz;

	active_cs		= calculate_active_cs(dev, spi, pin);
	dest->idle_cs		= 0x01ff & (~(active_cs));
	dest->active_cs		= active_cs;
	dest->bytes_per_trans	= xfer->len;
	/* "mode" for the mcp2210 means "spi mode" only */
	dest->mode		= spi->mode & SPI_MODE_3;

	if (cur_settings) {
		dest->cs_to_data_delay		= cur_settings->cs_to_data_delay;
		dest->last_byte_to_cs_delay	= cur_settings->last_byte_to_cs_delay;
		dest->delay_between_bytes	= cur_settings->delay_between_bytes;
	} else {
		const struct mcp2210_spi_config *cfg = &dev->config->pins[pin].body.spi;
		dest->cs_to_data_delay		= cfg->cs_to_data_delay;
		dest->last_byte_to_cs_delay	= cfg->last_byte_to_cs_delay;
		dest->delay_between_bytes	= cfg->delay_between_bytes;
	}


}


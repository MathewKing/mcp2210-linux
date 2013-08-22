/*
 *  MCP2210 driver spi layer
 *
 *  Copyright (c) 2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
 *		  2013 Daniel Santos <daniel.santos@pobox.com>
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

#include <linux/spi/spi.h>
#include <linux/version.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"

/* The non-queued mechanism will supposedly be phased out in the future.
 * However, we don't get any benefit from the new API since we just queue
 * a command (in our own queue) when we get a new message anyway. Thus, when/if
 * the old SPI transfer() mechanism is phased out, we can modify the below
 * KERNEL_VERSION() check.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,99,0)
# define USE_SPI_QUEUE 1
#endif

static int spi_submit_prepare(struct mcp2210_cmd *cmd_head);
static int spi_complete_urb(struct mcp2210_cmd *cmd_head);
static int spi_mcp_error(struct mcp2210_cmd *cmd_head);
static int spi_complete_cmd(struct mcp2210_cmd *cmd_head, void *context);

const struct mcp2210_cmd_type mcp2210_cmd_type_spi = {
	.id = MCP2210_CMD_TYPE_SPI,
	.submit_prepare = spi_submit_prepare,
	.complete_urb = spi_complete_urb,
	.mcp_error = spi_mcp_error,
	.dump = dump_cmd_spi,
	.desc = "spi"
};

static inline struct mcp2210_device *mcp2210_spi2dev(struct spi_device *spi)
{
	return *((struct mcp2210_device **)spi_master_get_devdata(spi->master));
}

static inline struct mcp2210_device *mcp2210_spi_master2dev(struct spi_master *master)
{
	return *((struct mcp2210_device **)spi_master_get_devdata(master));
}

/******************************************************************************
 * SPI Master funtions
 */

static void mcp2210_spi_cleanup(struct spi_device *spi);
static int mcp2210_spi_setup(struct spi_device *spi);
#ifndef USE_SPI_QUEUE
static int transfer(struct spi_device *spi, struct spi_message *msg);
#else
static int prepare_transfer_hardware(struct spi_master *master);
static int transfer_one_message(struct spi_master *master, struct spi_message *msg);
static int unprepare_transfer_hardware(struct spi_master *master);
#endif /* USE_SPI_QUEUE */

/**
 * mcp2210_spi_probe -
 * @dev:
 */
// may sleep
int mcp2210_spi_probe(struct mcp2210_device *dev) {
	struct spi_master *master; /* just a local for code brievity */
	int ret;
	unsigned i;

	mcp2210_info("mcp2210_spi_probe\n");

	BUG_ON(!dev);
	BUG_ON(!dev->config);
	BUG_ON(dev->spi_master);

	if (!dev || !dev->config || dev->spi_master)
		return -EINVAL;

	master = spi_alloc_master(&dev->udev->dev, sizeof(void*));
	if (!master)
		return -ENOMEM;

	/* we only need a pointer to the struct mcp2210_device */
	*((struct mcp2210_device **)spi_master_get_devdata(master)) = dev;

	//master->max_speed_hz = MCP2210_MAX_SPEED;
	master->bus_num = -1;
	master->num_chipselect = MCP2210_NUM_PINS;
	/* unused: master->dma_alignment */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_3WIRE;
	/* TODO: what's bits_per_word_mask for? */
	master->flags = 0;
	/* unused: master->bus_lock_spinlock
	 * unused: master->bus_lock_mutex
	 * unused: master->bus_lock_flag
	 */
	master->setup = mcp2210_spi_setup;
	master->cleanup = mcp2210_spi_cleanup;
#ifndef USE_SPI_QUEUE
	master->transfer = transfer;
#else
	master->transfer = NULL;
	master->prepare_transfer_hardware = prepare_transfer_hardware;
	master->transfer_one_message = transfer_one_message;
	master->unprepare_transfer_hardware = unprepare_transfer_hardware;
#endif

	ret = spi_register_master(master);

	if (ret) {
		spi_master_put(master);
		goto error0;
	}

	for (i = 0; i < MCP2210_NUM_PINS; ++i) {
		struct mcp2210_pin_config *cfg = &dev->config->pins[i];
		struct spi_device *chip;
		const char *modalias;

		if (cfg->mode != MCP2210_PIN_SPI) {
			dev->chips[i] = NULL;
			continue;
		}

		chip = spi_alloc_device(master);
		if (!chip) {
			ret = -ENOMEM;
			goto error1;
		}

		chip->max_speed_hz  = cfg->spi.max_speed_hz;
		chip->chip_select   = i;
		chip->mode	    = cfg->spi.mode;
		chip->bits_per_word = cfg->spi.bits_per_word;
		/* unused: chip->cs_gpio
		 * unused: chip->irq
		 * unused: chip->controller_state
		 * unused: chip->controller_data
		 */
		modalias = cfg->modalias ? cfg->modalias : "spidev";

		WARN_ON(strlen(modalias) >= sizeof(chip->modalias));
		strncpy(chip->modalias, modalias, sizeof(chip->modalias));

		ret = spi_add_device(chip);
		if (ret < 0) {
			spi_dev_put(chip);
			goto error1;
		}

		dev->chips[i] = chip;
	}

	dev->spi_master = master;
	return 0;

error1:
	spi_unregister_master(master);
	memset(dev->chips, 0, sizeof(dev->chips));

error0:
	dev->spi_master = NULL;

	return ret;
}

void mcp2210_spi_remove(struct mcp2210_device *dev)
{
	mcp2210_debug("mcp2210_spi_remove\n");

	if (!dev || !dev->spi_master)
		return;

	spi_unregister_master(dev->spi_master);

	dev->spi_master = NULL;
}


/* cleanup for this SPI device (not the SPI master) */
static void mcp2210_spi_cleanup(struct spi_device *spi)
{
	struct mcp2210_device *dev = mcp2210_spi2dev(spi);

	mcp2210_info("pin %d", spi->chip_select);

	/* Do we have any cleanup to do? */
}

static int validate_speed(struct mcp2210_device *dev, u32 bitrate)
{
	if (bitrate < MCP2210_MIN_SPEED || bitrate > MCP2210_MAX_SPEED) {
		mcp2210_err("The requested speed of %uHz is not supported "
			    "(must be in the range of 1500Hz and 12MHz)",
			    bitrate);
		return -EINVAL;
	}

	return 0;
}

/* must hold dev_spinlock */
/**
 * is_spi_in_flight - determine if a non-failed SPI message is in progress on
 * 		      the specified pin
 *
 * This will not return true if an SPI transfer is "in progress" or may be in
 * progress on the chip due to some communication failure, but only if a real,
 * live non-failed transfer is in progress.
 */
static int is_spi_in_flight(const struct mcp2210_device *dev, u8 pin)
{
	const struct mcp2210_cmd *cmd_head = dev->cur_cmd;

	if (cmd_head && cmd_head->type == &mcp2210_cmd_type_spi) {
		const struct mcp2210_cmd_spi_msg *cmd = (void *)cmd_head;

		return cmd->spi_in_flight && cmd->spi->chip_select == pin;
	}
	return 0;
}

/* may sleep */
static int mcp2210_spi_setup(struct spi_device *spi)
{
	struct mcp2210_device *dev = mcp2210_spi2dev(spi);
	unsigned long irqflags;
	u8 pin = spi->chip_select;
	int ret = 0;

	mcp2210_info("spi_in_flight: %d, cur_spi_config: %d",
		     dev->spi_in_flight, dev->s.cur_spi_config);

	if (dump_commands)
		dump_spi_device(KERN_INFO, 0, "mcp2210_spi_setup: "
				"spi_device = ", spi);

	if (validate_speed(dev, spi->max_speed_hz))
		return -EINVAL;

	if (spi->bits_per_word != 8) {
		mcp2210_err("MCP2210 only supports 8 bits per word, got "
			    "request for %hhu", spi->bits_per_word);
		return -EINVAL;
	}

	if (pin > 8) {
		mcp2210_err("spi->chip_select must be in the range of 0 - 8, "
			    "got %hu", pin);
		return -EINVAL;
	}

	spin_lock_irqsave(&dev->dev_spinlock, irqflags);

	/* We must error if a transfer is in progress on the same SPI device */
	if (is_spi_in_flight(dev, pin)) {
		mcp2210_err("SPI message in progress");
		ret = -EBUSY;
	}

	spin_unlock_irqrestore(&dev->dev_spinlock, irqflags);
	return ret;
}

static int queue_msg(struct mcp2210_device *dev, struct spi_message *msg,
		     gfp_t gfp_flags)
{
	u8 pin = msg->spi->chip_select;
	struct mcp2210_cmd_spi_msg *cmd;
	struct list_head *pos;
	struct mcp2210_pin_config *pin_config = &dev->config->pins[pin];
	struct spi_transfer *first_xfer = list_entry(msg->transfers.next,
						     struct spi_transfer,
						     transfer_list);
	int ret;

	mcp2210_info("Start new transfer (pin %d)\n", pin);

	/* debug-only sanity checks */
	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		if (pin_config->mode != MCP2210_PIN_SPI) {
			mcp2210_err("Attempt to SPI on non-spi pin!");
			return -EINVAL;
		}

		if (list_empty(&msg->transfers)) {
			mcp2210_err("empty transfer list");
			return -EINVAL;
		}
	}

	list_for_each(pos, &msg->transfers) {
		struct spi_transfer *xfer = list_entry(
				pos,
				struct spi_transfer,
				transfer_list);

		/* debug-only sanity checks */
		if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
			if (!(xfer->tx_buf || xfer->rx_buf)) {
				mcp2210_err("spi_transfer w/o tx or rx buffer");
				return -EINVAL;
			}
		}

		if (xfer->cs_change) {
			mcp2210_warn("unsupported: spi_transfer.cs_change = 1");
			return -EINVAL;
		}

		if (xfer->bits_per_word && xfer->bits_per_word != 8) {
			mcp2210_warn("unsupported: spi_transfer.bits_per_word "
				     "= %hhu", xfer->bits_per_word);
			return -EINVAL;
		}

		if (validate_speed(dev, xfer->speed_hz))
			return -EINVAL;
	}

	cmd = mcp2210_alloc_cmd_type(dev, struct mcp2210_cmd_spi_msg,
				     &mcp2210_cmd_type_spi, gfp_flags);

	if (!cmd)
		return -ENOMEM;

	cmd->head.complete = spi_complete_cmd;
	cmd->head.context = NULL;
	cmd->spi = msg->spi;
	cmd->msg = msg;
	cmd->xfer = first_xfer;
	/* not needed (kzalloc)
	cmd->tx_pos = 0;
	cmd->rx_pos = 0;
	cmd->tx_bytes_in_process = 0;
	cmd->xfer_settings_change = 0;
	cmd->kill = 0;
	cmd->ctl_cmd = NULL;
	*/

	ret = mcp2210_add_or_free_cmd(&cmd->head);
	if (ret)
		return ret;

	return process_commands(dev, gfp_flags, 0);
}

#ifndef USE_SPI_QUEUE
/* will not sleep */
static int transfer(struct spi_device *spi, struct spi_message *msg)
{
	return queue_msg(mcp2210_spi2dev(spi), msg, GFP_ATOMIC);
}

#else

static int prepare_transfer_hardware(struct spi_master *master)
{
	struct mcp2210_device *dev = mcp2210_spi_master2dev(master);
	mcp2210_info();

	return 0;
}

/* may sleep */
static int transfer_one_message(struct spi_master *master,
				struct spi_message *msg)
{
	return queue_msg(mcp2210_spi_master2dev(master), msg, GFP_KERNEL);
}

static int unprepare_transfer_hardware(struct spi_master *master)
{
	struct mcp2210_device *dev = mcp2210_spi_master2dev(master);
	mcp2210_info();

	return 0;
}
#endif /* USE_SPI_QUEUE */

/******************************************************************************
 * SPI Message command functions
 */

static int spi_submit_prepare(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_cmd_spi_msg *cmd = (void *)cmd_head;
	struct mcp2210_device *dev;
	struct mcp2210_msg *req;
	u8 pin;

	BUG_ON(!cmd_head->dev);
	dev = cmd_head->dev;
	pin = cmd->spi->chip_select;
	req = dev->eps[EP_OUT].buffer;

	mcp2210_info("pin %hhu\n", pin);

	BUG_ON(pin > 8);
	BUG_ON(dev->config->pins[pin].mode != MCP2210_PIN_SPI);


	/* If we're in the middle of an SPI transfer, we can't do control
	 * commands. Thus, we should never have a control command going on at
	 * the saime time we're in the middle of an SPI transfer.  Also, SPI
	 * transfers cannot be retried if there's a failure or stalled URB at
	 * the USB level.
	 */
	if (cmd->spi_in_flight)
		BUG_ON(cmd->ctl_cmd);
	else {
		struct mcp2210_spi_xfer_settings needed;
		int need_spi_settings;

		/* If we haev an active control command it probably means that
		 * this is a retry, so we need to finish that up. */
		if (cmd->ctl_cmd) {
			struct mcp2210_cmd *cc;
submit_ctl_cmd:
			cc = &cmd->ctl_cmd->head;
			cmd->head.can_retry = 1;

			mcp2210_info("----SUBMITING CONTROL COMMAND----");
			return cc->type->submit_prepare(cc);
		}

		/* the chip may be in the middle of a failed SPI transfer, so
		 * we have to kill that */
		if (unlikely(dev->spi_in_flight)) {
			mcp2210_warn("***** old SPI message still in-flight, "
				     "killing...");
			ctl_cmd_init(dev, &dev->ctl_cmd, MCP2210_CMD_SPI_CANCEL,
				     0, NULL, 0, false);
			cmd->ctl_cmd = &dev->ctl_cmd;
			goto submit_ctl_cmd;
		}

		need_spi_settings = 1;
		mcp2210_debug("dev->s.cur_spi_config = %d",
			      dev->s.cur_spi_config);
		dump_spi_xfer_settings(KERN_DEBUG, 0, "dev->s.spi_settings = ",
				       &dev->s.spi_settings);
		calculate_spi_settings(&needed, dev, cmd->spi, cmd->xfer, pin);


		/* If this is the pin we are currently configured to SPI on,
		 * then let's see if there's any difference in the SPI
		 * settings */
		if (pin == dev->s.cur_spi_config) {
			/* if nothing changed, then then do the SPI xfer */
			if (!compare_spi_settings(&needed, &dev->s.spi_settings))
				need_spi_settings = 0;
			else if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
				mcp2210_debug("SPI transfer settings didn't "
					      "match");
				dump_spi_xfer_settings(KERN_DEBUG, 0,
						       "needed = ", &needed);
				dump_spi_xfer_settings(KERN_DEBUG, 0,
						       "dev->spi_settings = ",
						       &dev->s.spi_settings);
			}
		}

		/* TODO: still missing cmd->spi->mode & (~(SPI_MODE_3 | SPI_CS_HIGH)) */
		if (need_spi_settings) {
			cmd->ctl_cmd = &dev->ctl_cmd;
			mcp2210_info("Settings SPI Transfer Settings");
			ctl_cmd_init(dev, cmd->ctl_cmd,
				     MCP2210_CMD_SET_SPI_CONFIG, 0, &needed,
				     sizeof(needed), false);
			cmd->ctl_cmd->pin = pin;
			goto submit_ctl_cmd;
		}
	}

	/* If the last attempt to tx failed, we need to rewind tx_pos */
	if (cmd->tx_bytes_in_process) {
		cmd->tx_pos -= cmd->tx_bytes_in_process;
		cmd->tx_bytes_in_process = 0;
	}

	/* Write spi transfer command */
	if (cmd->tx_pos < cmd->xfer->len) {
		unsigned len = cmd->xfer->len - cmd->tx_pos;
		const void *start = cmd->xfer->tx_buf + cmd->tx_pos;

		if(len > MCP2210_BUFFER_SIZE - 4)
			len = MCP2210_BUFFER_SIZE - 4;

		mcp2210_init_msg(req, MCP2210_CMD_SPI_TRANSFER, len,
				 0, start, len, true);

		cmd->tx_bytes_in_process = len;

		mcp2210_debug("sending %u bytes", len);
	} else {
		mcp2210_init_msg(req, MCP2210_CMD_SPI_TRANSFER, 0,
				 0, NULL, 0, true);
		mcp2210_debug("sending empty SPI message");
	}

	dev->spi_in_flight = 1;
	cmd->spi_in_flight = 1;
	cmd->head.can_retry = 0;

	/* FIXME: dump message params */
	return 0;
}

static int spi_complete_urb(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_cmd_spi_msg *cmd = (void *)cmd_head;
	struct mcp2210_device *dev;
	struct mcp2210_msg *rep;
	const struct mcp2210_pin_config_spi *cfg;
	unsigned bytes_pending;
	long expected_time_usec = 0;
	u8 pin;
	u8 len;

	dev = cmd_head->dev;
	pin = cmd->spi->chip_select;

	if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
		BUG_ON(!cmd_head->dev);
		BUG_ON(pin > 8);
		BUG_ON(dev->config->pins[pin].mode != MCP2210_PIN_SPI);
	}

	mcp2210_info();

	if(cmd->ctl_cmd) {
		struct mcp2210_cmd *cc = &cmd->ctl_cmd->head;
		int ret;
		ret = cc->type->complete_urb(cc);

		if (IS_ENABLED(CONFIG_MCP2210_DEBUG)) {
			mcp2210_info("----CONTROL COMMAND RESPONSED----");

			/* get the dump function to print the response */
			cc->state = MCP2210_STATE_COMPLETE;
			mcp2210_debug("dev->s.cur_spi_config = %d",
				      dev->s.cur_spi_config);
			dump_spi_xfer_settings(KERN_INFO, 0, "spi settings "
					       "now: ", &dev->s.spi_settings);
		}

		if (ret)
			BUG(); // oops
		else
			cmd->ctl_cmd = NULL;

		return -EAGAIN;
	}

	rep = dev->eps[EP_IN].buffer;
	len = rep->head.rep.spi.size;
	cfg = &dev->config->pins[pin].spi;


	/* by "in process", we mean really sent to the MCP2210 */
	cmd->tx_pos += cmd->tx_bytes_in_process;
	bytes_pending = cmd->tx_bytes_in_process - len;
	cmd->tx_bytes_in_process = 0;

	mcp2210_info("pin %hhu\n", pin);
	//print_mcp_msg(cmd->head.dev, KERN_DEBUG, "SPI response: ", rep);

	if (len) {
		/* There is data to receive */
		if(cmd->xfer->rx_buf)
			memcpy(cmd->xfer->rx_buf + cmd->rx_pos, rep->body.raw,
			       len);

		cmd->rx_pos += len;
		cmd->msg->actual_length += len;
	}

	if(cmd->rx_pos == cmd->xfer->len) {
		struct list_head *next = cmd->xfer->transfer_list.next;
		/* Transfer is done move next */
		mcp2210_info("All rx bytes read\n");
		dev->spi_in_flight = 0;
		cmd->spi_in_flight = 0;

		if (cmd->msg->transfers.prev == &cmd->xfer->transfer_list) {
			mcp2210_info("Command complete");
			cmd->msg->status = cmd->head.status = 0;
			//spi_finalize_current_message(dev->spi_master);
			return 0;
		}

		mcp2210_info("Starting next spi_transfer...");
		/* TODO: check if we have a new length and re-adjust if needed */
		cmd->xfer = list_entry(next, struct spi_transfer,
				       transfer_list);
		cmd->tx_pos = 0;
		cmd->rx_pos = 0;
		cmd->tx_bytes_in_process = 0;
		cmd->spi_in_flight = 0;
		cmd->busy_count = 0;
		cmd->ctl_cmd = NULL;

		expected_time_usec = cfg->delay_between_xfers;
		cmd->head.delay_until = jiffies + msecs_to_jiffies(1);
		cmd->head.delayed = 1;
	} else {
		/* try to determine when the device will be ready */
		expected_time_usec = 100ul * cfg->last_byte_to_cs_delay
			+ bytes_pending * dev->s.spi_delay_per_kb / 1024ul;
		if (rep->head.rep.spi.spi_status == 0x20)
			expected_time_usec += 100ul * cfg->cs_to_data_delay;
	}

	mcp2210_info("expected_time_usec: %lu (%lu jiffies)",
		     expected_time_usec, usecs_to_jiffies(expected_time_usec));

	cmd->head.delay_until = dev->eps[EP_OUT].submit_time
					+ usecs_to_jiffies(expected_time_usec);
	cmd->head.delayed = 1;
	return -EAGAIN;
}

static int spi_mcp_error(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_cmd_spi_msg *cmd = (void *)cmd_head;
	struct mcp2210_device *dev = cmd->head.dev;

	/* TODO: use delayed work? */
	mcp2210_warn("cmd->busy_count %u\n", cmd->busy_count);

	++cmd->busy_count;
	if (cmd->busy_count < 32) {
		/* hmm, hopefully shoudn't happen */
		cmd->head.delay_until = jiffies + 1;
		cmd->head.delayed = 1;
		return -EAGAIN;
	} else
		return -EBUSY;
}

static int spi_complete_cmd(struct mcp2210_cmd *cmd_head, void *context)
{
	struct mcp2210_cmd_spi_msg *cmd = (void*)cmd_head;
	struct spi_message *msg = cmd->msg;

	msg->status = cmd->head.status;

#ifndef USE_SPI_QUEUE
	if (msg->complete)
		msg->complete(msg->context);
#else
	spi_finalize_current_message(cmd->head.dev->spi_master);
#endif
	return 0;
}


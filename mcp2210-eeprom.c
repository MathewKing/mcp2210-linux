/*
 *  MCP2210 eeprom management
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

#include <linux/usb.h>
#include <linux/export.h>
#include <linux/bug.h>
#include <linux/slab.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"

#ifdef CONFIG_MCP2210_EEPROM

static int eeprom_submit_prepare(struct mcp2210_cmd *cmd_head);
static int eeprom_complete_urb(struct mcp2210_cmd *cmd_head);

const static struct mcp2210_cmd_type mcp2210_cmd_type_eeprom = {
	.id = MCP2210_CMD_TYPE_EEPROM,
	.submit_prepare = eeprom_submit_prepare,
	.complete_urb = eeprom_complete_urb,
	.dump = dump_cmd_eeprom,
	.desc = "eeprom",
};

static inline int is_eeprom_range_valid(u8 addr, u16 size)
{
	int ret = size && (size + (u16)addr) <= 0x100;
	if (unlikely(!ret))
		printk(KERN_ERR "invalid address, size or both. "
		       "addr=0x%02hhx, size=0x%02x, sum=0x%02x", addr, size,
		       (size + (unsigned)addr));
	return ret;
}

static inline int get_eeprom_byte_status(struct mcp2210_device *dev, u8 addr)
{
	const unsigned shift = (addr % 4) * 2;

	BUG_ON(!is_eeprom_range_valid(addr, 1));
	return (dev->eeprom_state[addr / 4] >> shift) & 3;
}

static inline void set_eeprom_byte_status(struct mcp2210_device *dev, u8 addr,
					  int status)
{
	u8 * const p = &dev->eeprom_state[addr / 4];
	const unsigned shift = (addr % 4) * 2;

	BUG_ON(!is_eeprom_range_valid(addr, 1));
	BUILD_BUG_ON(status != (status & 3));

	*p &= ~(3 << shift);
	*p |= status << shift;
}

static int queue_eeprom_cmd(struct mcp2210_device *dev, u8 op, u8 addr,
			    u16 size, mcp2210_complete_t complete,
			    void *context, gfp_t gfp_flags)
{
	struct mcp2210_cmd_eeprom *cmd = mcp2210_alloc_cmd_type(dev,
			struct mcp2210_cmd_eeprom, &mcp2210_cmd_type_eeprom,
			gfp_flags);
	//unsigned long irqflags;
	int ret;

	mcp2210_debug("dev: %p, op: 0x%02hhx, addr: 0x%02hhx, size: 0x%02hhx",
		     dev, op, addr, size);
	if (!cmd)
		return -ENOMEM;

	if (unlikely(!(op == MCP2210_CMD_READ_EEPROM
		    || op == MCP2210_CMD_WRITE_EEPROM))) {
		mcp2210_err("operation not one of MCP2210_CMD_READ_EEPROM or "
			    "MCP2210_CMD_WRITE_EEPROM");
		return -EINVAL;
	}

	cmd->head.can_retry = 1;
	cmd->op = op;
	cmd->addr = addr;
	cmd->size = size;
	cmd->zero_tail = 1;
	cmd->head.complete = complete;
	cmd->head.context = context;

//	dump_cmd_eeprom(KERN_WARNING, 0, "eeprom_cmd = ", &cmd->head);

	ret = mcp2210_add_cmd(&cmd->head, true);
	if (!dev->cur_cmd)
		process_commands(dev, GFP_ATOMIC, 0);

	return ret;
}

/*
 * Populates addr with the next address for this command to process.
 *
 * Returns zero if there are no (more) addresses that need to be processed,
 * non-zero otherwise.
 */
static int next_addr(struct mcp2210_cmd_eeprom *cmd, u8 *addr)
{
	const u16 end = cmd->addr + cmd->size;
	u16 i;

	for (i = cmd->addr; i < end; ++i) {
		switch (get_eeprom_byte_status(cmd->head.dev, i)) {
		case MCP2210_EEPROM_READ:
			continue;

		case MCP2210_EEPROM_DIRTY:
			/* We wont write dirty values on a read command, but
			 * there should be an upcomming write command in the
			 * queue.
			 */
			if (cmd->op == MCP2210_CMD_READ_EEPROM)
				continue;

			/* fall-through */
		case MCP2210_EEPROM_UNREAD:
		case MCP2210_EEPROM_READ_PENDING:
			if (unlikely(i > 0xff)) {
				BUG();
				return -EINVAL;
			}

			if (addr)
				*addr = i;
			return 1;

		default:
			BUG();
		};
	}

	return 0;
}

static int eeprom_submit_prepare(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_cmd_eeprom *cmd = (void *)cmd_head;
	struct mcp2210_device *dev = cmd->head.dev;
	u8 addr;
	u8 value;

	mcp2210_debug();

	if (dev->dead)
		return -ESHUTDOWN;

	if (!next_addr(cmd, &addr)) {
		/* it's possible to have nothing to do, like reading values
		 * that have already been read and are cached.
		 */
		cmd->head.state = MCP2210_STATE_COMPLETE;
		return -EALREADY;
	}

	if (cmd->op == MCP2210_CMD_WRITE_EEPROM)
		value = dev->eeprom_cache[addr];
	else
		value = 0;

	/* We only need to zero the tail once.  This should help on operations
	 * over a large range of addresses. */
	mcp2210_init_msg(dev->eps[EP_OUT].buffer, cmd->op, addr, value, NULL,
			 0, cmd->zero_tail);
	cmd->zero_tail = 0;

	return 0;
}

static int eeprom_complete_urb(struct mcp2210_cmd *cmd_head)
{
	struct mcp2210_cmd_eeprom *cmd = (void *)cmd_head;
	struct mcp2210_device *dev = cmd->head.dev;
	struct mcp2210_msg *req = dev->eps[EP_OUT].buffer;
	struct mcp2210_msg *rep = dev->eps[EP_IN].buffer;
	u8 addr = req->head.req.eeprom.addr;

	mcp2210_debug();
	switch (req->cmd) {
	case MCP2210_CMD_READ_EEPROM:
		//mcp2210_debug("read 0x%02hhx = ", addr, dev->eeprom_cache[addr]);
		dev->eeprom_cache[addr] = rep->head.rep.eeprom.value;

		/* fall-through */
	case MCP2210_CMD_WRITE_EEPROM:
		mcp2210_debug("set_eeprom_byte_status: 0x%02hhx, "
			      "MCP2210_EEPROM_READ", addr);
		set_eeprom_byte_status(dev, addr, MCP2210_EEPROM_READ);
		mcp2210_debug("get_eeprom_byte_status(dev, addr) = %d",
			      get_eeprom_byte_status(dev, addr));

		break;
	default:
		BUG();
	};

	/* find out if we have more addresses to do in this command */
	if (next_addr(cmd, &addr))
		return -EAGAIN;

	return 0;
}

int mcp2210_eeprom_read(struct mcp2210_device *dev, u8 *dest, u8 addr,
			u16 size, mcp2210_complete_t complete, void *context,
			gfp_t gfp_flags)
{
	const u16 end = size + addr;
	unsigned long irqflags;
	u16 i;
	int ready = 1;
	int ret;

	mcp2210_debug();

	if (!dev || !complete)
		return -EINVAL;

	if (!is_eeprom_range_valid(addr, size)) {
		mcp2210_err("invalid range, addr=%hhu, size=%hu\n", addr, size);
		return -EINVAL;
	}

	spin_lock_irqsave(&dev->eeprom_spinlock, irqflags);
	for (i = addr; i != end; ++i) {
		int status = get_eeprom_byte_status(dev, i);
		switch (status) {
		case MCP2210_EEPROM_UNREAD:
			set_eeprom_byte_status(dev, i, MCP2210_EEPROM_READ_PENDING);

			/* intentional fall-through */
		case MCP2210_EEPROM_READ_PENDING:
			ready = 0;
			break;

		default:
			/* no action on anything else */
			break;
		}
	}

	/* if all values are already read we copy it immediately to the
	 * caller's buffer (if one was supplied) */
	if (ready && dest)
		memcpy(dest, &dev->eeprom_cache[addr], size);

	spin_unlock_irqrestore(&dev->eeprom_spinlock, irqflags);

	if (ready && dest)
		return 0;

	/* otherwise, we queue the command */
	ret = queue_eeprom_cmd(dev, MCP2210_CMD_READ_EEPROM, addr, size,
			       complete, context, gfp_flags);

	if (ret) {
		mcp2210_err("failed to add command to queue, %de\n", ret);
		return ret;
	}

	return -EINPROGRESS;
}

/**
 * mcp2210_eeprom_write
 *
 * Returns:
 * Zero if there is nothing to do, -EINPROGRESS if message is queued or an
 * error.
 */
int mcp2210_eeprom_write(struct mcp2210_device *dev, const u8 *src, u8 addr,
			 u16 size, mcp2210_complete_t complete, void *context,
			 gfp_t gfp_flags)
{
	const u16 end = size + addr;
	unsigned long irqflags;
	const u8 *in = src;
	u16 i;
	int writes_in_progress = 0;
	int ret;

	mcp2210_debug();

	if (!dev || !src || !complete)
		return -EINVAL;

	if (!is_eeprom_range_valid(addr, size))
		return -EINVAL;

	spin_lock_irqsave(&dev->eeprom_spinlock, irqflags);
	for (i = addr; i != end; ++i, ++in) {
		switch (get_eeprom_byte_status(dev, i)) {
		/* cached value is current */
		case MCP2210_EEPROM_READ:
			/* if value being written is the same, just ignore it */
			if (dev->eeprom_cache[i] == *in)
				continue;
			break;

		case MCP2210_EEPROM_DIRTY:
			/* write already pending: just change the target value */
			dev->eeprom_cache[i] = *in;
			++writes_in_progress;
			continue;

		default:
			/* read pending or unread, we'll treat the same here */
			break;
		};

		/* all other cases, we queue up a write eeprom command */
		// TODO: queue write request
		dev->eeprom_cache[i] = *in;
		set_eeprom_byte_status(dev, i, MCP2210_EEPROM_DIRTY);
		++writes_in_progress;
	}
	spin_unlock_irqrestore(&dev->eeprom_spinlock, irqflags);

	if (!writes_in_progress)
		return 0;

	ret = queue_eeprom_cmd(dev, MCP2210_CMD_WRITE_EEPROM, addr, size,
			       complete, context, gfp_flags);
	if (ret)
		return ret;

	return -EINPROGRESS;
}

#endif /* CONFIG_MCP2210_EEPROM */

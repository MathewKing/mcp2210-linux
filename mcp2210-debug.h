/*
 * Verbose Debug functions for MCP2210 driver & libs
 *
 * Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
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


#ifndef _MCP2210_DEBUG_H
#define _MCP2210_DEBUG_H

#ifdef __KERNEL__
# include <linux/printk.h>
# include <linux/device.h>
#endif

#include "mcp2210.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __KERNEL__

#define _mcp2210_log(level, fmt, ...)					\
	do {								\
		if ((level[1] - '0') <= debug_level)			\
			dev_printk(level, &dev->udev->dev,		\
				   "%s: " fmt, __func__, ##__VA_ARGS__);\
	} while(0)

/* TODO: we need to submit a cleaned up version of this to mainline for
 * compiler*.h */
#define warn_on_non_const(value)					\
	do {								\
		extern void not_const_warn(void) __compiletime_warning(	\
			#value " is not a compile-time constant, this " \
			"will cause some bloating of generated code");	\
									\
		if (!__builtin_constant_p(value))			\
			not_const_warn();				\
	} while(0)

/**
 * mcp2210_log - logs messages
 *
 * Logs messages but assures that debug messages are completely compiled out
 * when CONFIG_MCP2210_DEBUG is not enabled
 */
#define mcp2210_log(level, fmt, ...)					\
	do {								\
		const char _level = level[1] - '0';			\
		const char _dbg = KERN_DEBUG[1] - '0';			\
									\
		warn_on_non_const(_level);				\
									\
		/* compile-out debug messages unless			\
		 * CONFIG_MCP2210_DEBUG is enabled */			\
		if (!IS_ENABLED(CONFIG_MCP2210_DEBUG) && _level == _dbg)\
			break;						\
									\
		_mcp2210_log(level, fmt, ##__VA_ARGS__);		\
	} while(0)

#define mcp2210_emerg(fmt, ...)	mcp2210_log(KERN_EMERG,	fmt, ##__VA_ARGS__)
#define mcp2210_alert(fmt, ...)	mcp2210_log(KERN_ALERT,	fmt, ##__VA_ARGS__)
#define mcp2210_crit(fmt, ...)	mcp2210_log(KERN_CRIT,	fmt, ##__VA_ARGS__)
#define mcp2210_err(fmt, ...)	mcp2210_log(KERN_ERR,	fmt, ##__VA_ARGS__)
#define mcp2210_warn(fmt, ...)	mcp2210_log(KERN_WARNING,fmt, ##__VA_ARGS__)
#define mcp2210_notice(fmt, ...)mcp2210_log(KERN_NOTICE,fmt, ##__VA_ARGS__)
#define mcp2210_info(fmt, ...)	mcp2210_log(KERN_INFO,	fmt, ##__VA_ARGS__)
#define mcp2210_debug(fmt, ...)	mcp2210_log(KERN_DEBUG,	fmt, ##__VA_ARGS__)

#ifdef CONFIG_MCP2210_DEBUG
# define MCP_ASSERT(cond) BUG_ON(!(cond))
#else
# define MCP_ASSERT(cond) do{}while(0)
#endif

#ifdef CONFIG_MCP2210_DEBUG_VERBOSE
void dump_dev(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_device *dev);
void dump_ep(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_endpoint *ep);
void dump_cmd_head(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_cmd *cmd);
void dump_cmd_ctl(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_cmd *cmd_head);
void dump_cmd_spi(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_cmd *cmd_head);
void dump_cmd_eeprom(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_cmd *cmd_head);
void dump_spi_message(
	const char *level, unsigned indent, const char *start,
	const struct spi_message *msg);
void dump_spi_transfer(
	const char *level, unsigned indent, const char *start,
	const struct spi_transfer *xfer);
void dump_spi_device(
	const char *level, unsigned indent, const char *start,
	const struct spi_device *spi_dev);
#else
# define dump_dev		_dump_nothing
# define dump_ep		_dump_nothing
# define dump_cmd_head		_dump_nothing
# define dump_cmd_ctl		_dump_nothing
# define dump_cmd_spi		_dump_nothing
# define dump_cmd_eeprom	_dump_nothing
# define dump_spi_message	_dump_nothing
# define dump_spi_transfer	_dump_nothing
# define dump_spi_device	_dump_nothing

static inline void _dump_nothing(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_cmd *cmd_head)
{
}
#endif

static inline void print_mcp_msg(const char *level, const char *start,
				 const struct mcp2210_msg *data)
{
	print_hex_dump(level, start, DUMP_PREFIX_OFFSET, 16, 1, data, 64, true);
}

void _mcp2210_dump_urbs(struct mcp2210_device *dev, const char *level,
			int urb_mask);
static inline void mcp2210_dump_urbs(struct mcp2210_device *dev,
				     const char *level, int urb_mask)
{
	if (IS_ENABLED(CONFIG_MCP2210_DEBUG) && dump_urbs)
		_mcp2210_dump_urbs(dev, level, urb_mask);
}

void _dump_cmd(const char *level, unsigned indent, const char *start,
	       const struct mcp2210_cmd *cmd_head);
static inline void dump_cmd(const char *level, unsigned indent,
			    const char *start,
			    const struct mcp2210_cmd *cmd_head)
{
	if (IS_ENABLED(CONFIG_MCP2210_DEBUG_VERBOSE) && dump_commands)
		_dump_cmd(level, indent, start, cmd_head);
}


#endif /* __KERNEL__ */

/* both kernel & userspace functions */
#ifdef CONFIG_MCP2210_DEBUG_VERBOSE
const char *get_cmd_str(u8 cmd);
const char *get_sub_cmd_str(u8 sub_cmd);
const char *get_status_str(u8 status);
const char *get_pin_mode_str(u8 mode);
const char *get_eeprom_status_str(u8 mode);
const char *get_cmd_type_str(u8 mode);
const char *get_state_str(u8 mode);
void dump_pin_config(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_pin_config *cfg);
void dump_chip_settings(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_chip_settings *s);
void dump_board_config(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_board_config *bc);
void dump_spi_xfer_settings(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_spi_xfer_settings *s);
void dump_usb_key_params(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_usb_key_params *params);
void dump_state(
	const char *level, unsigned indent, const char *start,
	const struct mcp2210_state *s);
void dump_mcp_msg(
	const char *level, unsigned indent, const char *start,
	struct mcp2210_msg *msg, int is_req);
#else
# define get_cmd_str(value)				while(0){}
# define get_sub_cmd_str(value)				while(0){}
# define get_status_str(value)				while(0){}
# define get_pin_mode_str(value)			while(0){}
# define get_eeprom_status_str(value)			while(0){}
# define get_cmd_type_str(value)			while(0){}
# define get_state_str(value)				while(0){}
# define dump_pin_config(level, indent, start, x)	while(0){}
# define dump_chip_settings(level, indent, start, x)	while(0){}
# define dump_board_config(level, indent, start, x)	while(0){}
# define dump_spi_xfer_settings(level, indent, start, x)while(0){}
# define dump_mcp_msg(level, indent, start, x)		while(0){}
#endif /* CONFIG_MCP2210_DEBUG_VERBOSE */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _MCP2210_DEBUG_H */

/*
 *  MCP 2210 driver for linux
 *
 *  Copyright (c) 2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
 *                2020 Cristian Balint <cristian.balint@gmail.com>
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifndef _MCP2210_H
#define _MCP2210_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/hid.h>

extern int debug;
#define verbose_debug(fmt, args...) \
	if (debug != 0) printk(fmt, ##args)

#define USB_VENDOR_ID_MICROCHIP		0x04d8
#define USB_DEVICE_ID_MCP2210		0x00de

#define MCP2210_BUFFER_SIZE		64
#define MCP2210_MAX_CS			8
#define MCP2210_MAX_SPEED		(12 * 1000 * 1000)

// MCP2210 commands
#define MCP2210_CMD_GET_STATUS		0x10
#define MCP2210_CMD_SPI_CANCEL		0x11
#define MCP2210_CMD_GET_INTERRUPTS	0x12
#define MCP2210_CMD_GET_CHIP_CONFIG	0x20
#define MCP2210_CMD_SET_CHIP_CONFIG	0x21
#define MCP2210_CMD_SET_PIN_VALUE	0x30
#define MCP2210_CMD_GET_PIN_VALUE	0x31
#define MCP2210_CMD_SET_PIN_DIR		0x32
#define MCP2210_CMD_GET_PIN_DIR		0x33
#define MCP2210_CMD_SET_SPI_CONFIG	0x40
#define MCP2210_CMD_GET_SPI_CONFIG	0x41
#define MCP2210_CMD_SPI_TRANSFER	0x42
#define MCP2210_CMD_READ_EEPROM		0x50
#define MCP2210_CMD_WRITE_EEPROM	0x51
#define MCP2210_CMD_SET_NVRAM		0x60
#define MCP2210_CMD_GET_NVRAM		0x61
#define MCP2210_CMD_SEND_PASSWD		0x70
#define MCP2210_CMD_SPI_RELEASE		0x80

// MCP2210 status
#define MCP2210_STATUS_SUCCESS		0x00
#define MCP2210_STATUS_SPI_NOT_OWNED	0xf7
#define MCP2210_STATUS_BUSY		0xf8
#define MCP2210_STATUS_UNKNOWN_CMD	0xf9
#define MCP2210_STATUS_WRITE_FAIL	0xfa
#define MCP2210_STATUS_NO_ACCESS	0xfb
#define MCP2210_STATUS_PERM_LOCKED	0xfc
#define MCP2210_STATUS_BAD_PASSWD	0xfd

struct mcp2210_dev_cfg {
	u8 active;
	u32 bitrate;
	u16 idle_cs;
	u16 active_cs;
	u16 cs_ds_delay;
	u16 cs_ls_delay;
	u16 byte_delay;
	u16 byte_per_spi;
	u8 spi_mode;
};

struct mcp2210_device {
	struct hid_device *hid;
	struct mcp2210_dev_cfg *cfg;
	wait_queue_head_t wait;
	struct mcp2210_command *current_command;
	struct list_head command_list;
	struct mutex command_mutex;
	spinlock_t command_lock;
	struct work_struct command_work;
	u8 requeust_buffer[MCP2210_BUFFER_SIZE];
	void *spi_data;
};

struct mcp2210_command {
	void *data;
	struct mcp2210_device *dev;
	int requests_pending;
	int processing;
	int (*next_request)(void *command_data, u8 *request);
	void (*data_received)(void *command_data, u8 *response);
	void (*interrupted)(void *command_data);
	struct list_head node;
	struct list_head request_list;
};

struct mcp2210_request_list {
	struct hid_report *report;
	struct list_head node;
};

int mcp2210_add_command(struct mcp2210_device *dev, void *cmd_data,
	int (*next_request)(void *command_data, u8 *request),
	void (*data_received)(void *command_data, u8 *response),
	void (*interrupted)(void *command_data));


static inline void print_msg(u8 *data, unsigned int len) {
	if (debug)
		print_hex_dump(KERN_DEBUG, "message: ", DUMP_PREFIX_ADDRESS, 16, 1, data, len, 1);
	return;
}

static inline int print_binary(char *buf, unsigned long x, int nbits) {
	unsigned long mask = 1UL << (nbits - 1);
	while (mask != 0) {
		*buf++ = (mask & x ? '1' : '0');
		mask >>= 1;
	}
	*buf = '\0';
	return nbits;
}
#endif // _MCP2210_H

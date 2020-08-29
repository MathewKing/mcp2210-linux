/*
 *  MCP 2210 driver for linux
 *
 *  Copyright (c) 2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
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

#define USB_VENDOR_ID_MICROCHIP		0x04d8
#define USB_DEVICE_ID_MCP2210		0x00de

#define MCP2210_BUFFER_SIZE		64
#define MCP2210_MAX_SPEED		(12 * 1000 * 1000)

struct mcp2210_device {
	struct hid_device *hid;
	wait_queue_head_t wait;
	struct mcp2210_command *current_command;
	//struct mcp2210_command_request *current_request;
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


static inline void print_msg(u8 *data) {
	return;
	int x;
	for(x = 0; x < 64; x++) {
		printk("%02x ", data[x]);
		if((x + 1) % 16 == 0) {
			printk("\n");
		}
		else if((x + 1) % 8 == 0) {
			printk("  ");
		}
	}

	printk("\n");
}

#endif // _MCP2210_H

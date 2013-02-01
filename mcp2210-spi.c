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
 
#include <linux/spi/spi.h>

#include "mcp2210.h"

struct mcp2210_spi {
	struct mcp2210_device *dev;
	struct spi_master	*master;
};

struct mcp2210_spi_message {
	struct spi_device *spi;
	struct spi_message *msg;
	struct spi_transfer *current_transfer;
	unsigned int tx_pos;
	unsigned int rx_pos;
	unsigned int settings_set;
	unsigned int tx_in_process;
	unsigned int tx_bytes_in_process;
	unsigned int kill;
	struct list_head *next;
};

static void fill_u32(u8 *ptr, u32 data) {
	ptr[0] = data & 0xFF;
	ptr[1] = (data >> 8) & 0xFF;
	ptr[2] = (data >> 16) & 0xFF;
	ptr[3] = (data >> 24) & 0xFF;
}


static void fill_u16(u8 *ptr, u16 data) {
	ptr[0] = data & 0xFF;
	ptr[1] = (data >> 8) & 0xFF;
}

static void mcp2210_spi_populate_transfer_settings(u8 *request, struct mcp2210_spi_message *mcp_msg) 
{
	u32 bit_rate = mcp_msg->spi->max_speed_hz;
	u16 idle_cs = ~(0x01 << mcp_msg->spi->chip_select);
	u16 active_cs = 0xff;
	u16 cs_to_data_delay = 5; // 5 * 100us delay
	u16 last_byte_to_cs = 5;
	u16 delay_between_bytes = 0;
	u16 tansaction_bytes = mcp_msg->current_transfer->len;
	
	if(mcp_msg->current_transfer->speed_hz)
		bit_rate = mcp_msg->current_transfer->speed_hz;
		
	if(bit_rate > MCP2210_MAX_SPEED)
		bit_rate = MCP2210_MAX_SPEED;
	
	// Set SPI Transfer Settings
	request[0] = 0x40;
	
	fill_u32(request + 4, bit_rate);
	fill_u16(request + 8, idle_cs);
	fill_u16(request + 10, active_cs);
	fill_u16(request + 12, cs_to_data_delay);
	fill_u16(request + 14, last_byte_to_cs);
	fill_u16(request + 16, delay_between_bytes);
	fill_u16(request + 18, tansaction_bytes);
	
	request[20] = mcp_msg->spi->mode & 0x03;
}

static int next_mcp2210_spi_request(void *data, u8 *request) 
{
	struct mcp2210_spi_message *mcp_msg = data;
	u32 len;
	
	// Only send one command at a time
	if((mcp_msg->current_transfer && !mcp_msg->settings_set) || mcp_msg->tx_in_process) {
		return 0;
	}
	
	// Check for done	
	if(mcp_msg->next == &mcp_msg->msg->transfers || mcp_msg->msg->status) {
		//printk("mcp2210_spi Message done %d\n", mcp_msg->msg->status);
		mcp_msg->msg->complete(mcp_msg->msg->context);
		kfree(mcp_msg);
		return 0;
	}
	
	// Check for start of transfer
	if(!mcp_msg->current_transfer) {		
		//printk("mcp2210_spi Start new transfer \n");
		mcp_msg->current_transfer = list_entry(mcp_msg->next, struct spi_transfer, transfer_list);
		mcp_msg->tx_pos = 0;
		mcp_msg->rx_pos = 0;
		mcp_msg->tx_in_process = 0;
		mcp_msg->tx_bytes_in_process = 0;
		mcp_msg->settings_set = 0;
		mcp2210_spi_populate_transfer_settings(request, mcp_msg);
		print_msg(request);
		return 1;
	}
	
	// Write spi transfer command
	if(mcp_msg->tx_pos < mcp_msg->current_transfer->len) {
		len = mcp_msg->current_transfer->len - mcp_msg->tx_pos;
		
		if(len > MCP2210_BUFFER_SIZE - 4)
			len = MCP2210_BUFFER_SIZE - 4;
		
		request[0] = 0x42;
		request[1] = len & 0xff;
		
		if(mcp_msg->current_transfer->tx_buf)
			memcpy(request + 4, mcp_msg->current_transfer->tx_buf + mcp_msg->tx_pos, len);
			
		mcp_msg->tx_pos += len;
		mcp_msg->tx_bytes_in_process = len;
		mcp_msg->tx_in_process++;
		
		if(len == 0) {
			printk("ERROR: we are sending 0 bytes %d \n", mcp_msg->current_transfer->len);
		}
		
		//printk("mcp2210_spi send %d bytes \n", len);
		
		print_msg(request);
		return 1;
	}
	
	if(mcp_msg->rx_pos < mcp_msg->current_transfer->len) {
		// All data is sent but we need to wait and receive the rest		
		request[0] = 0x42;
		request[1] = 0;
		
		mcp_msg->tx_in_process++;
		mcp_msg->kill++;
		
		if(mcp_msg->kill > 50) 
		{
			mcp_msg->msg->status = -EIO;
			request[0] = 0x11;
			//printk("mcp2210_spi kill the transfer \n");
		}
		
		//printk("mcp2210_spi request rx bytes \n");
		
		print_msg(request);
		return 1;
	}
	
	if(mcp_msg->tx_pos == mcp_msg->current_transfer->len) {
		//printk("mcp2210_spi All tx bytes sent\n");
	}
	
	return 0;
}

static void mcp2210_spi_response(void *data, u8 *response) 
{
	struct mcp2210_spi_message *mcp_msg = data;
	int x;
	u8 len;
	
	//printk("Received data\n\n");
	
	print_msg(response);
	
	// This is the settings response
	if(!mcp_msg->settings_set) {
		//printk("mcp2210_spi Sent spi settings\n");
		mcp_msg->settings_set = 1;
		return;
	}
	
	// This is a transfer response
	mcp_msg->tx_in_process--;
		
	// The message has an error just return
	if(mcp_msg->msg->status)
		return;
	
	if(response[1] == 0xf8) {
		// data not accepted we need to resend
		mcp_msg->tx_pos -= mcp_msg->tx_bytes_in_process;
		mcp_msg->tx_bytes_in_process = 0;
		return;
	}
	
	if(response[1]) {
		// Error response
		mcp_msg->msg->status = -response[1];
		return;
	}	
	
	len = response[2];
	
	if(len) {
		// There is data to receive
		if(mcp_msg->current_transfer->rx_buf) 
			memcpy(mcp_msg->current_transfer->rx_buf + mcp_msg->rx_pos, response + 4, len);
			
		mcp_msg->rx_pos += len;
		mcp_msg->msg->actual_length += len;
	}
	
	// Data was accepted so clear in process
	mcp_msg->tx_bytes_in_process = 0;
	
	if(mcp_msg->rx_pos == mcp_msg->current_transfer->len) {
		// Transfer is done move next
		//printk("mcp2210_spi All rx bytes read\n");
		mcp_msg->current_transfer = NULL;
		mcp_msg->next = mcp_msg->next->next;
	}
}

static void mcp2210_spi_interrupted(void *data) 
{
	struct mcp2210_spi_message *mcp_msg = data;

	mcp_msg->msg->status = -ESHUTDOWN;
	mcp_msg->msg->complete(mcp_msg->msg->context);
	kfree(mcp_msg);
}

static int mcp2210_spi_setup(struct spi_device *spi)
{
	struct mcp2210_spi *ms;
	ms = spi_master_get_devdata(spi->master);
	
	return 0;
}

static int mcp2210_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct mcp2210_spi *ms;
	struct mcp2210_spi_message *mcp_msg;
	ms = spi_master_get_devdata(spi->master);
	
	mcp_msg = kzalloc(sizeof(struct mcp2210_spi_message), GFP_ATOMIC);
	if(!mcp_msg)
		return -ENOMEM;
	
	mcp_msg->spi = spi;
	mcp_msg->msg = msg;
	mcp_msg->next = msg->transfers.next;
	msg->status = 0;
	
	mcp2210_add_command(ms->dev, mcp_msg, next_mcp2210_spi_request, mcp2210_spi_response, mcp2210_spi_interrupted);
	
	return 0;
}

static void mcp2210_spi_cleanup(struct spi_device *spi)
{
	struct mcp2210_spi *ms;
	ms = spi_master_get_devdata(spi->master);
}

static struct spi_board_info demo_spi_devices[] = {
	{
		.modalias = "spidev",
		.chip_select = 0,
		.max_speed_hz = MCP2210_MAX_SPEED,
		.bus_num = 0,
		.mode = SPI_MODE_3,
	}
};

int mcp2210_spi_probe(struct mcp2210_device *dev) {
	int ret;
	struct spi_master	*master;
	struct mcp2210_spi *ms;
	
	ret = -ENOMEM;
	master = spi_alloc_master(&dev->hid->dev, sizeof *ms);	
	if (!master)
		goto out_free;
	
	master->bus_num = -1;
	master->num_chipselect = 4;
	master->setup = mcp2210_spi_setup;
	master->transfer = mcp2210_spi_transfer;
	master->cleanup = mcp2210_spi_cleanup;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
		
	ms = spi_master_get_devdata(master);
	ms->dev = dev;	
	ms->master = master;
	dev->spi_data = ms;
	
	ret = spi_register_master(master);
	
	demo_spi_devices[0].bus_num = master->bus_num;
	printk("mcp2210 spi master registered bus number %d\n", demo_spi_devices[0].bus_num);
	
	spi_new_device(master, demo_spi_devices);
	
	if (ret)
		goto out_free;
	
	return 0;
	
out_free:
	spi_master_put(master);
	return ret;
}

void mcp2210_spi_remove(struct mcp2210_device *dev)
{
	struct mcp2210_spi *ms = dev->spi_data;
	struct spi_master	*master = ms->master;	
	
	spi_unregister_master(master);
}
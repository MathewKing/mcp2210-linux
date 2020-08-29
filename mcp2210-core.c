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

#include <linux/usb.h>
#include <linux/module.h>

#include "mcp2210.h"

struct info_command {
	int id;
	int count;
	int response_count;
	int done;
	struct mcp2210_dev_cfg *cfg;
};

int mcp2210_spi_probe(struct mcp2210_device *dev);
void mcp2210_spi_remove(struct mcp2210_device *dev);
static void mcp2210_output_command_atomic(struct mcp2210_device *dev);

int next_mcp2210_info_command(void *data, u8 *request) {
	struct info_command *cmd_data = data;
        // last info command
	if(cmd_data->count >= 1) {
		return 0;
	}

	cmd_data->count++;
	request[0] = MCP2210_CMD_GET_SPI_CONFIG;

	return 1;
}

void next_mcp2210_info_command_response(void *data, u8 *response) {
	struct info_command *cmd_data = data;
	char idle_cs[10], active_cs[10];

	cmd_data->response_count++;
        // last info command response
	if(cmd_data->response_count >= 1) {
		cmd_data->done = 1;

		verbose_debug(KERN_DEBUG "next_mcp2210_info_command done %d: %d\n", cmd_data->id, cmd_data->response_count);
		print_msg(response, MCP2210_BUFFER_SIZE);

		// parse configuration
		cmd_data->cfg->bitrate = *(u32*)&response[4];
		cmd_data->cfg->idle_cs = *(u16*)&response[8];
		cmd_data->cfg->active_cs = *(u16*)&response[10];
		cmd_data->cfg->cs_ds_delay = *(u16*)&response[12];
		cmd_data->cfg->cs_ls_delay = *(u16*)&response[14];
		cmd_data->cfg->byte_delay = *(u16*)&response[16];
		cmd_data->cfg->byte_per_spi = *(u16*)&response[18];
		cmd_data->cfg->spi_mode = response[20];

                // activate cfg
		cmd_data->cfg->active = 1;

		print_binary(idle_cs, *(u16*)&response[8], 9);
		print_binary(active_cs, *(u16*)&response[10], 9);
		printk(KERN_INFO "MCP2210 SPI settings: bitrate=%i(bit/s) idle_cs=%s active_cs=%s cs_ds_delay=%i(us) cs_ls_delay=%i(us) byte_delay=%i(us) byte_per_spi=%i spi_mode=0x%02x\n",
				cmd_data->cfg->bitrate, idle_cs, active_cs,
				cmd_data->cfg->cs_ds_delay * 100,
				cmd_data->cfg->cs_ls_delay * 100,
				cmd_data->cfg->byte_delay * 100,
				cmd_data->cfg->byte_per_spi,
				cmd_data->cfg->spi_mode);
	}

	if(response[0] != MCP2210_CMD_GET_SPI_CONFIG) {
		printk(KERN_ERR "wrong command response: 0x%02x\n", response[0]);
	}
}

void mcp2210_info_command_interrupted(void *data) {

	printk(KERN_INFO "mcp2210_info_command_interrupted\n");
}

static void mcp2210_process_commands(struct mcp2210_device *dev) {
	verbose_debug(KERN_DEBUG "mcp2210_process_commands\n");

	// Get the next request from the current command
	if(dev->current_command) {
		mcp2210_output_command_atomic(dev);
	}

	if(!dev->current_command && !list_empty(&dev->command_list)) {
		dev->current_command = list_first_entry(&dev->command_list, struct mcp2210_command, node);
		mcp2210_process_commands(dev);
	}
}

static void mcp2210_output_command(struct work_struct *work) {
	//struct mcp2210_device *dev = container_of(work, struct mcp2210_device, command_work);
}

static void mcp2210_output_command_atomic(struct mcp2210_device *dev) {
	struct hid_report *report;
	struct hid_field *field;
	struct mcp2210_request_list *list_node;
	int cnt, pending;

	int num;
	verbose_debug(KERN_DEBUG "mcp2210_output_command\n");

	if(!dev->current_command) {
		printk(KERN_ERR "Error: no current command\n");
		return;
	}

	memset(dev->requeust_buffer, 0, MCP2210_BUFFER_SIZE);
	num = dev->current_command->next_request(dev->current_command->data, dev->requeust_buffer);
	while(num > 0) {
		dev->current_command->requests_pending++;

		report = kzalloc(sizeof(struct hid_report), GFP_ATOMIC);
		if(!report)
			goto err_free_report;

		field = kzalloc(sizeof(struct hid_field), GFP_ATOMIC);
		if(!field)
			goto err_free_field;

		field->value = kzalloc((MCP2210_BUFFER_SIZE - 1) * sizeof(__s32), GFP_ATOMIC);
		if(!field->value)
			goto err_free_value;

		list_node = kzalloc(sizeof(struct mcp2210_request_list), GFP_ATOMIC);
		if(!list_node)
			goto err_free_node;

		report->id = dev->requeust_buffer[0];
		report->type = HID_OUTPUT_REPORT;
		report->field[0] = field;
		report->maxfield = 1;
		report->size = (MCP2210_BUFFER_SIZE - 1) << 3;
		report->device = dev->hid;

		field->logical_minimum = 0;
		field->logical_maximum = 255;
		field->report_count = MCP2210_BUFFER_SIZE - 1;
		field->report_size = 8;

		list_node->report = report;
		list_add_tail(&list_node->node, &dev->current_command->request_list);

		for(cnt = 0; cnt < (MCP2210_BUFFER_SIZE - 1); cnt++) {
			field->value[cnt] = dev->requeust_buffer[cnt + 1];
		}

		hid_hw_request(dev->hid, report, HID_REQ_SET_REPORT);

		if(dev->current_command->requests_pending >= 63)
			break;

		memset(dev->requeust_buffer, 0, MCP2210_BUFFER_SIZE);
		num = dev->current_command->next_request(dev->current_command->data, dev->requeust_buffer);
	}

	verbose_debug(KERN_DEBUG "Sending %d commands\n", dev->current_command->requests_pending);
	if(dev->current_command->requests_pending == 0) {
		// The command has finished so remove it
		list_del(&dev->current_command->node);

		kfree(dev->current_command);
		dev->current_command = NULL;
		mcp2210_process_commands(dev);
	}

	return;

err_free_node:
	kfree(list_node);
err_free_value:
	kfree(field->value);
err_free_field:
	kfree(field);
err_free_report:
	kfree(report);

	pending = dev->current_command->requests_pending;
	if(pending == 0) {
		if(dev->current_command->interrupted)
			dev->current_command->interrupted(dev->current_command->data);
		list_del(&dev->current_command->node);

		kfree(dev->current_command);
		dev->current_command = NULL;
		mcp2210_process_commands(dev);
	}
}

int mcp2210_add_command(struct mcp2210_device *dev, void *cmd_data,
		int (*next_request)(void *command_data, u8 *request),
		void (*data_received)(void *command_data, u8 *response),
		void (*interrupted)(void *command_data)) {

	struct mcp2210_command *cmd;

	spin_lock(&dev->command_lock);
	verbose_debug(KERN_DEBUG "mcp2210_add_command lock\n");

	cmd = kzalloc(sizeof(struct mcp2210_command), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->data = cmd_data;
	cmd->dev = dev;
	cmd->next_request = next_request;
	cmd->data_received = data_received;
	cmd->interrupted = interrupted;

	INIT_LIST_HEAD(&cmd->request_list);

	list_add_tail(&cmd->node, &dev->command_list);


	if(!dev->current_command) {
		mcp2210_process_commands(dev);
	}

	verbose_debug(KERN_DEBUG "mcp2210_add_command unlock\n");
	spin_unlock(&dev->command_lock);

	return 0;
}

static void mcp2210_free_request(struct mcp2210_request_list *list_node) {
	kfree(list_node->report->field[0]->value);
	kfree(list_node->report->field[0]);
	kfree(list_node->report);
	list_del(&list_node->node);
	kfree(list_node);
}

// Called when data is received from the mcp2210
static int mcp2210_raw_event(struct hid_device *hdev, struct hid_report *report,
			u8 *data, int size)
{
	struct mcp2210_device *dev = hid_get_drvdata(hdev);
	struct mcp2210_request_list *list_node;

	if(size == MCP2210_BUFFER_SIZE && dev->current_command) {
		spin_lock(&dev->command_lock);
		verbose_debug(KERN_DEBUG "mcp2210_raw_event lock\n");
		list_node = list_first_entry(&dev->current_command->request_list, struct mcp2210_request_list, node);
		mcp2210_free_request(list_node);

		dev->current_command->data_received(dev->current_command->data, data);
		dev->current_command->requests_pending--;

		mcp2210_process_commands(dev);
		verbose_debug(KERN_DEBUG "mcp2210_raw_event unlock\n");
		spin_unlock(&dev->command_lock);
		return 1;
	}

	return 0;
}

// Called when a new mcp2210 device is added
static int mcp2210_probe(struct hid_device *hdev,
		const struct hid_device_id *id)
{
	int ret;
	struct mcp2210_device *dev;
	struct mcp2210_dev_cfg *cfg;
	struct info_command *cmd_data;

	dev = kzalloc(sizeof(struct mcp2210_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	cfg = kzalloc(sizeof(struct mcp2210_dev_cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;
	cfg->active = 0;

	dev->cfg = cfg;

	verbose_debug(KERN_DEBUG "mcp2210_probe\n");

	hid_set_drvdata(hdev, dev);
	dev->hid = hdev;
	INIT_LIST_HEAD(&dev->command_list);
	mutex_init(&dev->command_mutex);
	spin_lock_init(&dev->command_lock);
	INIT_WORK(&dev->command_work, mcp2210_output_command);

	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}

	if (hdev->ll_driver->power) {
		ret = hdev->ll_driver->power(hdev, PM_HINT_FULLON);
		if (ret < 0)
			goto err_free;
	}
	ret = hdev->ll_driver->open(hdev);
	if (ret < 0) {
		if (hdev->ll_driver->power)
			hdev->ll_driver->power(hdev, PM_HINT_NORMAL);

		goto err_free;
	}

	ret = mcp2210_spi_probe(dev);
	if(ret < 0) {
		goto err_power;
	}

	cmd_data = kzalloc(sizeof(struct info_command), GFP_KERNEL);
	if (!cmd_data)
		return -ENOMEM;


	cmd_data->id = 1;
	cmd_data->cfg = dev->cfg;
	// Query current SPI settings from IC
	mcp2210_add_command(dev, cmd_data, next_mcp2210_info_command, next_mcp2210_info_command_response, mcp2210_info_command_interrupted);

	return 0;

err_power:
	if (hdev->ll_driver->power)
		hdev->ll_driver->power(hdev, PM_HINT_NORMAL);
	hdev->ll_driver->close(hdev);
err_free:
	kfree(dev);

	return ret;
}

// Called when a new mcp2210 device is removed
static void mcp2210_remove(struct hid_device *hdev)
{
	struct mcp2210_device *dev = hid_get_drvdata(hdev);
	struct list_head *pos, *q, *request_pos, *request_q;
	struct mcp2210_command *tmp;

	list_for_each_safe(pos, q, &dev->command_list){
		tmp = list_entry(pos, struct mcp2210_command, node);
		list_del(pos);

		if(tmp->interrupted)
			tmp->interrupted(tmp->data);

		list_for_each_safe(request_pos, request_q, &tmp->request_list) {
			mcp2210_free_request(list_entry(request_pos, struct mcp2210_request_list, node));
		}

		kfree(tmp);
	}

	mcp2210_spi_remove(dev);

	if (hdev->ll_driver->power)
		hdev->ll_driver->power(hdev, PM_HINT_NORMAL);
	hdev->ll_driver->close(hdev);

	kfree(dev->cfg);
	kfree(dev);

	verbose_debug(KERN_DEBUG "mcp2210_remove\n");
	hid_hw_stop(hdev);
}

static const struct hid_device_id mcp2210_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MCP2210),
		.driver_data = 0 },
	{ }
};

static struct hid_driver mcp2210_driver = {
	.name = "mcp2210",
	.id_table = mcp2210_devices,
	.raw_event = mcp2210_raw_event,
	.probe = mcp2210_probe,
	.remove = mcp2210_remove
};

static int __init mcp2210_init(void)
{
	int ret;
	verbose_debug(KERN_DEBUG "mcp2210_init\n");

	ret = hid_register_driver(&mcp2210_driver);
	if (ret)
		printk(KERN_ERR "can't register mcp2210 driver\n");

	return ret;
}

static void __exit mcp2210_exit(void)
{
	verbose_debug(KERN_DEBUG "mcp2210_exit\n");
	hid_unregister_driver(&mcp2210_driver);
}

int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug messages");

module_init(mcp2210_init);
module_exit(mcp2210_exit);
MODULE_LICENSE("GPL");

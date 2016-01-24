/*
 * MCP2210 userspace library
 * Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mcp2210-user.h"

int mcp2210_do_ioctl(const char *devname, enum mcp2210_ioctl_cmd cmd, struct mcp2210_ioctl_data *data) {
//int mcp2210_do_ioctl(const char *devname, enum mcp2210_ioctl_cmd cmd, void *arg) {
	struct mcp2210_device dev;// = INIT_MCP2210_DEVICE(devname);
	int ret = 0;

	if (mcp2210_open(&dev, devname) == -1) {
		perror("open failed");
		return -1;
	}

	ret = mcp2210_ioctl(&dev, cmd, data);
	if (ret < 0) {
		perror("ioctl failed");
		return -1;
	}

	if (mcp2210_close(&dev) == -1) {
		perror("ioctl failed");
		return -1;
	}

	return 0;
}
#if 0
int mcp2210_get_config(const char *devname, struct mcp2210_ioctl_data *dest) {
	struct mcp2210_ioctl_data *data;
//	struct mcp2210_device dev;
	const size_t struct_size = IOCTL_DATA_CONFIG_SIZE + 0x200;
	int ret = 0;

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;

	fprintf(stderr, "offset = %lu\n", (unsigned long)offsetof(struct mcp2210_ioctl_data, body.config));

	ret = mcp2210_do_ioctl(devname, MCP2210_IOCTL_CONFIG_GET, data);
	if (ret < 0)
		perror("mcp2210_do_ioctl");

	/*
	if ((ret = mcp2210_open(&dev, devname)))
		goto exit_free;

	ret = mcp2210_ioctl(&dev, MCP2210_IOCTL_CONFIG_GET, (unsigned long)((void*)data));
	mcp2210_close(&dev);
	*/

	free(data);
	return ret;
}

int mcp2210_set_config(const char *devname, const struct mcp2210_ioctl_data *src) {
	struct mcp2210_ioctl_data *data;
//	struct mcp2210_device dev;
	int ret = 0;

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;

	fprintf(stderr, "offset = %lu\n", (unsigned long)offsetof(struct mcp2210_ioctl_data, body.config));

	ret = mcp2210_do_ioctl(devname, MCP2210_IOCTL_CONFIG_GET, data);
	if (ret < 0)
		perror("mcp2210_do_ioctl");

	free(data);
	return ret;
}
#endif

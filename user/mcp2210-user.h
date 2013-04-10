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

#ifndef _MCP2210_USER_H
#define _MCP2210_USER_H

#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <error.h>

#include "mcp2210.h"
#include "mcp2210-creek.h"
#include "mcp2210-debug.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct mcp2210_device {
	int fd;
	const char *name;
};

#define INIT_MCP2210_DEVICE(_name) {.fd = -1, .name = _name}

static inline int mcp2210_open(struct mcp2210_device *dev, const char *name) {
	int fd;

	memset(dev, 0, sizeof(*dev));
	if ((fd = open(name, O_RDWR)) == -1)
		return -1;

	dev->fd = fd;
	dev->name = name;

	return 0;
}

static inline int mcp2210_ioctl(struct mcp2210_device *dev, uint cmd,
				struct mcp2210_ioctl_data *data) {
	int ret;

	if (cmd >= MCP2210_IOCTL_MAX) {
		errno = EINVAL;
		return -1;
	}

	ret = ioctl(dev->fd, mcp2210_ioctl_map[cmd], (unsigned long)data);
	if (ret) {
		errno = -ret;
		return -1;
	}

	return 0;
}

static inline int mcp2210_close(struct mcp2210_device *dev) {
	return close(dev->fd);
}

/**
 */
int mcp2210_do_ioctl(const char *devname, enum mcp2210_ioctl_cmd cmd, struct mcp2210_ioctl_data *data);


int test_encoding(int argc, char *argv[]);
int test_bit_stream(int argc, char *argv[]);
#define _do_error(fatal, err, file, line, fmt, ...) error_at_line(fatal, err, file, line, "%s - " fmt, __PRETTY_FUNCTION__, ## __VA_ARGS__)
#define fatal_error(fmt, ...)    _do_error(1, errno, __FILE__, __LINE__, fmt, ## __VA_ARGS__)
#define nonfatal_error(fmt, ...) _do_error(0, errno, __FILE__, __LINE__, fmt, ## __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* _MCP2210_USER_H */

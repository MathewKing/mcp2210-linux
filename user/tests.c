/*
 * MCP2210 userspace test functions
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
#include "settings.h"

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef CONFIG_MCP2210_CREEK
static int feature_disabled(void) {
	fprintf(stderr, "Feature disabled (set CONFIG_MCP2210_CREEK to enable)\n");
	return -EPERM;
}

int test_encoding(int argc, char *argv[]) {
	return feature_disabled();
}

int test_bit_stream(int argc, char *argv[]) {
	return feature_disabled();
}

#else /* CONFIG_MCP2210_CREEK */

static int compare_board_config(const struct mcp2210_board_config *a,
				const struct mcp2210_board_config *b) {
	uint i;

	if (a->poll_gpio_usecs	!= b->poll_gpio_usecs)	return 1;
	if (a->poll_intr_usecs	!= b->poll_intr_usecs)	return 2;
	if (a->stale_gpio_usecs	!= b->stale_gpio_usecs)	return 3;
	if (a->stale_intr_usecs	!= b->stale_intr_usecs)	return 4;

	for (i = 0; i < MCP2210_NUM_PINS; ++i) {
		const struct mcp2210_pin_config *pa = &a->pins[i];
		const struct mcp2210_pin_config *pb = &b->pins[i];
		uint desc = (i + 1) << 4;

		if (pa->mode != pb->mode)
			return desc;

		if (!!pa->name != !!pb->name)
			return desc | 1;

		if (pa->name && strcmp(pa->name, pb->name))
			return desc | 2;

		if (!!pa->modalias != !!pb->modalias)
			return desc | 3;

		if (pa->modalias && strcmp(pa->modalias, pb->modalias))
			return desc | 4;

		switch (pa->mode) {
		case MCP2210_PIN_DEDICATED:
		case MCP2210_PIN_GPIO:
			if (pa->has_irq	 != pb->has_irq	) return desc | 5;
			if (pa->irq	 != pb->irq	) return desc | 6;
			if (pa->irq_type != pb->irq_type) return desc | 7;
			break;

		case MCP2210_PIN_SPI:
			if (memcmp(&pa->spi, &pb->spi, sizeof(pa->spi)))
				return desc | 8;
			break;

		default:
			break;
		}
	}


	return 0;
}

static void print_failed_item(int val) {
	const char *item;
	uint section = val >> 4;

	if (val < 16) {
		switch (val) {
		case 1: item = "poll_gpio_usecs"; break;
		case 2: item = "poll_intr_usecs"; break;
		case 3: item = "stale_gpio_usecs"; break;
		case 4: item = "stale_intr_usecs"; break;
		default: item = "bad code"; break;
		}
		fprintf(stderr, "%s", item);
	} else {
		uint pin = section - 1;

		switch (val & 0xf) {
		case 0: item = "mode"; break;
		case 1: item = "name presence"; break;
		case 2: item = "name"; break;
		case 3: item = "modalias presence"; break;
		case 4: item = "modalias"; break;
		case 5: item = "has_irq"; break;
		case 6: item = "irq"; break;
		case 7: item = "irq_type"; break;
		case 8: item = "spi memcmp"; break;
		default: item = "bad code"; break;
		}
		fprintf(stderr, "pin %u, %s", pin, item);
	}
}

int test_encoding(int argc, char *argv[]) {
	struct mcp2210_board_config *board_config;
	struct mcp2210_board_config *new_cfg;
	u8 buf[0x100];
	int ret;

	memset(buf, 0, sizeof(buf));
	board_config = copy_board_config(NULL, &my_board_config, 0);
	if (!board_config) {
		errno = ENOMEM;
		perror("");
		return -ENOMEM;
	}

	ret = compare_board_config(board_config, &my_board_config);
	if (ret) {
		fprintf(stderr, "copy_board_config() produced a non-matching "
				"copy: ret = %d\n", ret);
		goto exit_free;
	}

	ret = creek_encode(board_config, &my_chip_settings, buf, sizeof(buf));

	if (ret < 0) {
		errno = -ret;
		perror("creek_encode");
		goto exit_free;
	}
	fprintf(stderr, "encoded into %d bits (%d bytes)\n", ret, (ret + 7) / 8);

	new_cfg = creek_decode(&my_chip_settings, buf, sizeof(buf), GFP_KERNEL);

	if (IS_ERR_VALUE(new_cfg)) {
		ret = PTR_ERR(new_cfg);
		errno = -ret;
		perror("creek_decode");
		goto exit_free;
	}

	dump_board_config(KERN_INFO, 0, "original = ", board_config);
	dump_board_config(KERN_INFO, 0, "extra_crispy = ", new_cfg);

	ret = compare_board_config(board_config, new_cfg);
	if (ret) {
		fprintf(stderr, "They don't match: ");
		print_failed_item(ret);
		fputs("\n", stderr);
	}else
		fprintf(stderr, "Success\n");

	free(new_cfg);
exit_free:
	errno = 0;
	free(board_config);
	return ret;
}

int test_bit_stream(int argc, char *argv[])
{
	int ret;
	int fd;
	unsigned i;
	u8 sizes[0x4000];
	u8 src[0x10000];
	u8 dest[0x10000];

	struct bit_creek bs1 = {
		.start = src,
		.size = sizeof(src),
	};

	struct bit_creek bs2 = {
		.start = dest,
		.size = sizeof(dest),
	};

	/* Get some random data */
	if ((fd = open("/dev/urandom", O_RDONLY)) < 0) {
		perror("can't open device");
		fatal_error("failed to open %s", "/dev/urandom");

	}

	/* random size of read/writes */
	if ((ret = read(fd, sizes, sizeof(sizes))) != sizeof(sizes)) {
		ret = -errno;
		perror("failed to read random size data");
		return ret;
	}

	/* get random data to read/write */

	if ((ret = read(fd, src, sizeof(src))) != sizeof(src)) {
		ret = -errno;
		perror("failed to read random data");
		return ret;
	}

	close(fd);

	for (i = 0; i < sizeof(sizes); ++i) {
		u8 bits = (sizes[i] & 31) + 1;
		unsigned u;

		u = creek_get_bits(&bs1, bits);
		if (bs1.overflow) {
			fprintf(stderr, "overflow in 1st bitstream, doesn't indicate a failure\n");
			bs1.pos -= bits;
			break;
		}

		//fprintf(stderr, "%lu (%lu), %hhu, 0x%08x\n", bs1.pos, bs1.pos / 8, bits, u);
		creek_put_bits(&bs2, u, bits);
		if (bs2.overflow) {
			fprintf(stderr, "overflow");
			return 1;
		}
	}

	if (bs1.pos != bs2.pos) {
		fprintf(stderr, "Fail, bs1.pos != bs2.pos (%u != %u)\n", (unsigned)bs1.pos, (unsigned)bs2.pos);
		return 1;
	}

	if (memcmp(src, dest, bs1.pos / 8 - 1)) {
		fprintf(stderr, "Fail, didn't match\n");
		return 1;
	}

	fprintf(stderr, "Success\n");
	return 0;
}
#endif /* CONFIG_MCP2210_CREEK */

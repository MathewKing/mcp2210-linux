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

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <unistd.h>

static const struct mcp2210_chip_settings chip_settings =  {
	.pin_mode = {
		MCP2210_PIN_SPI,
		MCP2210_PIN_GPIO,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
		MCP2210_PIN_UNUSED,
	},
	.gpio_value		= 2,
	.gpio_direction		= 2,
	.other_settings		= 0,
	.nvram_access_control	= 0,
	.password = {0, 0, 0, 0, 0, 0, 0, 0},
};

static const struct mcp2210_board_config static_board_config = {
	.pins = {
		{
			.mode = MCP2210_PIN_SPI,
			.body.spi.max_speed_hz = 20000,
			.body.spi.min_speed_hz = 2000,
			.body.spi.mode = SPI_MODE_3,
			.body.spi.bits_per_word = 8,
/* CS must be de-asserted between each byte in a transaction.  You send zeros
 * when you are receiving the response.
 *
 * fCK,MAX     | Maximum SPI clock frequency  | 5  MHz max
 * tsetCS      | Chip select setup time       | 350 ns min
 * trCK & tfCK | SPI clock rise and fall time | 25  ns max (@CL = 30 pF)
 * thCK & tlCK | SPI clock high and low time  | 75  ns min
 * tholCS      | Chip select hold time        | 10  ns min
 * tdisCS      | Deselect time                | 800 ns min
 * tsetSDI     | Data input setup time        | 25  ns min
 * tholSDI     | Data input hold time         | 20  ns min
 * tenSDO      | Data output enable time      | 38  ns max
 * tdisSDO     | Data output disable time     | 47  ns max
 * tvSDO       | Data output valid time       | 57  ns max
 */
			.body.spi.cs_to_data_delay = 1,
			.body.spi.last_byte_to_cs_delay = 1,
			.body.spi.delay_between_bytes = 1,
			.body.spi.delay_between_xfers = 1,
			.modalias = "spidev",
			.name = "L6470",

			/*.desc = "L6470 dSPIN: Fully integrated microstepping "
				"motor driver with motion engine and SPI" */
		}, {
			.mode = MCP2210_PIN_GPIO,
			.body.gpio.direction = 1,
			.body.gpio.init_value = 1,
			.name = "some pin",
		}, {
			.mode = MCP2210_PIN_UNUSED,
			.name="fuck me",
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}, {
			.mode = MCP2210_PIN_UNUSED,
		}
	},
	.strings_size = 0,
//	.strings = "",
};

static int compare_board_config(const struct mcp2210_board_config *a,
				const struct mcp2210_board_config *b) {

	uint i;

	for (i = 0; i < 9; ++i) {
		const struct mcp2210_pin_config *pa = &a->pins[i];
		const struct mcp2210_pin_config *pb = &b->pins[i];
		int ret = (int)i * 16;

		if (pa->mode != pb->mode)
			return ret + 1;

		if (!!pa->name != !!pb->name)
			return ret + 2;

		if (pa->name && strcmp(pa->name, pb->name))
			return ret + 3;

		if (!!pa->modalias != !!pb->modalias)
			return ret + 4;

		if (pa->modalias && strcmp(pa->modalias, pb->modalias))
			return ret + 5;

		switch (pa->mode) {
		case MCP2210_PIN_GPIO:
			if (memcmp(&pa->body.gpio, &pb->body.gpio, sizeof(pa->body.gpio)))
				return ret + 6;
			break;

		case MCP2210_PIN_SPI:
			if (memcmp(&pa->body.spi, &pb->body.gpio, sizeof(pa->body.spi)))
				return ret + 7;
			break;

		default:
			break;
		}
	}

	return 0;
}


int test_encoding(int argc, char *argv[]) {
	struct mcp2210_board_config *board_config;
	struct mcp2210_board_config *new_cfg;
	u8 buf[0x100];
	int ret;

	memset(buf, 0, sizeof(buf));
	board_config = copy_board_config(NULL, &static_board_config, 0);
	if (!board_config) {
		errno = ENOMEM;
		perror("");
		return -ENOMEM;
	}

	ret = compare_board_config(board_config, &static_board_config);
	if (ret) {
		fprintf(stderr, "copy_board_config() produced a non-matching copy: ret = %d\n", ret);
		goto exit_free;
	}

	ret = creek_encode(board_config, &chip_settings, buf, sizeof(buf));

	if (ret < 0) {
		errno = -ret;
		perror("creek_encode");
		goto exit_free;
	}
	fprintf(stderr, "encoded into %d bytes\n", ret);

	new_cfg = creek_decode(&chip_settings, buf, sizeof(buf), GFP_KERNEL);

	if (IS_ERR_VALUE(new_cfg)) {
		ret = PTR_ERR(new_cfg);
		errno = -ret;
		perror("creek_decode");
		goto exit_free;
	}

	dump_board_config(KERN_INFO, 0, "original = ", board_config);
	dump_board_config(KERN_INFO, 0, "extra_crispy = ", new_cfg);

	ret = compare_board_config(board_config, new_cfg);
	if (ret)
		fprintf(stderr, "They don't match: ret = %d\n", ret);
	else
		fprintf(stderr, "Success\n");

	free(new_cfg);
exit_free:
	errno = 0;
	free(board_config);
	return ret;
}

#if 0
void fuckme(const u8 *src, const u8 *dest)
{
	unsigned i;
	for (i = 0; i < 32; ++i) {
		printf("%02x ", src[i]);
	}
	puts("");
	for (i = 0; i < 32; ++i) {
		printf("%02x ", dest[i]);
	}
	puts("");

}
#endif

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

//fuckme(src, dest);
	}

	if (bs1.pos != bs2.pos) {
		fprintf(stderr, "Fail, bs1.pos != bs2.pos (%u != %u)\n", (unsigned)bs1.pos, (unsigned)bs2.pos);
		return 1;
	}
//	fprintf(stderr, "%hhu, 0x%08x\n", 32, creek_get_bits(&bs1, 32));
//	fuckme(src, dest);
	if (memcmp(src, dest, bs1.pos / 8 - 1)) {
		fprintf(stderr, "Fail, didn't match\n");
		return 1;
	}

	fprintf(stderr, "Success\n");
	return 0;
}

/*
 * MCP2210 userspace utility
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>

#include "mcp2210-user.h"

static const char *argv0;

static void show_usage(void);

static int eeprom_read(int argc, char *argv[]);
static int eeprom_write(int argc, char *argv[]);
static int eeprom_read_write(int is_read, int argc, char *argv[]);


static struct config {
	bool verbose;
	const char *name;
	const char *cmd;
	const char *target;
	const char *filein;
	const char *fileout;
	struct {
		/* u8  */ uint addr;
		/* u16 */ uint size;
	} eeprom;
	struct {
		const char *name;
		/* u8 */ uint mode;
		/* u8 */ uint  bits;
		/* u32 */ uint  speed;
		/* u32 */ uint  delay;
	} spi;
} config = {
	.verbose = 1,
	.name = "/dev/usb2spi_bridge0",
	.cmd = NULL,
	.target = NULL,
	.filein = NULL,
	.fileout = NULL,
	.eeprom = {
		.addr = 0,
		.size = 0x100,
	},
	.spi = {
		.name = "/dev/spidev1.1",
		.mode = 3,
		.bits = 8,
		.speed = 10000,
		.delay = 0,
	},
};
/**
 * Determine base by prefix and offset to number. Uses standard rules
 * (expressed in regex below):
 * 0[xX][0-9a-fA-F]+ denotes a hexidecimal number
 * 0[0-7]+           denotes an octal number
 * (0|[1-9][0-9]*)   denotes a decimal number
 */
int get_param_base_and_start(const char **str)
{
	assert(*str);

	/* Parse an "0x", "0X" or "0" -prefixed string, but not the string
	 * "0" (which will be treated as base ten). */
	if (**str == '0' && *(*str + 1)) {
		++*str;
		if (**str == 'x' || **str == 'X') {
			++*str;
			return 16;
		} else {
			return 8;
		}
	} else {
		return 10;
	}
}

/**
 * @returns zero on success, non-zero if the string didn't represent a clean
 *          number
 *
 * FIXME: wont be correct for u32 on platforms where sizeof(int) == 2
 */
static int get_uint_param(const char *str, uint *dest, uint min, uint max)
{
	const char *start = str;
	char *endptr;
	int base = get_param_base_and_start(&start);

	*dest = strtoul(start, &endptr, base);
	if (*endptr) {
		fprintf(stderr, "bad number: %s\n", str);
		exit (1);
	}

	if (*dest < min || *dest > max) {
		fprintf(stderr, "number out of range: %s should be between %u and %u\n", str, min, max);
		exit (1);
	}

	return *endptr;
}

static int get_bool_param(const char *str)
{
	uint dest;
	get_uint_param(str, &dest, 0, 1);
	return dest;
}

/**
 * @returns zero on success, non-zero if the string didn't represent a clean
 *          number
 * FIXME: assumes u64 is unsigned long long
 */
static inline int get_u64_param(const char *str, u64 *dest)
{
	char *endptr;
	int base = get_param_base_and_start(&str);

	*dest = strtoull(str, &endptr, base);
	if (*endptr) {
		fprintf(stderr, "bad number: %s\n", str);
		exit (1);
	}

	return *endptr;
}



/******************************************************************************
 *
 */

int get_input_data(void *dest, size_t size) {
	const char *file_name = config.filein;
	int use_stdin = !file_name || !strcmp(file_name, "-");
	int fd;
	ssize_t ret;

	fprintf(stderr, "get_input_data dest %p, size %lu\n", dest, (unsigned long) size);

	if (use_stdin)
		fd = fileno(stdin);
	else if ((fd = open(file_name, O_RDONLY)) < 0) {
		perror("open");
		return errno;
	}

	ret = read(fd, dest, size);
	if (ret < 0) {
		perror("read(): reading input");
		ret = errno;
	} else if ((size_t)ret != size) {
		fprintf(stderr, "Short input: expected %lu bytes, got %ld.\n",
			(unsigned long)size, (long)ret);
		ret = -EOVERFLOW;
	}

	if (!use_stdin)
		close(fd);

	return ret;
}

int put_output_data(const void *src, size_t size) {
	const char *file_name = config.fileout;
	int use_stdout = !file_name || !strcmp(file_name, "-");
	int fd;
	ssize_t ret;

	fprintf(stderr, "put_output_data src: %p, size: %lu\n", src, (unsigned long) size);

	if (use_stdout) {
		fd = fileno(stdout);
		if (isatty(fd)) {
			fprintf(stderr, "Cowardly refusing to write binary data to terminal\n");
			return -EPERM;
		}
	} else if ((fd = open(file_name, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR)) < 0) {
		perror("open");
		return errno;
	}

	ret = write(fd, src, size);
	if (ret < 0) {
		perror("write(): writing output");
		ret = errno;
	} else if ((size_t)ret != size) {
		fprintf(stderr, "put_output_data: Failed to write all data: have %lu bytes, only wrote %ld.\n",
			(unsigned long)size, (long)ret);
		ret = -EOVERFLOW;
	} else
		ret = 0;

	if (!use_stdout)
		close(fd);

	return ret;
}

static const struct mcp2210_chip_settings static_chip_settings =  {
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

int get_config(int argc, char *argv[]) {
	struct mcp2210_ioctl_data *data;
	struct mcp2210_ioctl_data_config *cfg;
	const size_t struct_size = IOCTL_DATA_CONFIG_SIZE + 0x100;
	int ret = 0;
	u8 buf[0x100];

	memset(buf, 0, sizeof(buf));

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;
	cfg = &data->body.config;

	fprintf(stderr, "offset = %u\n", (uint)offsetof(struct mcp2210_ioctl_data, body.config));

	ret = mcp2210_do_ioctl(config.name, MCP2210_IOCTL_CONFIG_GET, data);
	if (ret)
		goto exit_free;

	dump_state("", 0, "state = ", &cfg->state);
	if (cfg->state.have_config)
		dump_board_config("", 0, ".config = ", &cfg->config);

	ret = creek_encode(&cfg->config, &static_chip_settings, buf, sizeof(buf));
	if (ret < 0) {
		errno = -ret;
		fatal_error("creek_encode");
	}

	ret = put_output_data(buf, ret);

exit_free:
	free(data);
	return ret;
}


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

int set_config(int argc, char *argv[]) {
	struct mcp2210_ioctl_data *data;
	struct mcp2210_ioctl_data_config *cfg;
	const size_t struct_size = IOCTL_DATA_CONFIG_SIZE + 0x400;
	int ret = 0;

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;
	cfg = &data->body.config;
	cfg->config.strings_size = 0x400;
	copy_board_config(&cfg->config, &static_board_config, 0);
	ret = mcp2210_do_ioctl(config.name, MCP2210_IOCTL_CONFIG_SET, data);
	free(data);
	return ret;
}

static int eeprom_read(int argc, char *argv[]) {
	return eeprom_read_write(1, argc, argv);
}

static int eeprom_write(int argc, char *argv[]) {
	return eeprom_read_write(0, argc, argv);
}

static int eeprom_read_write(int is_read, int argc, char *argv[]) {
	struct mcp2210_ioctl_data *data;
	struct mcp2210_ioctl_data_eeprom *eeprom;
	const size_t struct_size = IOCTL_DATA_EEPROM_SIZE + 0x100;
	uint addr;
	uint size;
	int i;
	int ret = 0;

	for (i = 0; i < argc; ++i) {
		const char *arg = argv[i];
		if (!strncmp(arg, "addr=", 5)) {
			get_uint_param(&arg[5], &addr, 0, 0xff);
		} else if (!strncmp(arg, "size=", 5)) {
			get_uint_param(&arg[5], &size, 1, 0x100);
		} else {
			fprintf(stderr, "what you talkin' 'bout Wilis? %s\n", arg);
			show_usage();
			exit (-EINVAL);
		}
	}

	fprintf(stderr, "eeprom_read_write: addr %u, size %u\n", addr, size);

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;
	eeprom = &data->body.eeprom;
	eeprom->addr = addr;
	eeprom->size = size;
	eeprom->is_read = is_read;

	if (!is_read) {
		ret = get_input_data(eeprom->data, eeprom->size);
		if (ret != eeprom->size)
			goto exit_free;
	}

//	config.fileout = "a.out";
//	ret = put_output_data(data, struct_size);

	ret = mcp2210_do_ioctl(config.name, MCP2210_IOCTL_EEPROM, data);

	if (is_read) {
//		config.fileout = "b.out";
		ret = put_output_data(eeprom->data, eeprom->size);
//		ret = put_output_data(data, struct_size);
	}

exit_free:
	free(data);
	return ret;
}
#if 0
int asdfasdfasdfqwerwerwerxcvxcvxcv(void) {
	struct mcp2210_ioctl_data *data;
	struct mcp2210_ioctl_data_config *cfg;
	struct mcp2210_device dev;
	const size_t struct_size = sizeof(*data) + 0x400;
//	unsigned char buf[0x100];
//	char strbuf[7] = "[0] = ";
	int ret = 0;
//	unsigned i;

	data = malloc(struct_size);
	if (!data) {
		fatal_error("no mem");
		return -ENOMEM;
	}

	memset(data, 0, struct_size);
	data->struct_size = struct_size;
	cfg = &data->body.config;

	if (mcp2210_open(&dev, config.name))
		fatal_error("open failed");

	ret = mcp2210_ioctl(&dev, MCP2210_IOCTL_CONFIG_GET, (unsigned long)data);
	if (ret < 0)
		fatal_error("ioctl failed");

	fprintf(stderr, "is_spi_probed:  %hhu\n", cfg->is_spi_probed);
	fprintf(stderr, "is_gpio_probed: %hhu\n", cfg->is_gpio_probed);
	fprintf(stderr, "have_config: %hhu\n", cfg->have_config);
	dump_chip_settings("", 0, ".chip = ", &cfg->chip);

	dump_board_config("", 0, ".config = ", &cfg->config);
#if 0
	for (i = 0; i < 9; ++i) {
		strbuf[1] = '0' + i;
		dump_pin_config("", 2, strbuf, &cfg.config.pins[i]);
	}
#endif

	free(data);
	mcp2210_close(&dev);
	return ret;

/*
	cfg.config.pins[0].name[0] = 'l';
	ret = ioctl(fd, mcp2210_ioctl_map[MCP2210_IOCTL_CONFIG_SET], &cfg);
	if (ret == -1)
		pabort("config");
*/
#if 0

	eeprom_cmd.addr = 0;
	eeprom_cmd.size = 0x1;
	eeprom_cmd.is_read = 1;
	eeprom_cmd.buf = buf;

	ret = ioctl(fd, mcp2210_ioctl_map[MCP2210_IOCTL_EEPROM], &eeprom_cmd);
	if (ret == -1)
		pabort("read");

	printf("value = 0x%02hhx\n", buf[0]);

	eeprom_cmd.is_read = 0;
	buf[0] = 0xc0;
	buf[1] = 0x1d;
	ret = ioctl(fd, mcp2210_ioctl_map[MCP2210_IOCTL_EEPROM], &eeprom_cmd);
	if (ret == -1)
		pabort("write");

#if 0
	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	transfer(fd);
#endif
#endif

	//mcp2210_close(&dev);
	return ret;
}
#endif

struct command {
	const char *name;
	int min_args;
	const struct command *sub_cmd;
	int (*func)(int argc, char *argv[]);
};

static int dispatch(int argc, char *argv[], const struct command cmd_list[]) {
	const struct command *cmd;

	if (argc < 1) {
		fprintf(stderr, "Not enough arguments supplied\n");
		show_usage();
		return -EINVAL;
	}

	for (cmd = cmd_list; cmd->name; ++cmd) {
		if (!strcmp(cmd->name, *argv)) {
			if (argc < cmd->min_args) {
				fprintf(stderr, "too few arguments supplied for command %s, expected %u, got %u\n", cmd->name, cmd->min_args, argc);
				show_usage();
				return -EINVAL;
			}
			--argc;
			++argv;
			return cmd->sub_cmd
			     ? dispatch(argc, argv, cmd->sub_cmd)
			     : cmd->func(argc, argv);
		}
	}

	fprintf(stderr, "Invalid argument `%s'\n", argv[0]);
	show_usage();
	return -EINVAL;
}

static int not_yet_implemented(int argc, char *argv[]) {
	fprintf(stderr, "not yet implemented\n");
	return 0;
}


static const struct command sub_cmd_list_get[] = {
	{
		.name = "config",
		.func = get_config,
	}, {}
};

static const struct command sub_cmd_list_set[] = {
	{
		.name = "config",
		.func = set_config,
	}, {}
};

static const struct command sub_cmd_list_eeprom[] = {
	{
		.name = "read",
		.func = eeprom_read,
	}, {
		.name = "write",
		.func = eeprom_write,
	}, {}
};

static const struct command sub_cmd_list_test[] = {
	{
		.name = "bitstream",
		.func = test_bit_stream,
	}, {
		.name = "config",
		.func = test_encoding,
	}, {}
};

static const struct command command_list[] = {
	{
		.name	  = "get",
		.min_args = 1,
		.sub_cmd  = sub_cmd_list_get,
	}, {
		.name	  = "set",
		.min_args = 1,
		.sub_cmd  = sub_cmd_list_set,
	}, {
		.name	  = "eeprom",
		.min_args = 1,
		.sub_cmd  = sub_cmd_list_eeprom,
	}, {
		.name	  = "test",
		.min_args = 1,
		.sub_cmd  = sub_cmd_list_test,
	}, {
		.name	  = "msg",
		.min_args = 0,
		.func	  = not_yet_implemented,
	}, {
		.name	  = "spi",
		.min_args = 0,
		.func	  = not_yet_implemented,
	}, { },

};

static void show_usage() {
	fprintf(stderr,
"Usage: %s [options] command [arguments]\n"
"Options:\n"
"  -h --help        Show this help\n"
"  -v --verbose     Verbose output\n"
"  -d --device      Device name (default /dev/usb2spi_bridge)\n"
"  -i --in <file>   An input file (default: standard in)\n"
"  -o --out <file>  An output file (default: standard out)\n"
"\n"
"SPI Options:\n"
/* "  -s --spi name[,mode[,speed[,bits_per_word]]]\n" */
"  -D --spidev     SPI device (default /dev/spidev1.1)\n"
"  -S --speed      max speed in Hz (default 20kHz)\n"
"  -e --delay      delay in uS between messages (default 0)\n"
"  -b --bpw        bits per word (default 8, !=8 unsupported by mcp2210)\n"
"  -l --loop       enable loopback (default disabled, unsupported by mcp2210)\n"
"  -H --cpha <1|0> clock phase (default 1)\n"
"  -O --cpol <1|0> clock polarity (default 1)\n"
"  -L --lsb        least significant bit first (default MSB first)\n"
"  -C --cs-high    chip select active high (default low)\n"
"  -3 --3wire      SDI/SDO signals shared (default disabled)\n"
"\n"
"Commands:\n"
"  get    config\n"
"  set    config\n"
"  eeprom \n"
"         read\n"
"         write\n"
"  test   \n"
"         bitstream\n"
"         config\n"
"  msg    Send a raw message to the MCP2210 device.\n"
"  spi    Send an SPI (specify device with -D)\n"
"\n", argv0);
}

static int process_args(int argc, char *argv[])
{
	int c;
/* FIXME: mightily broken :) */
	while (1) {
		static struct option long_options[] = {
			{ "help",    no_argument,	0, 'h' },
			{ "verbose", no_argument,	0, 'v' },
			{ "device",  required_argument, 0, 'd' },
			{ "in",      required_argument, 0, 'i' },
			{ "out",     required_argument, 0, 'o' },

			/*{ "spi",     required_argument, 0, 's' },*/
			{ "spidev",  required_argument, 0, 'D' },
			{ "speed",   required_argument, 0, 's' },
			{ "delay",   required_argument, 0, 'e' },
			{ "bpw",     required_argument, 0, 'b' },
			{ "loop",    no_argument,	0, 'l' },
			{ "cpha",    required_argument, 0, 'H' },
			{ "cpol",    required_argument, 0, 'O' },
			{ "lsb",     no_argument,	0, 'L' },
			{ "cs-high", no_argument,	0, 'C' },
			{ "3wire",   no_argument,	0, '3' },
			{ NULL, 0, 0, 0 },
		};

		/* getopt_long stores the option index here. */
		int option_index = 0;
		c = getopt_long(argc, argv, "hvd:i:o:D:s:e:b:lH:O:LC3", long_options, &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
		case '?': fprintf(stderr, "Invalid argument: %s\n", argv[optind]);
		/* fall-through */
		case 'h': show_usage(); return 0;
		case 'v': config.verbose = 1;				break;
		case 'd': config.name = optarg;				break;
		case 'i': config.filein = optarg;			break;
		case 'o': config.fileout = optarg;			break;

		/* SPI options */
		case 'D': config.spi.name = optarg;			break;
		case 's':
			get_uint_param(optarg, &config.spi.speed,
				       MCP2210_MIN_SPEED, MCP2210_MAX_SPEED);
			break;
		case 'e':
			get_uint_param(optarg, &config.spi.delay, 0, UINT_MAX);
			break;
		case 'b':
			get_uint_param(optarg, &config.spi.bits, 8, 8);
			break;
		case 'l': config.spi.mode |= SPI_LOOP;			break;
		case 'H': if (!get_bool_param(optarg))
				config.spi.mode &= ~SPI_CPHA;		break;
		case 'O': if (!get_bool_param(optarg))
				config.spi.mode &= ~SPI_CPOL;		break;
		case 'L': config.spi.mode |= SPI_LSB_FIRST;		break;
		case 'C': config.spi.mode |= SPI_CS_HIGH;		break;
		case '3': config.spi.mode |= SPI_3WIRE;			break;
		default:
			return -EINVAL;
			break;
		}
	}

	return dispatch(argc - optind, &argv[optind], command_list);
}


int main(int argc, char *argv[])
{
	int ret;

	argv0 = argv[0];
	if ((ret = process_args(argc, argv))) {
		errno = ret;
		perror("main");
		errno = -ret;
		perror("main");
	}
	return ret;
}


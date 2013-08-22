/*
 *  MCP2210 Creek auto-configure support
 *
 *  Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
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

/**
 * @file
 * Creek is just another home-grown hardware configuration scheme where data is
 * stored in a compact fashion on a device (in this case, the MCP2210's usable
 * EEPROM) and used to configure a device.
 *
 * Creek -- because if it doesn't work, you're up one.
 *
 * Format of a Creek configuration
 * 00:0 - 03:7   (32 bits) magic
 * 04:0 - 04:3   (4 bits)  version
 * 04:4 - 06:5   (18 bits) 9x pairs of two bits: 1 = has_name, 2 = has_modalias
 * 06:6 - varies 9x SPI Data
 *               strings
 *
 * SPI data is encoded in a manner where each field is compared to a common or
 * standard value. If it matches that value, then a single 0 bit is written
 * and nothing else.  If it doesn't match, then a single 1 bit is written
 * followed by the value as a floating point unsigned interger. The exception
 * is mode, which is always 8 bits.
 *
 * SPI Data
 * Field                 Default Value     Precision  Magnitude
 * ------------------------------------------------------------
 * max_speed_hz          MCP2210_MAX_SPEED 10         2
 * min_speed_hz          MCP2210_MIN_SPEED 10         2
 * mode                --always 8 bits--
 * cs_to_data_delay      0                 7          2
 * last_byte_to_cs_delay 0                 7          2
 * delay_between_bytes   0                 7          2
 * delay_between_xfers   0                 7          2
 *
 * Strings are encoded as 7 bit ASCII with bit 7 terminatng the string.  (We
 * could get better compression by writing string sizes first and then limiting
 * them to 6 bits, but we seem to have plenty of room so far.)
 *
 */

#ifndef _MCP2210_CREEK_H
#define _MCP2210_CREEK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

static const u8 CREEK_CONFIG_MAGIC[4] = {0xc0, 0x1d, 0xf0, 0x0d};

/**
 * struct bit_creek - a struct for managing a buffer as a FIFO stream of bits
 */
struct bit_creek {
	u8 *start;
	size_t size;
	size_t pos;
	u8 overflow:1;
};

#define BIT_CREEK_INIT(buf, size) {buf, size, 0, 0,}

struct creek_data {
	u8 magic[4];
	u8 ver:4;
	u8 spi_pin_num[MCP2210_NUM_PINS];
	u8 spi_count;
	u8 name_index[MCP2210_NUM_PINS];
	u8 modalias_index[MCP2210_NUM_PINS];
	u8 str_size;
	const char **string_index;
	u8 str_count;
};

uint creek_get_bits(struct bit_creek *bs, uint num_bits);
int creek_put_bits(struct bit_creek *bs, uint value, uint num_bits);
int creek_encode(const struct mcp2210_board_config *src,
		 const struct mcp2210_chip_settings *chip, u8* dest,
		 size_t size);
struct mcp2210_board_config *creek_decode(
		const struct mcp2210_chip_settings *chip_settings,
		const u8* src, size_t size, gfp_t gfp_flags);

#if 0
static const uint pow10[] = {
	1,
	10,
	100,
	1000,
	10000,
	100000,
	1000000,
	10000000,
	100000000,
	1000000000,
};
#endif

/**
 * pack_uint - Pack a positive integral value into a smaller space by
 *	       tossing out precision.
 *
 * @value:	The (normal) value
 * @value_bits:	Size (in bits) of the value portion of the result (currently
 * 		requires a compile-time constant)
 * @scale_bits:	Size (in bits) of the scale portion of the result (currently
 * 		requires a compile-time constant)
 *
 * This mechanism for packing numbers allows for a wide range of values with a
 * limited precision.  A "10:2" packed value has 10 bits of value and 2 bits of
 * scale allowing for the values 987, 9870, 98700 or 987000, but not 9879,
 * 98799, etc.  In other words, the least significant portion of the value is
 * zeroed in the conversion process.
 *
 * @returns Upon error, (uint)-1.  Otherwise, the result which is
 * 	    guaranteed to fit into value_bits + scale_bits bits.
 */
static inline uint pack_uint(uint value, uint value_bits, uint scale_bits)
{
	const uint num_end = (1 << value_bits);
	const uint scale_end = (1 << scale_bits);
	uint num;
	uint scale;
	uint mult;
	uint ret;

	BUILD_BUG_ON(scale_bits + value_bits >= sizeof(uint) * 8);

	for (scale = 0, mult = 1; scale < scale_end; ++scale, mult *= 10) {
		num = value / mult;
		if (num < num_end) {
			ret = num | scale << value_bits;

			/* some sanity checks until we know that this is correct */
			BUG_ON(num   & ~((1 << value_bits) - 1));
			BUG_ON(scale & ~((1 << scale_bits) - 1));
			BUG_ON(ret   & ~((1 << (value_bits + scale_bits)) - 1));

			return ret;
		}
	}

	BUG();

	return (uint)-1;
}

/**
 * unpack_uint - Unpack a packed positive integral number.
 *
 * @packed:	The packed value
 * @value_bits:	Size (in bits) of the value portion of packed (currently
 * 		requires a compile-time constant)
 * @scale_bits:	Size (in bits) of the scale portion of packed (currently
 * 		requires a compile-time constant)
 *
 * @see mcp2210_pack_uint
 *
 * @returns The unpacked value.
 */
static inline uint unpack_uint(uint packed, uint value_bits, uint scale_bits)
{
	const uint value_mask = (1 << value_bits) - 1;
	const uint scale_mask = (1 << scale_bits) - 1;
	const uint value = packed & value_mask;
	const uint scale = (packed >> value_bits) & scale_mask;
	uint i, mult;

	BUG_ON(scale_bits + value_bits >= sizeof(uint) * 8);
	BUG_ON(scale > 9);

	for (i = 0, mult = 1; i != scale; ++i, mult *= 10);
	return value * mult;

//	return value * pow10[scale];
}

#ifdef __KERNEL__
struct mcp2210_board_config *mcp2210_creek_probe(struct mcp2210_device *dev,
						 gfp_t gfp_flags);
#endif /* __KERNEL__ */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // _MCP2210_CREEK_H

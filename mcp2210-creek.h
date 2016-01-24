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
 * Format of Creek Encoding
 *
 * Start  End  Descrption                             Size In Bits
 * -------------------------------------------------------------
 * 0      31   magic (0xc01df00d)                     32
 * 32     35   version                                4
 * 36     53   9x Presence of Strings                 18
 * 54     ?    3-Wire Capability                      1 to 5
 * ?      ?    IRQs                                   1 to 63
 * ?      ?    Polling Data (only if IRQs are used)   0 to 52
 * ?      ?    9x SPI Data                            0 to 648
 * ?      ?    Strings                                arbitrary
 *             Total                                  56 to 2048
 *
 * Polling Data and SPI data use an encoding where where each field is
 * compared to a common or standard value. If it matches that value, then a
 * single 0 bit is written and nothing else.  If it doesn't match, then a
 * single 1 bit is written followed by the value as a floating point unsigned
 * interger (see pack_uint_opt() and unpack_uint_opt() for details). In the
 * tables below, the type "uint_opt" indicates this type of storage (not an
 * actual C type).
 *
 * Presence of Strings
 * Field                 Num Bits  Presence
 * ----------------------------------------------------------------------
 * has_name              1         always
 * has_modalias          1         always
 *
 *
 * 3-Wire Capability
 * Field                 Num Bits  Presence
 * ----------------------------------------------------------------------
 * _3wire_capable        1         always
 * _3wire_tx_enable_active_high
 *                       1         only if _3wire_capable
 * _3wire_tx_enable_pin  3         only if _3wire_capable
 *
 *
 * IRQs
 * Field                 Num Bits  Presence
 * ----------------------------------------------------------------------
 * has_irq               1         only if spi, gpio or dedicated pin 6
 * irq.num               3         only if has_irq
 * irq.type              3         only if has_irq and gpio
 *
 *
 * Polling Data
 * This section is only present if one or more pins are configured to trigger
 * an IRQ.
 *
 * Field                 Type      Default Value     Precision  Magnitude
 * ----------------------------------------------------------------------
 * poll_gpio_usecs (1)   uint                       10          3
 * stale_gpio_usecs (1)  uint_opt  poll_gpio_usecs  10          3
 * same_as_gpio (2)      bit
 * poll_intr_usecs (3)   uint                       10          3
 * stale_intr_usecs (3)  uint_opt  poll_intr_usecs  10          3
 *
 * Footnotes:
 * 1. Only if a gpio pin exists that uses an IRQ, otherwise not written.
 * 2. Only if pin 6 is dedicated and has an IRQ and a gpio with an irq exists, otherwise not written.
 * 3. Only if requirements for both (1) and (2) are met and same_as_gpio is
 *    set, otherwise not written.
 *
 *
 * SPI Data
 * Field                 Type      Presence  Default Value     Precision  Mag.
 * ---------------------------------------------------------------------------
 * max_speed_hz          uint_opt  always    MCP2210_MAX_SPEED 10         2
 * min_speed_hz          uint_opt  always    MCP2210_MIN_SPEED 10         2
 * mode                  8 bits    always
 * use_cs_gpio           1 bit     always
 * cs_gpio               3 bits,   only if use_cs_gpio
 * cs_to_data_delay      uint_opt  always    0                 7          2
 * last_byte_to_cs_delay uint_opt  always    0                 7          2
 * delay_between_bytes   uint_opt  always    0                 7          2
 * delay_between_xfers   uint_opt  always    0                 7          2
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
	u8 str_count;
	u8 str_size;
	const char **string_index;
	u8 spi_pin_num[MCP2210_NUM_PINS];
	u8 spi_count;
	u8 name_index[MCP2210_NUM_PINS];
	u8 modalias_index[MCP2210_NUM_PINS];
	u8 have_gpio_irqs;
};

uint creek_get_bits(struct bit_creek *bs, uint num_bits);
int creek_put_bits(struct bit_creek *bs, uint value, uint num_bits);
int creek_encode(const struct mcp2210_board_config *src,
		 const struct mcp2210_chip_settings *chip, u8* dest,
		 size_t size);
struct mcp2210_board_config *creek_decode(
		const struct mcp2210_chip_settings *chip_settings,
		const u8* src, size_t size, gfp_t gfp_flags);

#ifdef __KERNEL__
struct mcp2210_board_config *mcp2210_creek_probe(struct mcp2210_device *dev,
						 gfp_t gfp_flags);
#endif /* __KERNEL__ */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // _MCP2210_CREEK_H

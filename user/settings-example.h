/*
 * MCP2210 hacky edit-this-file header for settings
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

/**
 * @file due to the fact that I have no better means to do configuration, edit
 * this file to your needs and recompile the userspace utility
 */

#ifndef _SETTINGS_H
#define _SETTINGS_H

static const struct mcp2210_chip_settings my_chip_settings = {
	.pin_mode = {
		MCP2210_PIN_SPI,
		MCP2210_PIN_GPIO,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_GPIO,
		MCP2210_PIN_SPI,
		MCP2210_PIN_GPIO,
	},
	.gpio_value		= 3,
	.gpio_direction		= 3,
	.other_settings		= 0x01,
	.nvram_access_control	= 0,
	.password = {0, 0, 0, 0, 0, 0, 0, 0},
};

static const struct mcp2210_chip_settings my_power_up_chip_settings = {
	.pin_mode = {
		MCP2210_PIN_SPI,
		MCP2210_PIN_GPIO,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_DEDICATED,
		MCP2210_PIN_GPIO,
		MCP2210_PIN_SPI,
		MCP2210_PIN_GPIO,
	},
	.gpio_value		= 3,
	.gpio_direction		= 3,
	.other_settings		= 0x01,
	.nvram_access_control	= 0,
	.password = {0, 0, 0, 0, 0, 0, 0, 0},
};

static const struct mcp2210_spi_xfer_settings my_spi_settings = {
	.bitrate		= MCP2210_MAX_SPEED,
	.idle_cs		= 0x01ff,
	.active_cs		= 0x0000,
	.cs_to_data_delay	= 1,
	.last_byte_to_cs_delay	= 1,
	.delay_between_bytes	= 1,
	.bytes_per_trans	= 4,
	.mode			= SPI_MODE_3
};

static const struct mcp2210_spi_xfer_settings my_power_up_spi_settings = {
	.bitrate		= MCP2210_MAX_SPEED,
	.idle_cs		= 0x01ff,
	.active_cs		= 0x0000,
	.cs_to_data_delay	= 1,
	.last_byte_to_cs_delay	= 1,
	.delay_between_bytes	= 1,
	.bytes_per_trans	= 4,
	.mode			= SPI_MODE_3
};

static const struct mcp2210_usb_key_params my_usb_key_params = {
	.vid		   = USB_VENDOR_ID_MICROCHIP,
	.pid		   = USB_DEVICE_ID_MCP2210,
	.chip_power_option = 0x80,
	.requested_power   = 0xfa, /* 500mA */
};

static const struct mcp2210_board_config my_board_config = {
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
			.body.gpio.direction = MCP2210_GPIO_OUTPUT,
			.body.gpio.init_value = 1,
		}, {
			.mode = MCP2210_PIN_DEDICATED,
			.name = "SSPND"
		}, {
			.mode = MCP2210_PIN_DEDICATED,
			.name = "LED",
		}, {
			.mode = MCP2210_PIN_DEDICATED,
			.name = "LOWPWR",
		}, {
			.mode = MCP2210_PIN_DEDICATED,
			.name = "USBCFG",
		}, {
			.mode = MCP2210_PIN_GPIO,
			.body.gpio.direction = MCP2210_GPIO_INPUT,
			.name = "MOTION",
		}, {
			.mode = MCP2210_PIN_SPI,
			.body.spi.max_speed_hz = 20000,
			.body.spi.min_speed_hz = 5000,
			.body.spi.mode = SPI_MODE_3,
			.body.spi.bits_per_word = 8,
			.body.spi.cs_to_data_delay = 2,
			.body.spi.last_byte_to_cs_delay = 2,
			.body.spi.delay_between_bytes = 4,
			.body.spi.delay_between_xfers = 0,
			.modalias = "spidev",
			.name = "ADNS-9800",
		}, {
			.mode = MCP2210_PIN_UNUSED,
			.name = "fried pin",
		}
	},
	.strings_size = 0,
};

#endif /* _SETTINGS_H */

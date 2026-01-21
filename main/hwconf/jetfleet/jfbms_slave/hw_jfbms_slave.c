/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw_jfbms_slave.h"

#include "bq769x2_defs.h"

#include "heap.h"
#include "lbm_defines.h"
#include "main.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"
#include "comm_can.h"
#include "bms.h"
#include "buffer.h"

#include <math.h>
#include <stdint.h>

// Settings
#define BQ_ADDR_1 0x08
#define BQ_ADDR_2 0x08  // Same address, selected via enable pins
#define I2C_SPEED 100000

// Macros
#define M_CELLS (m_cells_ic1 + m_cells_ic2)

// Variables
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t bq_mutex;
static unsigned int m_cells_ic1 = 16;
static unsigned int m_cells_ic2 = 0;
static uint16_t m_bal_state_ic1 = 0;
static uint16_t m_bal_state_ic2 = 0;

// Error messages
static char *error_comm_bq1 = "BQ1 communication error";
static char *error_comm_bq2 = "BQ2 communication error";

// ============================================================================
// BMS Master-Slave CAN Protocol (11-bit Standard IDs)
// ============================================================================
// CAN ID Format: (msg_type << 7) | slave_id
//
// Message Types (Slave -> Master):
//   0x0-0x7 = Cell voltages (4 cells per message, 8 messages for 32 cells)
//   0x8     = Temperatures (4 temps)
//   0x9     = Status (balance mask + faults)
//
// Message Types (Master -> Slave):
//   0xA     = Balance command (32-bit balance mask)
//
// Data Format:
//   - All multi-byte values are little-endian
//   - Cell voltages: uint16 mV (0x0000 = not populated, 0xFFFF = read error)
//   - Temperatures: int16 0.1°C (0x7FFF = not present/invalid)
// ============================================================================

// CAN ID macros per protocol spec
#define CAN_ID_CELLS(type, slave_id)  (((type) << 7) | (slave_id))
#define CAN_ID_TEMPS(slave_id)        (0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)       (0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)      (0x500 | (slave_id))

// Forward declarations (functions defined later in file)
static void select_bq_chip(uint8_t chip_num);
static bool subcommands_write16(uint8_t dev_addr, uint16_t command, uint16_t data);

// ============================================================================
// CAN Protocol TX Functions
// ============================================================================

/**
 * Send all 32 cell positions (8 CAN messages, 4 cells each)
 * @param slave_id  Slave ID (1-8)
 * @param cells_mv  Array of 32 cell voltages in mV (0 = not populated, 0xFFFF = error)
 */
static void can_send_all_cells(uint8_t slave_id, uint16_t *cells_mv) {
	for (uint8_t msg_type = 0; msg_type < 8; msg_type++) {
		uint8_t buf[8];
		uint8_t base_cell = msg_type * 4;

		// Pack 4 cells per message, little-endian
		for (uint8_t i = 0; i < 4; i++) {
			uint16_t v = cells_mv[base_cell + i];
			buf[i * 2]     = v & 0xFF;         // Low byte
			buf[i * 2 + 1] = (v >> 8) & 0xFF;  // High byte
		}

		comm_can_transmit_sid(CAN_ID_CELLS(msg_type, slave_id), buf, 8);
	}
}

/**
 * Send 4 temperatures (1 CAN message)
 * @param slave_id  Slave ID (1-8)
 * @param temps     Array of 4 temperatures in 0.1°C (0x7FFF = invalid)
 */
static void can_send_temps(uint8_t slave_id, int16_t *temps) {
	uint8_t buf[8];

	// Pack 4 temps, little-endian (T_BQ1, T_TS1, T_TS3, T_BQ2)
	for (uint8_t i = 0; i < 4; i++) {
		buf[i * 2]     = temps[i] & 0xFF;         // Low byte
		buf[i * 2 + 1] = (temps[i] >> 8) & 0xFF;  // High byte
	}

	comm_can_transmit_sid(CAN_ID_TEMPS(slave_id), buf, 8);
}

/**
 * Send status message (1 CAN message, 6 bytes)
 * @param slave_id  Slave ID (1-8)
 * @param bal_mask  32-bit balance bitmap
 * @param faults    Fault byte (bit0 = BQ1 init failed, bit1 = BQ2 init failed)
 * @param cell_count Total number of configured cells (cells_ic1 + cells_ic2)
 */
static void can_send_status(uint8_t slave_id, uint32_t bal_mask, uint8_t faults, uint8_t cell_count) {
	uint8_t buf[6];

	// Balance mask, little-endian
	buf[0] = (bal_mask >> 0) & 0xFF;
	buf[1] = (bal_mask >> 8) & 0xFF;
	buf[2] = (bal_mask >> 16) & 0xFF;
	buf[3] = (bal_mask >> 24) & 0xFF;
	buf[4] = faults;
	buf[5] = cell_count;

	comm_can_transmit_sid(CAN_ID_STATUS(slave_id), buf, 6);
}

// Get current balancing bitmap from both ICs
static uint32_t get_bal_bitmap(void) {
	return (uint32_t)m_bal_state_ic1 | ((uint32_t)m_bal_state_ic2 << 16);
}

// Apply balancing from bitmap
// Note: BQ76952 requires TOGGLE (0 then value) to reset internal ~18s timeout
// Just writing the same value does NOT reset the timer!
static bool apply_bal_bitmap(uint32_t bitmap) {
	uint16_t new_bal_ic1 = bitmap & 0xFFFF;
	uint16_t new_bal_ic2 = (bitmap >> 16) & 0xFFFF;
	bool res = true;

	// BQ1: Toggle - write 0 first, then actual value to reset internal timer
	select_bq_chip(1);
	subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, 0);  // Clear first
	if (subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, new_bal_ic1)) {
		m_bal_state_ic1 = new_bal_ic1;
	} else {
		res = false;
	}

	// BQ2: Toggle if present
	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, 0);  // Clear first
		if (subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, new_bal_ic2)) {
			m_bal_state_ic2 = new_bal_ic2;
		} else {
			res = false;
		}
	}

	return res;
}

// Stop all balancing
static bool stop_all_balancing(void) {
	return apply_bal_bitmap(0);
}

static esp_err_t i2c_tx_rx(
	uint8_t addr, const uint8_t *write_buffer, size_t write_size,
	uint8_t *read_buffer, size_t read_size
) {

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	esp_err_t res;
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(
				0, addr, write_buffer, write_size, read_buffer, read_size, 500
			);
		} else {
			res = i2c_master_read_from_device(
				0, addr, read_buffer, read_size, 500
			);
		}
	} else {
		res =
			i2c_master_write_to_device(0, addr, write_buffer, write_size, 500);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static uint8_t crc8(uint8_t *ptr, uint8_t len) {
	uint8_t i;
	uint8_t crc = 0;

	while (len-- != 0) {
		for (i = 0x80; i != 0; i /= 2) {
			if ((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= 0x107;
			} else {
				crc *= 2;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x107;
			}
		}
		ptr++;
	}

	return (crc);
}

/**
 * select_bq_chip - Select active BQ chip via enable pins
 * @chip_num: 1 for BQ1, 2 for BQ2
 *
 * Ensures only one chip is active on shared I2C bus by controlling
 * PIN_BQ1_EN and PIN_BQ2_EN (active LOW logic).
 */
static void select_bq_chip(uint8_t chip_num) {
	// chip_num: 1 or 2
	// Active LOW logic: 0 = enabled, 1 = disabled
	if (chip_num == 1) {
		gpio_set_level(PIN_BQ1_EN, 0);  // Enable BQ1
		gpio_set_level(PIN_BQ2_EN, 1);  // Disable BQ2
	} else if (chip_num == 2) {
		gpio_set_level(PIN_BQ2_EN, 0);  // Enable BQ2
		gpio_set_level(PIN_BQ1_EN, 1);  // Disable BQ1
	}
	vTaskDelay(1);  // Small delay for chip selection to settle
}

static bool bq_read_block(
	uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len
) {
	uint8_t read_data[2 * len];
	esp_err_t res          = i2c_tx_rx(dev_addr, &reg, 1, read_data, 2 * len);
	uint8_t *read_data_ptr = read_data;

	if (res != ESP_OK) {
		commands_printf_lisp("I2C Error: %d", res);
		return false;
	}

	uint8_t crcbuf[4];
	crcbuf[0]   = dev_addr << 1;
	crcbuf[1]   = reg;
	crcbuf[2]   = (dev_addr << 1) + 1;
	crcbuf[3]   = *read_data_ptr;
	uint8_t crc = crc8(crcbuf, 4);

	read_data_ptr++;
	if (crc != *read_data_ptr) {
		commands_printf_lisp("Bad CRC1");
		return false;
	} else {
		*buf = *(read_data_ptr - 1);
	}

	for (int i = 1; i < len; i++) {
		read_data_ptr++;
		crc = crc8(read_data_ptr, 1);
		read_data_ptr++;
		buf++;

		if (crc != *read_data_ptr) {
			commands_printf_lisp("Bad CRC2");
			return false;
		} else {
			*buf = *(read_data_ptr - 1);
		}
	}

	return true;
}

static bool bq_write_block(
	uint8_t dev_addr, uint8_t start_addr, uint8_t *buf, uint8_t len
) {
	uint8_t txbuf[2 * len + 2];
	txbuf[0] = dev_addr << 1;
	txbuf[1] = start_addr;
	txbuf[2] = buf[0];
	txbuf[3] = crc8(txbuf, 3);

	for (int i = 1; i < len; i++) {
		txbuf[2 + (2 * i)] = buf[i];
		txbuf[3 + (2 * i)] = crc8(&buf[i], 1);
	}

	esp_err_t res = i2c_tx_rx(dev_addr, txbuf + 1, 2 * len + 1, NULL, 0);

	return res == ESP_OK;
}

static uint8_t checksum(uint8_t *ptr, int len) {
	uint8_t sum = 0;

	for (int i = 0; i < len; i++) {
		sum += ptr[i];
	}

	return ~sum;
}

static bool bq_set_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t reg_data, uint8_t datalen
) {
	uint8_t TX_Buffer[2]  = {0x00, 0x00};
	uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	bool res = false;

	// TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch (datalen) {
		case 1: //1 byte datalength
			bq_write_block(dev_addr, 0x3E, TX_RegData, 3);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 3);
			TX_Buffer[1] = 0x05; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
		case 2: //2 byte datalength
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			bq_write_block(dev_addr, 0x3E, TX_RegData, 4);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 4);
			TX_Buffer[1] = 0x06; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
		case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			TX_RegData[4] = (reg_data >> 16) & 0xff;
			TX_RegData[5] = (reg_data >> 24) & 0xff;
			bq_write_block(dev_addr, 0x3E, TX_RegData, 6);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 6);
			TX_Buffer[1] = 0x08; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
	}

	return res;
}

static bool bq_read_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t *reg_data, uint8_t datalen
) {
	uint8_t TX_RegData[2] = {0x00, 0x00};
	uint8_t RX_RegData[4] = {0x00, 0x00, 0x00, 0x00};

	if (datalen > 4) {
		datalen = 4;
	}

	bool res = false;

	// TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;

	bq_write_block(dev_addr, 0x3E, TX_RegData, 2);
	vTaskDelay(2);
	res = bq_read_block(dev_addr, 0x40, RX_RegData, datalen);

	if (res) {
		*reg_data = (((uint32_t)RX_RegData[3]) << 24)
			| (((uint32_t)RX_RegData[2]) << 16)
			| (((uint32_t)RX_RegData[1]) << 8)
			| (((uint32_t)RX_RegData[0]) << 0);
	} else {
		*reg_data = 0;
	}

	return res;
}

static int16_t command_read(uint8_t dev_addr, uint8_t command, bool *ok) {
	if (ok) {
		*ok = false;
	}
	uint8_t RX_data[2] = {0, 0};
	if (bq_read_block(dev_addr, command, RX_data, 2)) {
		if (ok) {
			*ok = true;
		}
		return (int16_t)(((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0]);
	} else {
		return -1;
	}
}

static bool command_subcommands(uint8_t dev_addr, uint16_t command) {
	// For DEEPSLEEP/SHUTDOWN subcommand you will need to
	// call this function twice consecutively

	uint8_t TX_Reg[2] = {0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 2);
	vTaskDelay(2);
	return res;
}

static bool __attribute__((unused)) subcommands_read16(
	uint8_t dev_addr, uint16_t command, uint16_t *result
) {
	uint8_t TX_Reg[2] = {0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(2);

	uint8_t RX_data[2] = {0, 0};
	res                = bq_read_block(dev_addr, 0x40, RX_data, 2);

	if (!res) {
		return false;
	}

	*result = (int16_t)(((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0]);

	return true;
}

static bool subcommands_write16(
	uint8_t dev_addr, uint16_t command, uint16_t data
) {
	uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;
	TX_Reg[2] = data & 0xff;
	TX_Reg[3] = (data >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 4);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	TX_Reg[0] = checksum(TX_Reg, 4);
	TX_Reg[1] = 0x06;

	res = bq_write_block(dev_addr, 0x60, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	return true;
}

static bool subcommands_write8(
	uint8_t dev_addr, uint16_t command, uint8_t data
) {
	uint8_t TX_Reg[3] = {0x00, 0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;
	TX_Reg[2] = data;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 3);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	TX_Reg[0] = checksum(TX_Reg, 3);
	TX_Reg[1] = 0x05;

	res = bq_write_block(dev_addr, 0x60, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	return true;
}

static uint32_t float_to_u(float number) {
	// Set subnormal numbers to 0 as they are not handled properly
	// using this method.
	if (fabsf(number) < 1.5e-38) {
		number = 0.0;
	}

	int e          = 0;
	float sig      = frexpf(number, &e);
	float sig_abs  = fabsf(sig);
	uint32_t sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e    += 126;
	}

	uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1U << 31;
	}

	return res;
}

static void bq_init(uint8_t dev_addr) {
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	vTaskDelay(10);

	//command_subcommands(dev_addr, BQ769x2_RESET);
	//vTaskDelay(60);

	// Disable all FETs (BQ76952 not used for FET control, only cell voltage monitoring)
	// 0x0F = all FETs OFF (bit 0 = DSG FET, bit 2 = CHG FET, bit 1,3 = other FETs)
	subcommands_write8(dev_addr, FET_CONTROL, 0x0F);

	command_subcommands(dev_addr, SET_CFGUPDATE);
	command_subcommands(dev_addr, SET_CFGUPDATE);

	// DPSLP_OT: 1
	// SHUT_TS2: 0
	// DPSLP_PD: 0
	// DPSLP_LDO: 1
	// DPSLP_LFO: 1
	// SLEEP: 0
	// OTSD: 1
	// FASTADC: 0
	// CB_LOOP_SLOW: 0
	// LOOP_SLOW: 0
	// WK_SPD: 0
	bq_set_reg(dev_addr, PowerConfig, 0b0010011010000000, 2);
	// Sometimes the first write has no effect. Do a few extra writes just in case...
	bq_set_reg(dev_addr, PowerConfig, 0b0010011010000000, 2);

	// REG0_EN: 1
	bq_set_reg(dev_addr, REG0Config, 0x01, 1);

	// REG1V: 6 (3.3v)
	// REG1_EN: 1
	bq_set_reg(dev_addr, REG12Config, 0b00001101, 1);
	
	// FETOptions
	// 5: FET_INIT_OFF
	// 4: PDSG_EN
	// 3: FET_CTRL_EN
	// 2: HOST_FET_EN
	// 1: SLEEPCHG
	// 0: SFET
	bq_set_reg(dev_addr, FETOptions, 0b00101100, 1);

	// Disabled
	bq_set_reg(dev_addr, CFETOFFPinConfig, 0x00, 1);
	bq_set_reg(dev_addr, DFETOFFPinConfig, 0x00, 1);

	// ADC inputs with 18k pull-up
	bq_set_reg(dev_addr, TS1Config, 0b00111011, 1);
	bq_set_reg(dev_addr, TS3Config, 0b00111011, 1);
	bq_set_reg(dev_addr, ALERTPinConfig, 0b00111011, 1);
	bq_set_reg(dev_addr, DCHGPinConfig, 0b00111011, 1);
	bq_set_reg(dev_addr, HDQPinConfig, 0b00111011, 1);

	// Disabled
	bq_set_reg(dev_addr, DDSGPinConfig, 0x00, 1);

	// Use all cells
	bq_set_reg(dev_addr, VCellMode, 0x0000, 2);

	// Disable automatic protections
	bq_set_reg(dev_addr, EnabledProtectionsA, 0x00, 1);
	bq_set_reg(dev_addr, EnabledProtectionsB, 0x00, 1);

	// Host-controlled balancing
	bq_set_reg(dev_addr, BalancingConfiguration, 0x00, 1);

	// Current gain
	float cc_gain = 7.4768 / (HW_R_SHUNT * 1000.0);
	bq_set_reg(dev_addr, CCGain, float_to_u(cc_gain), 4);
	bq_set_reg(dev_addr, CapacityGain, float_to_u(cc_gain * 298261.6178), 4);

	// Voltage and current reporting, 1 mV and 10 mA (range +- 320A)
	bq_set_reg(dev_addr, DAConfiguration, 0b00011110, 1);

	command_subcommands(dev_addr, EXIT_CFGUPDATE);

	vTaskDelay(10);

	command_subcommands(dev_addr, SLEEP_DISABLE);
}

// Extensions
static lbm_value ext_bms_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;

	unsigned int cells_ic1 = 16;
	if (argn >= 1) {
		cells_ic1 = lbm_dec_as_u32(args[0]);
	}

	unsigned int cells_ic2 = 0;  // Default to 0 (single chip mode)
	if (argn >= 2) {
		cells_ic2 = lbm_dec_as_u32(args[1]);
	}

	// Validation
	if (cells_ic1 < 3 || cells_ic1 > 16 || cells_ic2 > 16) {
		lbm_set_error_reason("Invalid cell combination");
		return ENC_SYM_TERROR;
	}

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	// Restart i2c
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	i2c_driver_delete(0);
	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_SPEED,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	i2c_reset_tx_fifo(0);
	i2c_reset_rx_fifo(0);

	vTaskDelay(10);
	xSemaphoreGive(i2c_mutex);

	// Initialize BQ1
	select_bq_chip(1);
	bq_init(BQ_ADDR_1);

	// Initialize BQ2 if present
	if (cells_ic2 > 0) {
		select_bq_chip(2);
		bq_init(BQ_ADDR_2);
	}

	m_cells_ic1 = cells_ic1;
	m_cells_ic2 = cells_ic2;

	// Test communication with both chips
	bool res = false;
	select_bq_chip(1);
	command_read(BQ_ADDR_1, Cell2Voltage, &res);

	if (cells_ic2 > 0) {
		bool res2 = false;
		select_bq_chip(2);
		command_read(BQ_ADDR_2, Cell2Voltage, &res2);
		res = res && res2;
	}

	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_hw_sleep(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	// Stop balancing on both chips
	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;

	select_bq_chip(1);
	if (!subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1)) {
		goto exit_error1;
	}

	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		if (!subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2)) {
			goto exit_error2;
		}
	}

	// Configure BQ1 for sleep (disable temp pull-ups, keep regulator on)
	select_bq_chip(1);
	if (!command_subcommands(BQ_ADDR_1, SET_CFGUPDATE)
		|| !bq_set_reg(BQ_ADDR_1, PowerConfig, 0b0010011010000000, 2)
		|| !bq_set_reg(BQ_ADDR_1, TS1Config, 0x00, 1)
		|| !bq_set_reg(BQ_ADDR_1, TS3Config, 0x00, 1)
		|| !command_subcommands(BQ_ADDR_1, EXIT_CFGUPDATE)) {
		goto exit_error1;
	}

	// Configure BQ2 for sleep if present
	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		if (!command_subcommands(BQ_ADDR_2, SET_CFGUPDATE)
			|| !bq_set_reg(BQ_ADDR_2, PowerConfig, 0b0010011010000000, 2)
			|| !bq_set_reg(BQ_ADDR_2, TS1Config, 0x00, 1)
			|| !bq_set_reg(BQ_ADDR_2, TS3Config, 0x00, 1)
			|| !command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE)) {
			goto exit_error2;
		}
	}

	// Send DEEPSLEEP command to BQ1
	select_bq_chip(1);
	command_subcommands(BQ_ADDR_1, DEEPSLEEP);
	command_subcommands(BQ_ADDR_1, DEEPSLEEP);

	// Send DEEPSLEEP command to BQ2 if present
	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		command_subcommands(BQ_ADDR_2, DEEPSLEEP);
		command_subcommands(BQ_ADDR_2, DEEPSLEEP);
	}

	xSemaphoreGive(bq_mutex);
	return ENC_SYM_TRUE;

exit_error1:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static lbm_value ext_get_vcells(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value vc_list = ENC_SYM_NIL;

	// Read BQ1 cells
	select_bq_chip(1);
	for (int i = 0; i < m_cells_ic1; i++) {
		bool ok = false;
		int res = command_read(BQ_ADDR_1, Cell1Voltage + i * 2, &ok);
		if (ok) {
			vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
		} else {
			lbm_set_error_reason(error_comm_bq1);
			return ENC_SYM_EERROR;
		}
	}

	// Read BQ2 cells if present
	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		for (int i = 0; i < m_cells_ic2; i++) {
			bool ok = false;
			int res = command_read(BQ_ADDR_2, Cell1Voltage + i * 2, &ok);
			if (ok) {
				vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
			} else {
				lbm_set_error_reason(error_comm_bq2);
				return ENC_SYM_EERROR;
			}
		}
	}

	return lbm_list_destructive_reverse(vc_list);
}

#define NTC_TEMP(res, beta)                                                    \
	(1.0 / ((logf((res) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)
#define NTC_RES(volts) (18.0e3 / (1.8 / volts - 1.0) - 500.0)
// Return 999.0 for invalid NTC (will be converted to 0x7FFF in broadcast)
#define NTC_INVALID_MARKER 999.0f
#define NAN_TO_INVALID(x)  (UTILS_IS_NAN(x) ? NTC_INVALID_MARKER : x)

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value ts_list = ENC_SYM_NIL;
	bool ok           = false;

	// Read BQ1 internal temperature
	select_bq_chip(1);
	ts_list = lbm_cons(
		lbm_enc_float(
			(float)command_read(BQ_ADDR_1, IntTemperature, &ok) * 0.1 - 273.15
		),
		ts_list
	);
	if (!ok) {
		goto exit_error1;
	}

	// Multiply by 256 as only 16 of the 24 bits are used
	const float counts_to_volts = 0.358e-6 * 256.0;

	// Read TS1 (NTC sensor 1)
	float v1 = (float)command_read(BQ_ADDR_1, TS1Temperature, &ok) * counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}

	// Read TS3 (NTC sensor 2)
	float v3 = (float)command_read(BQ_ADDR_1, TS3Temperature, &ok) * counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}

	// TODO: Use config
	float ntc_beta = 3380.0;

	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_INVALID(NTC_TEMP(NTC_RES(v1), ntc_beta))), ts_list
	);
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_INVALID(NTC_TEMP(NTC_RES(v3), ntc_beta))), ts_list
	);

	// Read BQ2 internal temperature if present
	if (m_cells_ic2 > 0) {
		select_bq_chip(2);
		ts_list = lbm_cons(
			lbm_enc_float(
				(float)command_read(BQ_ADDR_2, IntTemperature, &ok) * 0.1 - 273.15
			),
			ts_list
		);
		if (!ok) {
			goto exit_error2;
		}
	} else {
		// BQ2 not present - mark as invalid
		ts_list = lbm_cons(lbm_enc_float(NTC_INVALID_MARKER), ts_list);
	}

	return lbm_list_destructive_reverse(ts_list);

exit_error1:
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static lbm_value ext_get_vout(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	bool ok = false;
	int16_t res = command_read(BQ_ADDR_1, LDPinVoltage, &ok);
	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float((float)res / 100.0);
}

static lbm_value ext_get_vstack(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	
	bool ok = false;
	int16_t res = command_read(BQ_ADDR_1, StackVoltage, &ok);
	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float((float)res / 100.0);
}

static lbm_value ext_set_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int state       = lbm_dec_as_i32(args[1]);
	bool res        = false;

	if (ch < m_cells_ic1) {
		// Control BQ1
		if (state) {
			m_bal_state_ic1 |= (1 << ch);
		} else {
			m_bal_state_ic1 &= ~(1 << ch);
		}

		select_bq_chip(1);
		res = subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1);
		if (!res) {
			lbm_set_error_reason(error_comm_bq1);
		}
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		// Control BQ2
		unsigned int local_ch = ch - m_cells_ic1;
		if (state) {
			m_bal_state_ic2 |= (1 << local_ch);
		} else {
			m_bal_state_ic2 &= ~(1 << local_ch);
		}

		select_bq_chip(2);
		res = subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2);
		if (!res) {
			lbm_set_error_reason(error_comm_bq2);
		}
	}

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_get_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int res         = -1;

	if (ch < m_cells_ic1) {
		res = (m_bal_state_ic1 >> ch) & 0x01;
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		res = (m_bal_state_ic2 >> (ch - m_cells_ic1)) & 0x01;
	}

	return lbm_enc_i(res);
}

static lbm_value ext_direct_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;

	bool ok = false;
	int res = command_read(addr, lbm_dec_as_u32(args[1]), &ok);
	if (ok) {
		return lbm_enc_i(res);
	} else {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_subcmd_cmdonly(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;
	return lbm_enc_i(command_subcommands(addr, lbm_dec_as_u32(args[1])));
}

static lbm_value ext_read_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	uint8_t addr = BQ_ADDR_1;
	int reg      = lbm_dec_as_i32(args[1]);
	int len      = lbm_dec_as_i32(args[2]);

	uint32_t reg_data = 0;
	bool ok           = bq_read_reg(addr, reg, &reg_data, len);

	if (ok) {
		return lbm_enc_u32(reg_data);
	} else {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_write_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	uint8_t addr = BQ_ADDR_1;

	int reg       = lbm_dec_as_i32(args[1]);
	uint32_t data = lbm_dec_as_u32(args[2]);
	int len       = lbm_dec_as_i32(args[3]);

	bool ok = bq_set_reg(addr, reg, data, len);

	if (ok) {
		return ENC_SYM_TRUE;
	} else {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}
}

typedef struct {
	lbm_uint slave_id;
	lbm_uint cells_ic1;
	lbm_uint cells_ic2;
} config_syms;

static config_syms syms_cfg = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_cfg.slave_id) {
			lbm_add_symbol_const("slave_id", comp);
		} else if (comp == &syms_cfg.cells_ic1) {
			lbm_add_symbol_const("cells_ic1", comp);
		} else if (comp == &syms_cfg.cells_ic2) {
			lbm_add_symbol_const("cells_ic2", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value bms_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg)) {
			lbm_set_error_reason((char *)lbm_error_str_no_number);
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name      = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t *)&backup.config;

	if (compare_symbol(name, &syms_cfg.slave_id)) {
		res = get_or_set_i(set, &cfg->slave_id, &set_arg);
	} else if (compare_symbol(name, &syms_cfg.cells_ic1)) {
		res = get_or_set_i(set, &cfg->cells_ic1, &set_arg);
	} else if (compare_symbol(name, &syms_cfg.cells_ic2)) {
		res = get_or_set_i(set, &cfg->cells_ic2, &set_arg);
	}

	return res;
}

static lbm_value ext_bms_get_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(false, args, argn);
}

static lbm_value ext_bms_set_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(true, args, argn);
}

static lbm_value ext_bms_store_cfg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

// I2C Overrides

static lbm_value ext_i2c_start(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return ENC_SYM_TRUE;
}

static lbm_value ext_i2c_tx_rx(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		return ENC_SYM_EERROR;
	}

	uint16_t addr  = 0;
	size_t txlen   = 0;
	size_t rxlen   = 0;
	uint8_t *txbuf = 0;
	uint8_t *rxbuf = 0;

	const unsigned int max_len = 20;
	uint8_t to_send[max_len];

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}
	addr = lbm_dec_as_u32(args[0]);

	if (lbm_is_array_r(args[1])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		txbuf                     = (uint8_t *)array->data;
		txlen                     = array->size;
	} else {
		lbm_value curr = args[1];
		while (lbm_is_cons(curr)) {
			lbm_value arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[txlen++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (txlen == max_len) {
				break;
			}

			curr = lbm_cdr(curr);
		}

		if (txlen > 0) {
			txbuf = to_send;
		}
	}

	if (argn >= 3 && lbm_is_array_rw(args[2])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);
		rxbuf                     = (uint8_t *)array->data;
		rxlen                     = array->size;
	}

	return lbm_enc_i(i2c_tx_rx(addr, txbuf, txlen, rxbuf, rxlen));
}

static lbm_value ext_i2c_detect_addr(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t address = lbm_dec_as_u32(args[0]);
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(0, cmd, 50 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mutex);

	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_bms_fw_version(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(6);
}

static lbm_value ext_set_buzzer(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_BUZZER, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

// ============================================================================
// CAN Protocol LispBM Extensions
// ============================================================================

// Track fault state for status messages
static volatile uint8_t m_fault_flags = 0;

// (bms-set-fault-flags flags)
// Set fault flags (bit0 = BQ1 init failed, bit1 = BQ2 init failed)
static lbm_value ext_set_fault_flags(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	m_fault_flags = lbm_dec_as_u32(args[0]) & 0xFF;
	return ENC_SYM_TRUE;
}

// (bms-broadcast-all slave-id cells-list temps-list bq1-ok bq2-ok)
// Broadcast all data to master per protocol (8 cell msgs + 1 temp + 1 status)
static lbm_value ext_broadcast_all(lbm_value *args, lbm_uint argn) {
	if (argn < 3) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_list(args[1]) || !lbm_is_list(args[2])) {
		return ENC_SYM_EERROR;
	}

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	// Extract cell voltages into 32-element array
	// 0 = not populated, 0xFFFF = read error
	uint16_t cells_mv[32] = {0};
	uint8_t num_cells = 0;
	lbm_value curr = args[1];

	while (lbm_is_cons(curr) && num_cells < 32) {
		lbm_value cell = lbm_car(curr);
		if (lbm_is_number(cell)) {
			float v = lbm_dec_as_float(cell);
			if (v < 0) {
				cells_mv[num_cells] = 0xFFFF;  // Error marker
			} else {
				cells_mv[num_cells] = (uint16_t)(v * 1000.0f);  // Convert V to mV
			}
			num_cells++;
		}
		curr = lbm_cdr(curr);
	}

	// Extract temperatures into 4-element array
	// 0x7FFF = not present/invalid
	int16_t temps[4] = {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF};
	uint8_t num_temps = 0;
	curr = args[2];

	while (lbm_is_cons(curr) && num_temps < 4) {
		lbm_value temp = lbm_car(curr);
		if (lbm_is_number(temp)) {
			float t = lbm_dec_as_float(temp);
			if (t < -40.0f || t > 120.0f) {
				temps[num_temps] = 0x7FFF;  // Invalid marker
			} else {
				temps[num_temps] = (int16_t)(t * 10.0f);  // Convert to 0.1°C
			}
			num_temps++;
		}
		curr = lbm_cdr(curr);
	}

	// Get fault flags from optional args or use stored value
	uint8_t faults = m_fault_flags;
	if (argn >= 5) {
		bool bq1_ok = lbm_dec_as_i32(args[3]) != 0;
		bool bq2_ok = lbm_dec_as_i32(args[4]) != 0;
		faults = 0;
		if (!bq1_ok) faults |= 0x01;
		if (!bq2_ok) faults |= 0x02;
	}

	// Send all messages per protocol
	// TX queue is 20 messages, we send 10, so no delays needed
	can_send_all_cells(slave_id, cells_mv);
	can_send_temps(slave_id, temps);
	can_send_status(slave_id, get_bal_bitmap(), faults, (uint8_t)M_CELLS);

	return ENC_SYM_TRUE;
}

// (bms-set-bal-bitmap bitmap)
// Set balancing state from 32-bit bitmap (for master control)
static lbm_value ext_set_bal_bitmap(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t bitmap = lbm_dec_as_u32(args[0]);

	xSemaphoreTake(bq_mutex, portMAX_DELAY);
	bool res = apply_bal_bitmap(bitmap);
	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (bms-get-bal-bitmap)
// Get current balancing state as 32-bit bitmap
static lbm_value ext_get_bal_bitmap(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(get_bal_bitmap());
}

// (bms-stop-balancing)
// Stop all cell balancing
static lbm_value ext_stop_balancing(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(bq_mutex, portMAX_DELAY);
	bool res = stop_all_balancing();
	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (bms-set-bal-bitmap-demo bitmap)
// Set balance mask directly without writing to BQ chips (for demo/testing)
static lbm_value ext_set_bal_bitmap_demo(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t bitmap = lbm_dec_as_u32(args[0]);
	m_bal_state_ic1 = bitmap & 0xFFFF;
	m_bal_state_ic2 = (bitmap >> 16) & 0xFFFF;

	return ENC_SYM_TRUE;
}

// (bms-get-slave-id)
// Get slave ID from configuration (set via VESC Tool -> JFBMS Slave -> Slave ID)
static lbm_value ext_get_slave_id(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_config_t *cfg = (main_config_t *)&backup.config;
	return lbm_enc_i(cfg->slave_id);
}

// ============================================================================
// Direct CAN RX Buffer - bypasses broken event system
// ============================================================================

#define CAN_BUF_SIZE 16

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} can_msg_t;

static can_msg_t can_rx_buffer[CAN_BUF_SIZE];
static volatile int can_rx_write = 0;
static volatile int can_rx_read = 0;
static volatile uint32_t can_rx_overflow = 0;

// Hardware CAN hook - called from comm_can.c for every received message
void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	if (is_ext) return;  // Only handle standard 11-bit IDs

	int next_write = (can_rx_write + 1) % CAN_BUF_SIZE;
	if (next_write == can_rx_read) {
		// Buffer full - overflow
		can_rx_overflow++;
		return;
	}

	can_rx_buffer[can_rx_write].id = id;
	can_rx_buffer[can_rx_write].len = len > 8 ? 8 : len;
	memcpy(can_rx_buffer[can_rx_write].data, data, can_rx_buffer[can_rx_write].len);
	can_rx_write = next_write;
}

// (slave-can-available) - Returns number of messages in buffer
static lbm_value ext_slave_can_available(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	int count = can_rx_write - can_rx_read;
	if (count < 0) count += CAN_BUF_SIZE;
	return lbm_enc_i(count);
}

// (slave-can-read) - Read one message from buffer, returns (id . data) or nil
static lbm_value ext_slave_can_read(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (can_rx_read == can_rx_write) {
		return ENC_SYM_NIL;  // Buffer empty
	}

	can_msg_t *msg = &can_rx_buffer[can_rx_read];
	can_rx_read = (can_rx_read + 1) % CAN_BUF_SIZE;

	// Create array for data
	lbm_value data_arr;
	if (!lbm_heap_allocate_array(&data_arr, msg->len)) {
		return ENC_SYM_NIL;
	}
	lbm_array_header_t *arr = (lbm_array_header_t *)lbm_car(data_arr);
	memcpy(arr->data, msg->data, msg->len);

	// Return (id . data)
	return lbm_cons(lbm_enc_u32(msg->id), data_arr);
}


static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_cfg, 0, sizeof(syms_cfg));

	// Wake up and initialize hardware
	lbm_add_extension("bms-init", ext_bms_init);

	// Put BMS hardware in sleep mode
	lbm_add_extension("bms-sleep", ext_hw_sleep);

	// Get list of cell voltages
	lbm_add_extension("bms-get-vcells", ext_get_vcells);

	// Get list of temperature readings
	lbm_add_extension("bms-get-temps", ext_get_temps);

	// Get output voltage after power switch
	lbm_add_extension("bms-get-vout", ext_get_vout);

	// Get stack voltage
	lbm_add_extension("bms-get-vstack", ext_get_vstack);

	// Set and get balancing state for cell
	lbm_add_extension("bms-set-bal", ext_set_bal);
	lbm_add_extension("bms-get-bal", ext_get_bal);

	// Buzzer control
	lbm_add_extension("bms-set-buzzer", ext_set_buzzer);

	// CAN protocol for master-slave communication (11-bit IDs)
	lbm_add_extension("bms-broadcast-all", ext_broadcast_all);
	lbm_add_extension("bms-set-fault-flags", ext_set_fault_flags);
	lbm_add_extension("bms-set-bal-bitmap", ext_set_bal_bitmap);
	lbm_add_extension("bms-get-bal-bitmap", ext_get_bal_bitmap);
	lbm_add_extension("bms-stop-balancing", ext_stop_balancing);
	lbm_add_extension("bms-set-bal-bitmap-demo", ext_set_bal_bitmap_demo);
	lbm_add_extension("bms-get-slave-id", ext_get_slave_id);

	// Direct CAN buffer extensions (bypass broken event system)
	lbm_add_extension("slave-can-available", ext_slave_can_available);
	lbm_add_extension("slave-can-read", ext_slave_can_read);

	// HW-specific commands
	lbm_add_extension("bms-direct-cmd", ext_direct_cmd);
	lbm_add_extension("bms-subcmd-cmdonly", ext_subcmd_cmdonly);
	lbm_add_extension("bms-read-reg", ext_read_reg);
	lbm_add_extension("bms-write-reg", ext_write_reg);

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);

	// Replace existing I2C-extensions
	lbm_add_extension("i2c-start", ext_i2c_start);
	lbm_add_extension("i2c-tx-rx", ext_i2c_tx_rx);
	lbm_add_extension("i2c-detect-addr", ext_i2c_detect_addr);

	lbm_add_extension("bms-fw-version", ext_bms_fw_version);
}

void hw_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();
	bq_mutex  = xSemaphoreCreateMutex();

	// Disable VESC CAN protocol decoder - we only use our 11-bit protocol
	comm_can_use_vesc_decoder(false);

	gpio_config_t gpconf = {0};

	// Configure BQ communication enable pins (active LOW, only one can be LOW at a time)
	// Enable BQ1 by default, disable BQ2
	gpio_set_level(PIN_BQ1_EN, 0);  // LOW = enabled
	gpio_set_level(PIN_BQ2_EN, 1);  // HIGH = disabled

	gpconf.pin_bit_mask = BIT(PIN_BQ1_EN) | BIT(PIN_BQ2_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// Set levels again after config to ensure they're applied
	gpio_set_level(PIN_BQ1_EN, 0);  // LOW = enabled
	gpio_set_level(PIN_BQ2_EN, 1);  // HIGH = disabled

	// Configure buzzer pin
	gpio_set_level(PIN_BUZZER, 0);
	gpconf.pin_bit_mask = BIT(PIN_BUZZER);
	gpconf.mode         = GPIO_MODE_OUTPUT;
	gpio_config(&gpconf);

	// Initialize I2C
	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	lispif_add_ext_load_callback(load_extensions);
}

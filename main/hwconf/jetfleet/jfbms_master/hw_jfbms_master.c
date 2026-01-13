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

#include "hw_jfbms_master.h"

#include "driver/twai.h"
#include "driver/gpio.h"
#include "heap.h"
#include "lbm_defines.h"
#include "main.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"
#include "comm_can.h"
#include "bms.h"
#include "buffer.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

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
//   - Temperatures: int16 0.1C (0x7FFF = not present/invalid)
// ============================================================================

// CAN ID macros per protocol spec
#define CAN_ID_CELLS(type, slave_id)  (((type) << 7) | (slave_id))
#define CAN_ID_TEMPS(slave_id)        (0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)       (0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)      (0x500 | (slave_id))

// Extract slave_id and msg_type from CAN ID
#define CAN_GET_SLAVE_ID(id)          ((id) & 0x7F)
#define CAN_GET_MSG_TYPE(id)          (((id) >> 7) & 0x0F)

// Variables
static master_bms_data_t m_bms_data;
static SemaphoreHandle_t m_data_mutex;

// ============================================================================
// CAN Protocol TX Functions
// ============================================================================

/**
 * Send balance command to a slave
 * @param slave_id  Slave ID (1-8)
 * @param bal_mask  32-bit balance bitmap (each bit = one cell)
 */
static void can_send_balance_cmd(uint8_t slave_id, uint32_t bal_mask) {
	uint8_t buf[4];

	// Balance mask, little-endian
	buf[0] = (bal_mask >> 0) & 0xFF;
	buf[1] = (bal_mask >> 8) & 0xFF;
	buf[2] = (bal_mask >> 16) & 0xFF;
	buf[3] = (bal_mask >> 24) & 0xFF;

	comm_can_transmit_sid(CAN_ID_BAL_CMD(slave_id), buf, 4);
}

// ============================================================================
// CAN Protocol RX Functions
// ============================================================================

/**
 * Parse cell voltage message (msg_type 0x0-0x7)
 * @param slave_id  Slave ID (1-8)
 * @param msg_type  Message type (0-7, determines which 4 cells)
 * @param data      CAN payload (8 bytes, 4 uint16 little-endian)
 */
static void parse_cell_voltages(uint8_t slave_id, uint8_t msg_type, uint8_t *data) {
	if (slave_id < 1 || slave_id > MAX_SLAVES) return;
	if (msg_type > 7) return;

	uint8_t idx = slave_id - 1;  // Convert to 0-based index
	uint8_t base_cell = msg_type * 4;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	// Unpack 4 cells from little-endian uint16
	for (int i = 0; i < 4; i++) {
		m_bms_data.cell_voltages[idx][base_cell + i] =
			(uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
	}

	// Update last seen timestamp
	m_bms_data.last_seen_ms[idx] = xTaskGetTickCount() * portTICK_PERIOD_MS;
	m_bms_data.active[idx] = true;

	xSemaphoreGive(m_data_mutex);
}

/**
 * Parse temperature message (msg_type 0x8)
 * @param slave_id  Slave ID (1-8)
 * @param data      CAN payload (8 bytes, 4 int16 little-endian in 0.1C)
 */
static void parse_temperatures(uint8_t slave_id, uint8_t *data) {
	if (slave_id < 1 || slave_id > MAX_SLAVES) return;

	uint8_t idx = slave_id - 1;  // Convert to 0-based index

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	// Unpack 4 temps from little-endian int16
	for (int i = 0; i < 4; i++) {
		m_bms_data.temperatures[idx][i] =
			(int16_t)((uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
	}

	// Update last seen timestamp
	m_bms_data.last_seen_ms[idx] = xTaskGetTickCount() * portTICK_PERIOD_MS;
	m_bms_data.active[idx] = true;

	xSemaphoreGive(m_data_mutex);
}

/**
 * Parse status message (msg_type 0x9)
 * @param slave_id  Slave ID (1-8)
 * @param data      CAN payload (5 bytes: 4 bytes balance mask + 1 byte faults)
 */
static void parse_status(uint8_t slave_id, uint8_t *data) {
	if (slave_id < 1 || slave_id > MAX_SLAVES) return;

	uint8_t idx = slave_id - 1;  // Convert to 0-based index

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	// Unpack balance mask (little-endian uint32)
	m_bms_data.balance_mask[idx] =
		(uint32_t)data[0] |
		((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) |
		((uint32_t)data[3] << 24);

	// Unpack fault flags
	m_bms_data.fault_flags[idx] = data[4];

	// Update last seen timestamp
	m_bms_data.last_seen_ms[idx] = xTaskGetTickCount() * portTICK_PERIOD_MS;
	m_bms_data.active[idx] = true;

	xSemaphoreGive(m_data_mutex);
}

/**
 * Check for timed-out slaves and mark them as inactive
 * @param timeout_ms  Timeout in milliseconds
 */
static void check_slave_timeouts(uint32_t timeout_ms) {
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0; i < MAX_SLAVES; i++) {
		if (m_bms_data.active[i]) {
			if ((now - m_bms_data.last_seen_ms[i]) > timeout_ms) {
				m_bms_data.active[i] = false;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);
}

// ============================================================================
// Public API
// ============================================================================

/**
 * Get pointer to master BMS data structure
 * Note: Caller should use mutex for thread-safe access
 */
master_bms_data_t* hw_master_get_data(void) {
	return &m_bms_data;
}

// ============================================================================
// LispBM Extensions
// ============================================================================

// (master-get-cell-voltage slave-id cell-idx)
// Returns cell voltage in V, or nil if not available
static lbm_value ext_master_get_cell_voltage(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);
	uint8_t cell_idx = lbm_dec_as_u32(args[1]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;
	if (cell_idx >= CELLS_PER_SLAVE) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];
	uint16_t mv = m_bms_data.cell_voltages[idx][cell_idx];
	xSemaphoreGive(m_data_mutex);

	if (!active) return ENC_SYM_NIL;
	if (mv == 0x0000) return ENC_SYM_NIL;  // Not populated
	if (mv == 0xFFFF) return ENC_SYM_NIL;  // Read error

	return lbm_enc_float((float)mv / 1000.0f);
}

// (master-get-temp slave-id temp-idx)
// Returns temperature in C, or nil if not available
static lbm_value ext_master_get_temp(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);
	uint8_t temp_idx = lbm_dec_as_u32(args[1]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;
	if (temp_idx >= TEMPS_PER_SLAVE) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];
	int16_t temp = m_bms_data.temperatures[idx][temp_idx];
	xSemaphoreGive(m_data_mutex);

	if (!active) return ENC_SYM_NIL;
	if (temp == 0x7FFF) return ENC_SYM_NIL;  // Invalid marker

	return lbm_enc_float((float)temp / 10.0f);
}

// (master-get-status slave-id)
// Returns list: (balance-mask fault-flags) or nil if not available
static lbm_value ext_master_get_status(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];
	uint32_t bal_mask = m_bms_data.balance_mask[idx];
	uint8_t faults = m_bms_data.fault_flags[idx];
	xSemaphoreGive(m_data_mutex);

	if (!active) return ENC_SYM_NIL;

	lbm_value result = ENC_SYM_NIL;
	result = lbm_cons(lbm_enc_i(faults), result);
	result = lbm_cons(lbm_enc_u32(bal_mask), result);

	return result;
}

// (master-get-all-cells slave-id)
// Returns list of 32 cell voltages in V (0 for not populated)
static lbm_value ext_master_get_all_cells(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];

	lbm_value cells_list = ENC_SYM_NIL;
	if (active) {
		// Build list in reverse order (will be reversed by cons)
		for (int i = CELLS_PER_SLAVE - 1; i >= 0; i--) {
			uint16_t mv = m_bms_data.cell_voltages[idx][i];
			float v = 0.0f;
			if (mv != 0x0000 && mv != 0xFFFF) {
				v = (float)mv / 1000.0f;
			}
			cells_list = lbm_cons(lbm_enc_float(v), cells_list);
		}
	}
	xSemaphoreGive(m_data_mutex);

	return cells_list;
}

// (master-get-all-temps slave-id)
// Returns list of 4 temperatures in C (nil for invalid)
static lbm_value ext_master_get_all_temps(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];

	lbm_value temps_list = ENC_SYM_NIL;
	if (active) {
		// Build list in reverse order
		for (int i = TEMPS_PER_SLAVE - 1; i >= 0; i--) {
			int16_t temp = m_bms_data.temperatures[idx][i];
			if (temp == 0x7FFF) {
				temps_list = lbm_cons(ENC_SYM_NIL, temps_list);
			} else {
				temps_list = lbm_cons(lbm_enc_float((float)temp / 10.0f), temps_list);
			}
		}
	}
	xSemaphoreGive(m_data_mutex);

	return temps_list;
}

// (master-send-balance slave-id mask)
// Send balance command to a slave
static lbm_value ext_master_send_balance(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);
	uint32_t mask = lbm_dec_as_u32(args[1]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	can_send_balance_cmd(slave_id, mask);

	return ENC_SYM_TRUE;
}

// (master-slave-active? slave-id)
// Check if slave is responding (received data recently)
static lbm_value ext_master_slave_active(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	uint8_t idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];
	xSemaphoreGive(m_data_mutex);

	return active ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-get-active-slaves)
// Returns list of active slave IDs
static lbm_value ext_master_get_active_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value result = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	// Build list in reverse order (will be in ascending order after cons)
	for (int i = MAX_SLAVES - 1; i >= 0; i--) {
		if (m_bms_data.active[i]) {
			result = lbm_cons(lbm_enc_i(i + 1), result);  // 1-based slave ID
		}
	}
	xSemaphoreGive(m_data_mutex);

	return result;
}

// (master-check-timeouts timeout-ms)
// Check for timed-out slaves
static lbm_value ext_master_check_timeouts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t timeout_ms = lbm_dec_as_u32(args[0]);
	check_slave_timeouts(timeout_ms);

	return ENC_SYM_TRUE;
}

// (master-get-num-slaves)
// Get configured number of expected slaves from config
static lbm_value ext_master_get_num_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	main_config_t *cfg = (main_config_t *)&backup.config;
	return lbm_enc_i(cfg->num_slaves);
}

// (master-get-timeout-ms)
// Get configured slave timeout from config
static lbm_value ext_master_get_timeout_ms(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	main_config_t *cfg = (main_config_t *)&backup.config;
	return lbm_enc_i(cfg->slave_timeout_ms);
}

// (master-reset-data)
// Reset all stored data (useful for reinitialization)
static lbm_value ext_master_reset_data(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	memset(&m_bms_data, 0, sizeof(m_bms_data));

	// Initialize temperatures to invalid marker
	for (int s = 0; s < MAX_SLAVES; s++) {
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			m_bms_data.temperatures[s][t] = 0x7FFF;
		}
	}
	xSemaphoreGive(m_data_mutex);

	return ENC_SYM_TRUE;
}

// (master-parse-can-msg id data)
// Parse incoming CAN message and store in data structure
// Called from Lisp event handler when event-can-sid is received
static lbm_value ext_master_parse_can_msg(lbm_value *args, lbm_uint argn) {
	if (argn != 2) return ENC_SYM_EERROR;
	if (!lbm_is_number(args[0])) return ENC_SYM_EERROR;
	if (!lbm_is_array_r(args[1])) return ENC_SYM_EERROR;

	uint32_t id = lbm_dec_as_u32(args[0]);
	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
	uint8_t *data = (uint8_t *)array->data;
	uint8_t len = array->size;

	uint8_t slave_id = CAN_GET_SLAVE_ID(id);
	uint8_t msg_type = CAN_GET_MSG_TYPE(id);

	// Validate slave_id range
	if (slave_id < 1 || slave_id > MAX_SLAVES) return ENC_SYM_NIL;

	switch (msg_type) {
		case 0x0:
		case 0x1:
		case 0x2:
		case 0x3:
		case 0x4:
		case 0x5:
		case 0x6:
		case 0x7:
			// Cell voltages (msg_type indicates which 4-cell group)
			if (len >= 8) {
				parse_cell_voltages(slave_id, msg_type, data);
			}
			break;
		case 0x8:
			// Temperatures (0x400 | slave_id)
			if (len >= 8) {
				parse_temperatures(slave_id, data);
			}
			break;
		case 0x9:
			// Status (0x480 | slave_id)
			if (len >= 5) {
				parse_status(slave_id, data);
			}
			break;
		default:
			// Unknown message type
			return ENC_SYM_NIL;
	}

	return ENC_SYM_TRUE;
}

// ============================================================================
// Configuration Parameters
// ============================================================================

typedef struct {
	lbm_uint num_slaves;
	lbm_uint slave_timeout_ms;
} config_syms;

static config_syms syms_cfg = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_cfg.num_slaves) {
			lbm_add_symbol_const("num_slaves", comp);
		} else if (comp == &syms_cfg.slave_timeout_ms) {
			lbm_add_symbol_const("slave_timeout_ms", comp);
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

static lbm_value master_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
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

	if (compare_symbol(name, &syms_cfg.num_slaves)) {
		res = get_or_set_i(set, &cfg->num_slaves, &set_arg);
	} else if (compare_symbol(name, &syms_cfg.slave_timeout_ms)) {
		res = get_or_set_i(set, &cfg->slave_timeout_ms, &set_arg);
	}

	return res;
}

static lbm_value ext_master_get_param(lbm_value *args, lbm_uint argn) {
	return master_get_set_param(false, args, argn);
}

static lbm_value ext_master_set_param(lbm_value *args, lbm_uint argn) {
	return master_get_set_param(true, args, argn);
}

static lbm_value ext_master_store_cfg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

// ============================================================================
// Debug Extensions
// ============================================================================

// (can-debug) - Returns CAN/TWAI driver status for debugging
// Returns list: (state tx-err-cnt rx-err-cnt msgs-to-tx msgs-to-rx tx-failed arb-lost bus-err)
static lbm_value ext_can_debug(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	twai_status_info_t status;
	esp_err_t err = twai_get_status_info(&status);

	if (err != ESP_OK) {
		return ENC_SYM_NIL;
	}

	lbm_value result = ENC_SYM_NIL;

	// Build list in reverse order (bus_error_count first, state last)
	result = lbm_cons(lbm_enc_i32(status.bus_error_count), result);
	result = lbm_cons(lbm_enc_i32(status.arb_lost_count), result);
	result = lbm_cons(lbm_enc_i32(status.tx_failed_count), result);
	result = lbm_cons(lbm_enc_i32(status.msgs_to_rx), result);
	result = lbm_cons(lbm_enc_i32(status.msgs_to_tx), result);
	result = lbm_cons(lbm_enc_i32(status.rx_error_counter), result);
	result = lbm_cons(lbm_enc_i32(status.tx_error_counter), result);
	result = lbm_cons(lbm_enc_i32(status.state), result);

	return result;
}

// (gpio-read pin) - Read GPIO pin level (0 or 1)
static lbm_value ext_gpio_read(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}

	int pin = lbm_dec_as_i32(args[0]);
	if (pin < 0 || pin > 21) {
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(gpio_get_level(pin));
}

// (can-rx-count) - Get total number of CAN messages received by TWAI driver
static lbm_value ext_can_rx_count(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(comm_can_get_rx_total_cnt());
}

// (can-event-enabled?) - Check if CAN SID events are enabled
static lbm_value ext_can_event_enabled(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	extern volatile bool event_can_sid_en;
	return event_can_sid_en ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// ============================================================================
// Direct CAN RX Buffer - bypasses broken event system
// ============================================================================

#define CAN_BUF_SIZE 64

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

// (master-can-available) - Returns number of messages in buffer
static lbm_value ext_master_can_available(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	int count = can_rx_write - can_rx_read;
	if (count < 0) count += CAN_BUF_SIZE;
	return lbm_enc_i(count);
}

// (master-can-read) - Read one message from buffer, returns (id . data) or nil
static lbm_value ext_master_can_read(lbm_value *args, lbm_uint argn) {
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

// (master-can-read-all) - Read all messages and parse them, returns count
static lbm_value ext_master_can_read_all(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	int count = 0;
	while (can_rx_read != can_rx_write) {
		can_msg_t *msg = &can_rx_buffer[can_rx_read];
		can_rx_read = (can_rx_read + 1) % CAN_BUF_SIZE;

		// Parse message directly
		uint8_t slave_id = CAN_GET_SLAVE_ID(msg->id);
		uint8_t msg_type = CAN_GET_MSG_TYPE(msg->id);

		if (slave_id >= 1 && slave_id <= MAX_SLAVES) {
			switch (msg_type) {
				case 0x0: case 0x1: case 0x2: case 0x3:
				case 0x4: case 0x5: case 0x6: case 0x7:
					if (msg->len >= 8) parse_cell_voltages(slave_id, msg_type, msg->data);
					break;
				case 0x8:
					if (msg->len >= 8) parse_temperatures(slave_id, msg->data);
					break;
				case 0x9:
					if (msg->len >= 5) parse_status(slave_id, msg->data);
					break;
			}
		}
		count++;
	}

	return lbm_enc_i(count);
}

// (master-can-overflow) - Get overflow count
static lbm_value ext_master_can_overflow(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(can_rx_overflow);
}

// ============================================================================
// VESC BMS Data Integration
// ============================================================================

// Update VESC BMS values from master data (for VESC Tool display)
static void update_vesc_bms_values(void) {
	volatile bms_values *bms = bms_get_values();

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	// Find first active slave and use its data
	int active_slave = -1;
	for (int i = 0; i < MAX_SLAVES; i++) {
		if (m_bms_data.active[i]) {
			active_slave = i;
			break;
		}
	}

	if (active_slave >= 0) {
		// Count populated cells and calculate totals
		int num_cells = 0;
		float v_tot = 0.0f;
		float v_min = 9999.0f;
		float v_max = 0.0f;

		for (int i = 0; i < CELLS_PER_SLAVE; i++) {
			uint16_t mv = m_bms_data.cell_voltages[active_slave][i];
			if (mv > 0 && mv < 0xFFFF) {
				float v = (float)mv / 1000.0f;
				bms->v_cell[num_cells] = v;
				bms->bal_state[num_cells] = (m_bms_data.balance_mask[active_slave] >> i) & 1;
				v_tot += v;
				if (v < v_min) v_min = v;
				if (v > v_max) v_max = v;
				num_cells++;
			}
		}

		bms->cell_num = num_cells;
		bms->v_tot = v_tot;
		bms->v_cell_min = (num_cells > 0) ? v_min : 0.0f;
		bms->v_cell_max = (num_cells > 0) ? v_max : 0.0f;

		// Temperatures
		int num_temps = 0;
		float t_max = -273.0f;
		for (int i = 0; i < TEMPS_PER_SLAVE; i++) {
			int16_t t_raw = m_bms_data.temperatures[active_slave][i];
			if (t_raw != 0x7FFF) {
				float t = (float)t_raw / 10.0f;
				bms->temps_adc[num_temps] = t;
				if (t > t_max) t_max = t;
				num_temps++;
			}
		}
		bms->temp_adc_num = num_temps;
		bms->temp_max_cell = (num_temps > 0) ? t_max : 0.0f;
		bms->temp_ic = (num_temps > 0) ? bms->temps_adc[0] : 0.0f;  // First temp is BQ internal

		// Balancing state
		bms->is_balancing = (m_bms_data.balance_mask[active_slave] != 0) ? 1 : 0;

		// Update timestamp
		bms->update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

		// Simple SOC estimate based on cell voltage (rough approximation)
		// 3.0V = 0%, 4.2V = 100%
		float avg_v = (num_cells > 0) ? (v_tot / num_cells) : 3.7f;
		bms->soc = (avg_v - 3.0f) / (4.2f - 3.0f) * 100.0f;
		if (bms->soc < 0) bms->soc = 0;
		if (bms->soc > 100) bms->soc = 100;

		bms->soh = 100.0f;  // Assume 100% health
		bms->can_id = active_slave + 1;
	}

	xSemaphoreGive(m_data_mutex);
}

// (master-update-vesc-bms) - Update VESC BMS values for display in VESC Tool
static lbm_value ext_master_update_vesc_bms(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	update_vesc_bms_values();
	return ENC_SYM_TRUE;
}

// ============================================================================
// Extension Loading
// ============================================================================

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_cfg, 0, sizeof(syms_cfg));

	// Data access extensions
	lbm_add_extension("master-get-cell-voltage", ext_master_get_cell_voltage);
	lbm_add_extension("master-get-temp", ext_master_get_temp);
	lbm_add_extension("master-get-status", ext_master_get_status);
	lbm_add_extension("master-get-all-cells", ext_master_get_all_cells);
	lbm_add_extension("master-get-all-temps", ext_master_get_all_temps);

	// Control extensions
	lbm_add_extension("master-send-balance", ext_master_send_balance);

	// Status extensions
	lbm_add_extension("master-slave-active?", ext_master_slave_active);
	lbm_add_extension("master-get-active-slaves", ext_master_get_active_slaves);
	lbm_add_extension("master-check-timeouts", ext_master_check_timeouts);
	lbm_add_extension("master-reset-data", ext_master_reset_data);
	lbm_add_extension("master-parse-can-msg", ext_master_parse_can_msg);

	// Configuration extensions
	lbm_add_extension("master-get-num-slaves", ext_master_get_num_slaves);
	lbm_add_extension("master-get-timeout-ms", ext_master_get_timeout_ms);
	lbm_add_extension("master-get-param", ext_master_get_param);
	lbm_add_extension("master-set-param", ext_master_set_param);
	lbm_add_extension("master-store-cfg", ext_master_store_cfg);

	// Debug extensions
	lbm_add_extension("can-debug", ext_can_debug);
	lbm_add_extension("gpio-read", ext_gpio_read);
	lbm_add_extension("can-rx-count", ext_can_rx_count);
	lbm_add_extension("can-event-enabled?", ext_can_event_enabled);

	// Direct CAN buffer extensions (bypass broken event system)
	lbm_add_extension("master-can-available", ext_master_can_available);
	lbm_add_extension("master-can-read", ext_master_can_read);
	lbm_add_extension("master-can-read-all", ext_master_can_read_all);
	lbm_add_extension("master-can-overflow", ext_master_can_overflow);

	// VESC BMS integration
	lbm_add_extension("master-update-vesc-bms", ext_master_update_vesc_bms);
}

// ============================================================================
// Hardware Initialization
// ============================================================================

void hw_init(void) {
	// Create mutex for thread-safe data access
	m_data_mutex = xSemaphoreCreateMutex();

	// Initialize BMS data structure
	memset(&m_bms_data, 0, sizeof(m_bms_data));

	// Initialize temperatures to invalid marker
	for (int s = 0; s < MAX_SLAVES; s++) {
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			m_bms_data.temperatures[s][t] = 0x7FFF;
		}
	}

	// Disable VESC CAN protocol decoder - we only use our 11-bit protocol
	comm_can_use_vesc_decoder(false);

	// Note: CAN RX is handled via LispBM event system (event-can-sid)
	// See jfbms_master_main.lisp for the event handler

	// Register LispBM extension loader
	lispif_add_ext_load_callback(load_extensions);
}

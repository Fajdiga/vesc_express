/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se
	Copyright 2025 JetFleet

	JFBMS Master: VBMS32 hardware base + CAN master protocol for slave communication.

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

#include "hw_jfbms_master_c6.h"

#include "main.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"
#include "comm_can.h"
#include "bms.h"

#include <math.h>
#include <string.h>

// ============================================================================
// Current Sense (GPIO2 / ADC1_CH2): 1 mΩ shunt, 50× amp, center ~1.65 V
// ============================================================================

#define ISENSE_GAIN    50.0f
#define ISENSE_RSHUNT  0.001f
#define ISENSE_SCALE   (1.0f / (ISENSE_GAIN * ISENSE_RSHUNT))   // 20 A/V

static float m_current_offset = 1.65f;     // Calibrated at startup, default 1.65 V
static float m_current_filtered = 0.0f;    // EMA-filtered current (updated 10 Hz)
static bool  m_current_filter_init = false;
static volatile bool m_calibrate_request = false;
#define ISENSE_EMA_ALPHA  0.60f            // ~500 ms 90% settling at 10 Hz

// ============================================================================
// Charger Voltage (GPIO3 / ADC1_CH3): 300 kΩ : 4.7 kΩ divider → 64.83× scale
// ============================================================================

#define VCHG_DIV_SCALE  ((300.0e3f + 4.7e3f) / 4.7e3f)   // 64.83
#define VCHG_EMA_ALPHA  0.60f

static float m_vchg_filtered = 0.0f;
static bool  m_vchg_filter_init = false;

// ============================================================================
// PCB Temp NTC (GPIO4 / ADC1_CH4): NCP18XH103F03RB, 10 kΩ pull-up to 3.3 V
// Topology: 3.3 V → R_pull (10 k) → ADC node → R_ntc → GND
//   R_ntc = V_adc · R_pull / (3.3 - V_adc)
//   T = 1 / (ln(R/R0)/B + 1/T0) - 273.15  (Steinhart simplified)
// ============================================================================

#define NTC_R_PULL      10000.0f
#define NTC_VREF        3.3f
#define NTC_R25         10000.0f
#define NTC_BETA        3434.0f                       // B25/85 for NCP18XH103F03RB
#define NTC_T0_INV      (1.0f / 298.15f)
#define NTC_EMA_ALPHA   0.60f

static float m_temp_pcb = -300.0f;
static bool  m_temp_pcb_filter_init = false;

// ============================================================================
// Section B: Master CAN Protocol
// ============================================================================

// CAN ID macros (matching slave TX format)
#define CAN_ID_CELLS(type, slave_id)  (((type) << 7) | (slave_id))
#define CAN_ID_TEMPS(slave_id)        (0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)       (0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)      (0x500 | (slave_id))

// CAN RX circular buffer for 11-bit messages
#define CAN_BUF_SIZE 128

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} can_msg_t;

static can_msg_t can_rx_buf[CAN_BUF_SIZE];
static volatile int can_rx_head = 0;
static volatile int can_rx_tail = 0;
static volatile uint32_t can_rx_overflow = 0;

// Master slave data
static master_bms_data_t m_bms_data;
static SemaphoreHandle_t m_data_mutex;


// ============================================================================
// Section B: CAN RX Hook & Message Parsing
// ============================================================================

// Called from comm_can.c ISR context for every received CAN frame
void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	if (is_ext) return;  // Only handle 11-bit standard IDs

	int next_head = (can_rx_head + 1) % CAN_BUF_SIZE;
	if (next_head == can_rx_tail) {
		can_rx_overflow++;
		return;  // Buffer full
	}

	can_rx_buf[can_rx_head].id = id;
	can_rx_buf[can_rx_head].len = (len > 8) ? 8 : len;
	memcpy(can_rx_buf[can_rx_head].data, data, can_rx_buf[can_rx_head].len);
	can_rx_head = next_head;
}

// Parse a single CAN message from a slave
static void parse_slave_message(uint32_t id, uint8_t *data, int len) {
	// Extract slave_id from bits 6-0 (we use low 7 bits for slave_id area)
	// But per protocol: slave_id = id & 0x7F for cell msgs, or id & 0x7F for others
	// Actually the protocol says: CAN ID = (msg_type << 7) | slave_id
	// So: slave_id = id & 0x7F, msg_type = (id >> 7)
	// But slave IDs are 1-8, so only low bits matter

	uint8_t slave_id;
	uint8_t msg_type;

	// Check if this is a temperature message (0x400-0x47F)
	if (id >= 0x400 && id < 0x480) {
		slave_id = id & 0x7F;
		msg_type = 0x08;  // Temps
	}
	// Check if this is a status message (0x480-0x4FF)
	else if (id >= 0x480 && id < 0x500) {
		slave_id = id & 0x7F;
		msg_type = 0x09;  // Status
	}
	// Check if this is a cell voltage message (0x000-0x3FF)
	else if (id < 0x400) {
		slave_id = id & 0x7F;
		msg_type = (id >> 7) & 0x07;  // 0-7
	}
	else {
		return;  // Unknown message type
	}

	// Validate slave_id (1-8)
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return;
	}

	uint8_t idx = slave_id - 1;  // 0-based index

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	m_bms_data.last_seen_ms[idx] = xTaskGetTickCount() * portTICK_PERIOD_MS;
	m_bms_data.active[idx] = true;

	if (msg_type <= 0x07) {
		// Cell voltage message: 4 cells per message, little-endian uint16 mV
		if (len >= 8) {
			uint8_t base_cell = msg_type * 4;
			for (int i = 0; i < 4 && (base_cell + i) < CELLS_PER_SLAVE; i++) {
				m_bms_data.cell_voltages[idx][base_cell + i] =
					(uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
			}
		}
	} else if (msg_type == 0x08) {
		// Temperature message: 4 temps, little-endian int16 0.1 deg C
		if (len >= 8) {
			for (int i = 0; i < TEMPS_PER_SLAVE; i++) {
				m_bms_data.temperatures[idx][i] =
					(int16_t)((uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
			}
		}
	} else if (msg_type == 0x09) {
		// Status message: balance mask (4B) + faults (1B) + cells_ic1 (1B) + cells_ic2 (1B)
		if (len >= 5) {
			m_bms_data.balance_mask[idx] =
				(uint32_t)data[0] |
				((uint32_t)data[1] << 8) |
				((uint32_t)data[2] << 16) |
				((uint32_t)data[3] << 24);
			m_bms_data.fault_flags[idx] = data[4];
			m_bms_data.settled[idx] = (data[4] & 0x04) != 0;
			if (len >= 7) {
				// New format: cells_ic1 + cells_ic2 as separate bytes
				m_bms_data.cells_ic1[idx] = data[5];
				m_bms_data.cells_ic2[idx] = data[6];
			} else if (len >= 6) {
				// Old format: single cell_count byte, assume split at 16
				uint8_t total = data[5];
				m_bms_data.cells_ic1[idx] = (total > 16) ? 16 : total;
				m_bms_data.cells_ic2[idx] = (total > 16) ? (total - 16) : 0;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);
}

// Send balance command with buzzer beep code to slave
static void send_balance_cmd(uint8_t slave_id, uint32_t mask, uint8_t beep_code) {
	uint8_t buf[5];
	buf[0] = (mask >> 0) & 0xFF;
	buf[1] = (mask >> 8) & 0xFF;
	buf[2] = (mask >> 16) & 0xFF;
	buf[3] = (mask >> 24) & 0xFF;
	buf[4] = beep_code;
	comm_can_transmit_sid(CAN_ID_BAL_CMD(slave_id), buf, 5);
}


// ============================================================================
// Configuration symbol table
// ============================================================================

typedef struct {
	lbm_uint cells_ic1;
	lbm_uint cells_ic2;
	lbm_uint temp_num;
	lbm_uint batt_ah;
	lbm_uint max_bal_ch;
	lbm_uint soc_use_ah;
	lbm_uint block_sleep;
	lbm_uint vc_empty;
	lbm_uint vc_full;
	lbm_uint vc_balance_start;
	lbm_uint vc_balance_end;
	lbm_uint vc_charge_start;
	lbm_uint vc_charge_end;
	lbm_uint vc_charge_min;
	lbm_uint vc_balance_min;
	lbm_uint balance_max_current;
	lbm_uint min_current_ah_wh_cnt;
	lbm_uint min_current_sleep;
	lbm_uint v_charge_detect;
	lbm_uint t_charge_max;
	lbm_uint t_charge_max_mos;
	lbm_uint sleep_regular;
	lbm_uint sleep_long;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint psw_t_pchg;
	lbm_uint psw_scd_en;
	lbm_uint psw_scd_tres;
	lbm_uint t_psw_en;
	lbm_uint t_psw_max_mos;
	lbm_uint psw_wait_init;
	// Master-specific
	lbm_uint num_slaves;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.cells_ic1) {
			lbm_add_symbol_const("cells_ic1", comp);
		} else if (comp == &syms_vesc.cells_ic2) {
			lbm_add_symbol_const("cells_ic2", comp);
		} else if (comp == &syms_vesc.temp_num) {
			lbm_add_symbol_const("temp_num", comp);
		} else if (comp == &syms_vesc.batt_ah) {
			lbm_add_symbol_const("batt_ah", comp);
		} else if (comp == &syms_vesc.max_bal_ch) {
			lbm_add_symbol_const("max_bal_ch", comp);
		} else if (comp == &syms_vesc.soc_use_ah) {
			lbm_add_symbol_const("soc_use_ah", comp);
		} else if (comp == &syms_vesc.block_sleep) {
			lbm_add_symbol_const("block_sleep", comp);
		} else if (comp == &syms_vesc.vc_empty) {
			lbm_add_symbol_const("vc_empty", comp);
		} else if (comp == &syms_vesc.vc_full) {
			lbm_add_symbol_const("vc_full", comp);
		} else if (comp == &syms_vesc.vc_balance_start) {
			lbm_add_symbol_const("vc_balance_start", comp);
		} else if (comp == &syms_vesc.vc_balance_end) {
			lbm_add_symbol_const("vc_balance_end", comp);
		} else if (comp == &syms_vesc.vc_charge_start) {
			lbm_add_symbol_const("vc_charge_start", comp);
		} else if (comp == &syms_vesc.vc_charge_end) {
			lbm_add_symbol_const("vc_charge_end", comp);
		} else if (comp == &syms_vesc.vc_charge_min) {
			lbm_add_symbol_const("vc_charge_min", comp);
		} else if (comp == &syms_vesc.vc_balance_min) {
			lbm_add_symbol_const("vc_balance_min", comp);
		} else if (comp == &syms_vesc.balance_max_current) {
			lbm_add_symbol_const("balance_max_current", comp);
		} else if (comp == &syms_vesc.min_current_ah_wh_cnt) {
			lbm_add_symbol_const("min_current_ah_wh_cnt", comp);
		} else if (comp == &syms_vesc.min_current_sleep) {
			lbm_add_symbol_const("min_current_sleep", comp);
		} else if (comp == &syms_vesc.v_charge_detect) {
			lbm_add_symbol_const("v_charge_detect", comp);
		} else if (comp == &syms_vesc.t_charge_max) {
			lbm_add_symbol_const("t_charge_max", comp);
		} else if (comp == &syms_vesc.t_charge_max_mos) {
			lbm_add_symbol_const("t_charge_max_mos", comp);
		} else if (comp == &syms_vesc.sleep_regular) {
			lbm_add_symbol_const("sleep_regular", comp);
		} else if (comp == &syms_vesc.sleep_long) {
			lbm_add_symbol_const("sleep_long", comp);
		} else if (comp == &syms_vesc.min_charge_current) {
			lbm_add_symbol_const("min_charge_current", comp);
		} else if (comp == &syms_vesc.max_charge_current) {
			lbm_add_symbol_const("max_charge_current", comp);
		} else if (comp == &syms_vesc.soc_filter_const) {
			lbm_add_symbol_const("soc_filter_const", comp);
		} else if (comp == &syms_vesc.t_bal_max_cell) {
			lbm_add_symbol_const("t_bal_max_cell", comp);
		} else if (comp == &syms_vesc.t_bal_max_ic) {
			lbm_add_symbol_const("t_bal_max_ic", comp);
		} else if (comp == &syms_vesc.t_charge_min) {
			lbm_add_symbol_const("t_charge_min", comp);
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			lbm_add_symbol_const("t_charge_mon_en", comp);
		} else if (comp == &syms_vesc.psw_t_pchg) {
			lbm_add_symbol_const("psw_t_pchg", comp);
		} else if (comp == &syms_vesc.psw_scd_en) {
			lbm_add_symbol_const("psw_scd_en", comp);
		} else if (comp == &syms_vesc.psw_scd_tres) {
			lbm_add_symbol_const("psw_scd_tres", comp);
		} else if (comp == &syms_vesc.t_psw_en) {
			lbm_add_symbol_const("t_psw_en", comp);
		} else if (comp == &syms_vesc.t_psw_max_mos) {
			lbm_add_symbol_const("t_psw_max_mos", comp);
		} else if (comp == &syms_vesc.psw_wait_init) {
			lbm_add_symbol_const("psw_wait_init", comp);
		} else if (comp == &syms_vesc.num_slaves) {
			lbm_add_symbol_const("num_slaves", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_float(bool set, float *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_float(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_float(*val);
	}
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_bool(bool set, bool *val, lbm_value *lbm_val) {
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

	if (compare_symbol(name, &syms_vesc.cells_ic1)) {
		res = get_or_set_i(set, &cfg->cells_ic1, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cells_ic2)) {
		res = get_or_set_i(set, &cfg->cells_ic2, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_num)) {
		res = get_or_set_i(set, &cfg->temp_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.batt_ah)) {
		res = get_or_set_float(set, &cfg->batt_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_bal_ch)) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_use_ah)) {
		res = get_or_set_bool(set, &cfg->soc_use_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.block_sleep)) {
		res = get_or_set_bool(set, &cfg->block_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_empty)) {
		res = get_or_set_float(set, &cfg->vc_empty, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_full)) {
		res = get_or_set_float(set, &cfg->vc_full, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_start)) {
		res = get_or_set_float(set, &cfg->vc_balance_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_end)) {
		res = get_or_set_float(set, &cfg->vc_balance_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_start)) {
		res = get_or_set_float(set, &cfg->vc_charge_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_end)) {
		res = get_or_set_float(set, &cfg->vc_charge_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_min)) {
		res = get_or_set_float(set, &cfg->vc_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_min)) {
		res = get_or_set_float(set, &cfg->vc_balance_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.balance_max_current)) {
		res = get_or_set_float(set, &cfg->balance_max_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_ah_wh_cnt)) {
		res = get_or_set_float(set, &cfg->min_current_ah_wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_sleep)) {
		res = get_or_set_float(set, &cfg->min_current_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_charge_detect)) {
		res = get_or_set_float(set, &cfg->v_charge_detect, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max)) {
		res = get_or_set_float(set, &cfg->t_charge_max, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max_mos)) {
		res = get_or_set_float(set, &cfg->t_charge_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep_regular)) {
		res = get_or_set_float(set, &cfg->sleep_regular, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep_long)) {
		res = get_or_set_float(set, &cfg->sleep_long, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_charge_current)) {
		res = get_or_set_float(set, &cfg->min_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_charge_current)) {
		res = get_or_set_float(set, &cfg->max_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_filter_const)) {
		res = get_or_set_float(set, &cfg->soc_filter_const, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_cell)) {
		res = get_or_set_float(set, &cfg->t_bal_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_ic)) {
		res = get_or_set_float(set, &cfg->t_bal_max_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_min)) {
		res = get_or_set_float(set, &cfg->t_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_mon_en)) {
		res = get_or_set_bool(set, &cfg->t_charge_mon_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_t_pchg)) {
		res = get_or_set_float(set, &cfg->psw_t_pchg, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_en)) {
		res = get_or_set_bool(set, &cfg->psw_scd_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_tres)) {
		res = get_or_set_i(set, &cfg->psw_scd_tres, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_en)) {
		res = get_or_set_bool(set, &cfg->t_psw_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_max_mos)) {
		res = get_or_set_float(set, &cfg->t_psw_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_wait_init)) {
		res = get_or_set_bool(set, &cfg->psw_wait_init, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.num_slaves)) {
		res = get_or_set_i(set, &cfg->num_slaves, &set_arg);
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

static lbm_value ext_bms_fw_version(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(6);
}


// ============================================================================
// Section D: Master Lisp Extensions (CAN slave communication)
// ============================================================================

// (master-can-read-all) - Read and parse all buffered CAN messages, return count
static lbm_value ext_master_can_read_all(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	int count = 0;
	while (can_rx_tail != can_rx_head) {
		can_msg_t *msg = &can_rx_buf[can_rx_tail];
		parse_slave_message(msg->id, msg->data, msg->len);
		can_rx_tail = (can_rx_tail + 1) % CAN_BUF_SIZE;
		count++;
	}

	return lbm_enc_i(count);
}

// (master-can-available) - Get count of buffered messages
static lbm_value ext_master_can_available(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	int available = can_rx_head - can_rx_tail;
	if (available < 0) available += CAN_BUF_SIZE;

	return lbm_enc_i(available);
}

// (master-can-overflow) - Get overflow count
static lbm_value ext_master_can_overflow(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(can_rx_overflow);
}

// (master-get-slave-cells slave-id) - Get list of cell voltages in V (only non-zero up to cell_count)
static lbm_value ext_master_get_slave_cells(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;
	lbm_value vc_list = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	int num_cells = m_bms_data.cells_ic1[idx] + m_bms_data.cells_ic2[idx];
	if (num_cells == 0) num_cells = CELLS_PER_SLAVE;
	if (num_cells > CELLS_PER_SLAVE) num_cells = CELLS_PER_SLAVE;

	for (int i = num_cells - 1; i >= 0; i--) {
		uint16_t mv = m_bms_data.cell_voltages[idx][i];
		if (mv != 0 && mv != 0xFFFF) {
			vc_list = lbm_cons(lbm_enc_float((float)mv / 1000.0f), vc_list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return vc_list;
}

// (master-get-slave-temps slave-id) - Get list of 4 temperatures in deg C
static lbm_value ext_master_get_slave_temps(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;
	lbm_value ts_list = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = TEMPS_PER_SLAVE - 1; i >= 0; i--) {
		int16_t raw = m_bms_data.temperatures[idx][i];
		if (raw != 0x7FFF) {
			ts_list = lbm_cons(lbm_enc_float((float)raw / 10.0f), ts_list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return ts_list;
}

// (master-get-slave-status slave-id) - Get (balance-mask faults cells-ic1 cells-ic2)
static lbm_value ext_master_get_slave_status(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	lbm_value res = ENC_SYM_NIL;
	res = lbm_cons(lbm_enc_i(m_bms_data.cells_ic2[idx]), res);
	res = lbm_cons(lbm_enc_i(m_bms_data.cells_ic1[idx]), res);
	res = lbm_cons(lbm_enc_i(m_bms_data.fault_flags[idx]), res);
	res = lbm_cons(lbm_enc_u32(m_bms_data.balance_mask[idx]), res);

	xSemaphoreGive(m_data_mutex);

	return res;
}

// (master-slave-active? slave-id) - Check if slave is responding
static lbm_value ext_master_slave_active(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = m_bms_data.active[idx];
	xSemaphoreGive(m_data_mutex);

	return active ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-get-slave-settled? slave-id) - Check if slave voltages are settled (balance off >= 2s)
static lbm_value ext_master_get_slave_settled(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool settled = m_bms_data.settled[idx];
	xSemaphoreGive(m_data_mutex);

	return settled ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-get-active-slaves) - Get list of active slave IDs
static lbm_value ext_master_get_active_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value list = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = MAX_SLAVES - 1; i >= 0; i--) {
		if (m_bms_data.active[i]) {
			list = lbm_cons(lbm_enc_i(i + 1), list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return list;
}

// (master-send-balance slave-id ic1-mask ic2-mask beep-code)
// Takes IC1 and IC2 masks separately to avoid LispBM 28-bit integer overflow
// when combining into a 32-bit balance mask (IC1 bits 0-15, IC2 bits 16-31)
static lbm_value ext_master_send_balance(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	int slave_id = lbm_dec_as_i32(args[0]);
	uint32_t ic1_mask = lbm_dec_as_u32(args[1]) & 0xFFFF;
	uint32_t ic2_mask = lbm_dec_as_u32(args[2]) & 0xFFFF;
	uint8_t beep_code = (uint8_t)lbm_dec_as_u32(args[3]);

	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	uint32_t mask = ic1_mask | (ic2_mask << 16);
	send_balance_cmd(slave_id, mask, beep_code);
	return ENC_SYM_TRUE;
}

// (master-check-timeouts timeout-ms) - Mark timed-out slaves as inactive
static lbm_value ext_master_check_timeouts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t timeout = lbm_dec_as_u32(args[0]);
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0; i < MAX_SLAVES; i++) {
		if (m_bms_data.active[i]) {
			if ((now - m_bms_data.last_seen_ms[i]) > timeout) {
				m_bms_data.active[i] = false;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);

	return ENC_SYM_TRUE;
}

// (master-reset-slaves) - Clear all stored slave data
static lbm_value ext_master_reset_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	memset(&m_bms_data, 0, sizeof(m_bms_data));

	// Initialize temperatures to invalid
	for (int s = 0; s < MAX_SLAVES; s++) {
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			m_bms_data.temperatures[s][t] = 0x7FFF;
		}
	}

	xSemaphoreGive(m_data_mutex);

	// Also reset CAN buffer
	can_rx_head = 0;
	can_rx_tail = 0;
	can_rx_overflow = 0;

	return ENC_SYM_TRUE;
}

// (master-get-cell-count slave-id) - Get total cell count from slave (ic1 + ic2)
static lbm_value ext_master_get_cell_count(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = m_bms_data.cells_ic1[idx] + m_bms_data.cells_ic2[idx];
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

// (master-get-cells-ic1 slave-id) - Get cells on BQ1 from slave
static lbm_value ext_master_get_cells_ic1(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = m_bms_data.cells_ic1[idx];
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

// (master-get-cells-ic2 slave-id) - Get cells on BQ2 from slave
static lbm_value ext_master_get_cells_ic2(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = m_bms_data.cells_ic2[idx];
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

// ============================================================================
// VESC BMS Display Integration
// ============================================================================

// (master-update-vesc-bms) - Combine slave cells into VESC BMS display
static lbm_value ext_master_update_vesc_bms(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	volatile bms_values *bms = bms_get_values();

	int total_cells = 0;
	float v_tot = 0.0f;
	float v_min = 9999.0f;
	float v_max = 0.0f;
	float t_ic_max = -300.0f;
	float t_cell_min = 9999.0f;
	float t_cell_max = -300.0f;

	// Add slave cells
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int s = 0; s < MAX_SLAVES && total_cells < BMS_MAX_CELLS; s++) {
		if (!m_bms_data.active[s]) continue;

		int num_cells = m_bms_data.cells_ic1[s] + m_bms_data.cells_ic2[s];
		if (num_cells == 0) continue;
		if (num_cells > CELLS_PER_SLAVE) num_cells = CELLS_PER_SLAVE;

		int ic1_cnt = m_bms_data.cells_ic1[s];
		for (int c = 0; c < num_cells && total_cells < BMS_MAX_CELLS; c++) {
			uint16_t mv = m_bms_data.cell_voltages[s][c];
			if (mv != 0 && mv != 0xFFFF) {
				float v = (float)mv / 1000.0f;
				bms->v_cell[total_cells] = v;
				// Balance mask is ic1[0:15] | ic2[16:31], map cell index to correct bit
				int bit = (c < ic1_cnt) ? c : (16 + c - ic1_cnt);
				bms->bal_state[total_cells] = (m_bms_data.balance_mask[s] >> bit) & 1;
				v_tot += v;
				if (v < v_min) v_min = v;
				if (v > v_max) v_max = v;
				total_cells++;
			}
		}

		// Aggregate slave temperatures for VESC 6.06 convention
		// Temp order per slave: [0]=BQ1 IC, [1]=BQ1 TS1 (cell), [2]=BQ2 IC, [3]=BQ2 TS1 (cell)
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			int16_t raw = m_bms_data.temperatures[s][t];
			if (raw == 0x7FFF) continue;
			float temp_c = (float)raw / 10.0f;
			if (temp_c > 900.0f) continue;
			if (t == 0 || t == 2) {
				if (temp_c > t_ic_max) t_ic_max = temp_c;
			} else {
				if (temp_c < t_cell_min) t_cell_min = temp_c;
				if (temp_c > t_cell_max) t_cell_max = temp_c;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);

	// Clear remaining cell slots
	for (int i = total_cells; i < BMS_MAX_CELLS; i++) {
		bms->v_cell[i] = 0.0f;
		bms->bal_state[i] = 0;
	}

	// Update BMS values
	bms->cell_num = total_cells;
	bms->v_tot = v_tot;
	{
		float v = adc_get_voltage(HW_ADC_CH3);
		if (v >= 0.0f) {
			float raw_vchg = v * VCHG_DIV_SCALE;
			if (!m_vchg_filter_init) {
				m_vchg_filtered = raw_vchg;
				m_vchg_filter_init = true;
			} else {
				m_vchg_filtered = VCHG_EMA_ALPHA * m_vchg_filtered
					+ (1.0f - VCHG_EMA_ALPHA) * raw_vchg;
			}
		}
		bms->v_charge = m_vchg_filtered;
	}
	// Read pack current — single call drains the entire DMA ring buffer (~500 samples
	// at 5 kHz / 10 Hz), all pre-filtered by the hardware IIR (coeff 64). Apply EMA on top.
	{
		float v = adc_get_voltage(HW_ADC_CH2);
		if (v >= 0.0f) {
			if (m_calibrate_request) {
				m_current_offset      = v;
				m_current_filtered    = 0.0f;
				m_current_filter_init = false;
				m_calibrate_request   = false;
				commands_printf_lisp("Current calibrated: offset=%.4f V (range +-%.1f A)",
					m_current_offset, 1.65f * ISENSE_SCALE);
			}
			float raw_a = (v - m_current_offset) * ISENSE_SCALE;
			if (!m_current_filter_init) {
				m_current_filtered    = raw_a;
				m_current_filter_init = true;
			} else {
				m_current_filtered = ISENSE_EMA_ALPHA * m_current_filtered
				                   + (1.0f - ISENSE_EMA_ALPHA) * raw_a;
			}
			bms->i_in    = m_current_filtered;
			bms->i_in_ic = m_current_filtered;
		}
	}

	{
		float v = adc_get_voltage(HW_ADC_CH4);
		if (v > 0.01f && v < (NTC_VREF - 0.01f)) {
			float r_ntc = (v * NTC_R_PULL) / (NTC_VREF - v);
			float temp_c = (1.0f / ((logf(r_ntc / NTC_R25) / NTC_BETA) + NTC_T0_INV)) - 273.15f;
			if (!m_temp_pcb_filter_init) {
				m_temp_pcb = temp_c;
				m_temp_pcb_filter_init = true;
			} else {
				m_temp_pcb = NTC_EMA_ALPHA * m_temp_pcb
					+ (1.0f - NTC_EMA_ALPHA) * temp_c;
			}
		}
	}

	bms->v_cell_min = (total_cells > 0) ? v_min : 0.0f;
	bms->v_cell_max = (total_cells > 0) ? v_max : 0.0f;
	// VESC 6.06 temperature sensor convention (indices 0-4)
	bms->temps_adc[0] = t_ic_max;                                       // Balance IC
	bms->temps_adc[1] = (t_cell_min < 9000.0f) ? t_cell_min : -300.0f;  // Cell Min
	bms->temps_adc[2] = (t_cell_max > -299.0f) ? t_cell_max : -300.0f;  // Cell Max
	bms->temps_adc[3] = m_temp_pcb;                                      // Mosfet / PCB NTC
	bms->temps_adc[4] = -300.0f;                                         // Ambient N/A
	int temps_count = 5;
	for (int s = 0; s < MAX_SLAVES && temps_count < BMS_MAX_TEMPS; s++) {
		for (int t = 0; t < TEMPS_PER_SLAVE && temps_count < BMS_MAX_TEMPS; t++) {
			int16_t raw = m_bms_data.temperatures[s][t];
			if (raw == 0x7FFF) continue;
			float temp_c = (float)raw / 10.0f;
			if (temp_c > 900.0f) continue;
			if (t == 1 || t == 3) {
				bms->temps_adc[temps_count] = temp_c;
				temps_count++;
			}
		}
	}

	bms->temp_adc_num = temps_count;

	bms->temp_max_cell = (t_cell_max > -299.0f) ? t_cell_max : 0.0f;
	bms->data_version = 1;

	// SOC estimate based on average cell voltage
	float avg_v = (total_cells > 0) ? (v_tot / total_cells) : 3.7f;
	bms->soc = (avg_v - 3.0f) / (4.2f - 3.0f);
	if (bms->soc < 0.0f) bms->soc = 0.0f;
	if (bms->soc > 1.0f) bms->soc = 1.0f;
	bms->soh = 1.0f;

	// Update timestamp
	bms->update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

	return ENC_SYM_TRUE;
}

// ============================================================================
// Current Sense Extensions
// ============================================================================

// (master-calibrate-current) — request zero-current calibration.
// Sets a flag consumed by master-update-vesc-bms on its next ADC read (within ~100 ms).
// Returns true immediately; calibration confirmation is printed by the update loop.
static lbm_value ext_master_calibrate_current(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	m_calibrate_request = true;
	commands_printf_lisp("Current calibration requested — will apply on next ADC update");
	return ENC_SYM_TRUE;
}

// (master-get-current) — returns EMA-filtered current (A); updated by master-update-vesc-bms
static lbm_value ext_master_get_current(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(m_current_filtered);
}

// (master-get-vchg) — returns EMA-filtered charger voltage (V); updated by master-update-vesc-bms
static lbm_value ext_master_get_vchg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(m_vchg_filtered);
}

// (master-get-temp-pcb) — returns EMA-filtered PCB NTC temp (°C); updated by master-update-vesc-bms
static lbm_value ext_master_get_temp_pcb(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(m_temp_pcb);
}

// ============================================================================
// Debug Extensions
// ============================================================================

// (can-debug) - TWAI driver status
static lbm_value ext_can_debug(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	commands_printf_lisp("CAN RX buf: head=%d tail=%d overflow=%lu",
		can_rx_head, can_rx_tail, (unsigned long)can_rx_overflow);
	commands_printf_lisp("CAN core: rx_total=%lu rx_recovery=%d ring_ovf=%lu q_missed=%lu q_overrun=%lu tx_fail=%lu tx_timeout=%lu",
		(unsigned long)comm_can_get_rx_total_cnt(),
		comm_can_get_rx_recovery_cnt(),
		(unsigned long)comm_can_get_rx_ring_overflow_cnt(),
		(unsigned long)comm_can_get_rx_queue_missed_cnt(),
		(unsigned long)comm_can_get_rx_queue_overrun_cnt(),
		(unsigned long)comm_can_get_tx_fail_cnt(),
		(unsigned long)comm_can_get_tx_timeout_cnt());

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	for (int i = 0; i < MAX_SLAVES; i++) {
		if (m_bms_data.active[i]) {
			commands_printf_lisp("Slave %d: active, ic1=%d, ic2=%d, faults=0x%02X, bal=0x%08lX",
				i + 1, m_bms_data.cells_ic1[i], m_bms_data.cells_ic2[i],
				m_bms_data.fault_flags[i], (unsigned long)m_bms_data.balance_mask[i]);
		}
	}
	xSemaphoreGive(m_data_mutex);

	return ENC_SYM_TRUE;
}

// ============================================================================
// Section E: Extension Registration & hw_init
// ============================================================================

static void load_extensions(bool main_found) {
	(void)main_found;

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);

	lbm_add_extension("bms-fw-version", ext_bms_fw_version);

	// === Master CAN Protocol Extensions ===

	// CAN buffer access
	lbm_add_extension("master-can-read-all", ext_master_can_read_all);
	lbm_add_extension("master-can-available", ext_master_can_available);
	lbm_add_extension("master-can-overflow", ext_master_can_overflow);

	// Slave data access
	lbm_add_extension("master-get-slave-cells", ext_master_get_slave_cells);
	lbm_add_extension("master-get-slave-temps", ext_master_get_slave_temps);
	lbm_add_extension("master-get-slave-status", ext_master_get_slave_status);
	lbm_add_extension("master-slave-active?", ext_master_slave_active);
	lbm_add_extension("master-get-slave-settled?", ext_master_get_slave_settled);
	lbm_add_extension("master-get-active-slaves", ext_master_get_active_slaves);
	lbm_add_extension("master-get-cell-count", ext_master_get_cell_count);
	lbm_add_extension("master-get-cells-ic1", ext_master_get_cells_ic1);
	lbm_add_extension("master-get-cells-ic2", ext_master_get_cells_ic2);

	// Slave control
	lbm_add_extension("master-send-balance", ext_master_send_balance);
	lbm_add_extension("master-check-timeouts", ext_master_check_timeouts);
	lbm_add_extension("master-reset-slaves", ext_master_reset_slaves);

	// VESC BMS display
	lbm_add_extension("master-update-vesc-bms", ext_master_update_vesc_bms);

	// Current sense
	lbm_add_extension("master-calibrate-current", ext_master_calibrate_current);
	lbm_add_extension("master-get-current", ext_master_get_current);
	lbm_add_extension("master-get-vchg", ext_master_get_vchg);
	lbm_add_extension("master-get-temp-pcb", ext_master_get_temp_pcb);

	// Debug
	lbm_add_extension("can-debug", ext_can_debug);
}

void hw_init(void) {
	m_data_mutex = xSemaphoreCreateMutex();

	// Initialize master slave data
	memset(&m_bms_data, 0, sizeof(m_bms_data));
	for (int s = 0; s < MAX_SLAVES; s++) {
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			m_bms_data.temperatures[s][t] = 0x7FFF;
		}
	}

	// GPIO setup
	gpio_config_t gpconf = {0};

	// Push-pull outputs. CHG/PCHG are active high; COM_EN is active low.
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_COM_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);

	gpconf.pin_bit_mask = BIT(PIN_CHG_EN) | BIT(PIN_COM_EN) | BIT(PIN_PCHG_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_COM_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);

	// Open-drain output: SHUTDOWN on GPIO19 (active low, default hi-Z = off)
	// GPIO8 (buzzer) is driven by the PWM peripheral — not configured here
	gpio_set_level(PIN_SHUTDOWN, 1);

	gpconf.pin_bit_mask = BIT(PIN_SHUTDOWN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_OUTPUT_OD;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	gpio_set_level(PIN_SHUTDOWN, 1);

	// GPIO2/3/4 are ADC inputs; do not configure them as digital I/O.

	// Calibrate current sense zero offset.
	// Wait 500 ms so the DMA ring buffer fills before the first read.
	vTaskDelay(pdMS_TO_TICKS(500));
	{
		float v = adc_get_voltage(HW_ADC_CH2);
		if (v >= 0.0f) m_current_offset = v;
	}

	lispif_add_ext_load_callback(load_extensions);
}

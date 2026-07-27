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

#include "hw_jfbms_master.h"
#include "jfbms_master_fast_adc.h"
#include "jfbms_master_confparser.h"

#include "main.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "terminal.h"
#include "utils.h"
#include "comm_can.h"
#include "bms.h"
#include "soc/soc_caps.h"

#include <math.h>
#include <string.h>
#include <sys/time.h>

#ifndef JFBMS_USE_DEDICATED_SLAVE_TWAI
#define JFBMS_USE_DEDICATED_SLAVE_TWAI 0
#endif

// ============================================================================
// Current Sense (GPIO2 / ADC1_CH2): nominal 1 mΩ shunt, INA181A3 100×
// amplifier, center ~1.65 V. Bench calibration on the assembled master gives
// 55 mV at 1.1 A, so apply the measured 2× correction to the nominal 10 A/V.
// ============================================================================

#define ISENSE_GAIN    100.0f
#define ISENSE_RSHUNT  0.001f
#define ISENSE_CALIBRATION 2.0f
#define ISENSE_SCALE   (ISENSE_CALIBRATION / (ISENSE_GAIN * ISENSE_RSHUNT)) // 20 A/V
#define ISENSE_ZERO_DEADBAND_A 0.10f
#define ISENSE_DEFAULT_OFFSET_V 1.65f
#define ISENSE_OFFSET_MIN_V 1.50f
#define ISENSE_OFFSET_MAX_V 1.80f
#define ISENSE_POWER_SETTLE_MS 150U
#define ISENSE_NVS_NAMESPACE "jfbms"
#define ISENSE_NVS_KEY "i_zero"
#define ISENSE_NVS_MAGIC 0x4A46495AU
#define ISENSE_NVS_VERSION 1U
#define JFBMS_BOOT_DIAG_MAGIC 0x4A464244U

RTC_NOINIT_ATTR static uint32_t m_boot_diag_magic;
RTC_NOINIT_ATTR static uint32_t m_boot_count;
RTC_NOINIT_ATTR static uint32_t m_previous_reset_reason;

static float m_current_offset = ISENSE_DEFAULT_OFFSET_V;
static float m_current_filtered = 0.0f;    // EMA-filtered current (updated 10 Hz)
static bool  m_current_filter_init = false;
static bool  m_current_valid = false;
static bool  m_current_offset_calibrated = false;
static volatile bool m_calibration_in_progress = false;
#define ISENSE_EMA_ALPHA  0.85f            // Calm BMS current display/counters at 10 Hz

static float isense_read_voltage(void) {
	// The continuous ADC task already averages every conversion frame.
	return adc_get_voltage(HW_ADC_CH2);
}

static bool isense_offset_valid(float offset_v) {
	return isfinite(offset_v) && offset_v >= ISENSE_OFFSET_MIN_V &&
			offset_v <= ISENSE_OFFSET_MAX_V;
}

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t reserved;
	float offset_v;
} isense_zero_record_t;

static bool isense_load_stored_offset(float *offset_v) {
	if (!offset_v) return false;

	nvs_handle_t handle;
	if (nvs_open(ISENSE_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
		return false;
	}

	isense_zero_record_t record = {0};
	size_t size = sizeof(record);
	esp_err_t result = nvs_get_blob(handle, ISENSE_NVS_KEY, &record, &size);
	nvs_close(handle);
	if (result != ESP_OK || size != sizeof(record) ||
			record.magic != ISENSE_NVS_MAGIC ||
			record.version != ISENSE_NVS_VERSION ||
			!isense_offset_valid(record.offset_v)) {
		return false;
	}

	*offset_v = record.offset_v;
	return true;
}

static bool isense_store_offset(float offset_v) {
	if (!isense_offset_valid(offset_v)) return false;

	isense_zero_record_t record = {
		.magic = ISENSE_NVS_MAGIC,
		.version = ISENSE_NVS_VERSION,
		.reserved = 0,
		.offset_v = offset_v,
	};
	nvs_handle_t handle;
	if (nvs_open(ISENSE_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
		return false;
	}

	esp_err_t result = nvs_set_blob(handle, ISENSE_NVS_KEY, &record, sizeof(record));
	if (result == ESP_OK) result = nvs_commit(handle);
	nvs_close(handle);
	return result == ESP_OK;
}

static bool isense_apply_offset(float offset_v) {
	if (!isense_offset_valid(offset_v)) return false;

	// Filtering is independent of the hardware threshold monitor. Keep using
	// the stored/current zero even if protection cannot be armed; CHG_EN still
	// remains fail-closed through the error return below.
	m_current_offset      = offset_v;
	m_current_filtered    = 0.0f;
	m_current_filter_init = false;

	if (!jfbms_fast_adc_set_current_offset(offset_v)) {
		gpio_set_level(PIN_CHG_EN, 0);
		return false;
	}
	return true;
}

// ============================================================================
// Charger Voltage (GPIO3 / ADC1_CH3): 300 kΩ : 4.7 kΩ divider → 64.83× scale
// ============================================================================

#define VCHG_DIV_SCALE  ((300.0e3f + 4.7e3f) / 4.7e3f)   // 64.83
#define VCHG_EMA_ALPHA  0.60f

static float m_vchg_filtered = 0.0f;
static bool  m_vchg_filter_init = false;
static bool  m_vchg_valid = false;

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
static bool  m_temp_pcb_valid = false;

// Rebuilding the continuous ADC monitor stops the ADC briefly. Keep track of
// the settings that actually affect that monitor so a normal configuration
// save (for example a cell-voltage limit) does not interrupt the controller.
static bool m_fast_oc_config_valid = false;
static bool m_fast_oc_config_en;
static float m_fast_oc_config_trip_a;
static int m_applied_slave_count = -1;

static int configured_slave_count(void);
static bool stop_balancing_for_config_change(int slave_count);
static void reset_pack_state_for_config_change(void);

static void remember_fast_oc_config(const main_config_t *cfg) {
	m_fast_oc_config_en = cfg->fast_charge_oc_en;
	m_fast_oc_config_trip_a = cfg->fast_charge_oc_a;
	m_fast_oc_config_valid = true;
}

static bool bms_temp_valid(float temp_c) {
	return temp_c >= -40.0f && temp_c <= 120.0f;
}

// Keep ADC1 ownership and the fast CHG_EN monitor private to this hardware
// profile. The shared adc.c wrapper delegates to hw_adc_get_voltage().
#include "jfbms_master_fast_adc.c"

// Board-owned compatibility and safety hooks. These intentionally live
// outside the VESC Tool-generated parser so regeneration cannot erase them.
bool jfbms_master_migrate_legacy_config(uint32_t signature,
		main_config_t *conf) {
	if (!conf || (signature != JFBMS_MASTER_CONFIG_SIGNATURE_LEGACY &&
			signature != JFBMS_MASTER_CONFIG_SIGNATURE_WITH_CHARGE_TIMERS)) {
		return false;
	}

	if (signature == JFBMS_MASTER_CONFIG_SIGNATURE_LEGACY) {
		conf->fast_charge_oc_en = CONF_FAST_CHARGE_OC_EN;
		conf->fast_charge_oc_a = CONF_FAST_CHARGE_OC_A;
		if (conf->max_charge_current <= conf->min_charge_current ||
				conf->max_charge_current >= conf->fast_charge_oc_a) {
			conf->max_charge_current = CONF_MAX_CHARGE_CURRENT;
		}
	}
	memset(conf->config_reserved, 0, sizeof(conf->config_reserved));
	return true;
}

bool jfbms_master_validate_config(const main_config_t *conf) {
	if (!conf) return false;
	const float *safety_floats[] = {
		&conf->batt_ah, &conf->vc_empty, &conf->vc_full,
		&conf->vc_balance_start, &conf->vc_balance_end,
		&conf->vc_charge_start, &conf->vc_charge_end,
		&conf->vc_charge_min, &conf->vc_balance_min,
		&conf->balance_max_current, &conf->min_current_ah_wh_cnt,
		&conf->min_current_sleep, &conf->v_charge_detect,
		&conf->t_charge_max, &conf->t_charge_max_mos, &conf->sleep,
		&conf->min_charge_current, &conf->max_charge_current,
		&conf->soc_filter_const, &conf->t_bal_max_cell,
		&conf->t_bal_max_ic, &conf->t_charge_min,
		&conf->fast_charge_oc_a,
	};
	for (size_t i = 0; i < sizeof(safety_floats) / sizeof(safety_floats[0]); i++) {
		if (!isfinite(*safety_floats[i])) return false;
	}

	return conf->batt_ah > 0.0f && conf->batt_ah <= 10000.0f &&
			conf->max_bal_ch >= 1 && conf->max_bal_ch <= 8 &&
			conf->vc_empty >= 1.5f && conf->vc_empty < conf->vc_full &&
			conf->vc_full <= 5.0f &&
			conf->vc_balance_start >= 0.0f && conf->vc_balance_start <= 0.5f &&
			conf->vc_balance_end >= 0.0f &&
			conf->vc_balance_end <= conf->vc_balance_start &&
			conf->vc_charge_min >= 1.5f &&
			conf->vc_charge_min < conf->vc_charge_start &&
			conf->vc_charge_start <= conf->vc_charge_end &&
			conf->vc_charge_end <= 5.0f &&
			conf->vc_balance_min >= 1.5f && conf->vc_balance_min <= 5.0f &&
			conf->balance_max_current >= 0.0f && conf->balance_max_current <= 100.0f &&
			conf->min_current_ah_wh_cnt >= 0.0f && conf->min_current_ah_wh_cnt <= 100.0f &&
			conf->min_current_sleep >= 0.0f && conf->min_current_sleep <= 100.0f &&
			conf->v_charge_detect >= 1.0f && conf->v_charge_detect <= 200.0f &&
			conf->t_charge_min >= -40.0f &&
			conf->t_charge_min < conf->t_charge_max && conf->t_charge_max <= 120.0f &&
			conf->t_charge_max_mos >= -40.0f && conf->t_charge_max_mos <= 120.0f &&
			conf->t_bal_max_cell >= -40.0f && conf->t_bal_max_cell <= 120.0f &&
			conf->t_bal_max_ic >= -40.0f && conf->t_bal_max_ic <= 120.0f &&
			conf->sleep >= 0.0f && conf->sleep <= 8760.0f &&
			conf->shutdown >= 0 && conf->shutdown <= 3650 &&
			conf->min_charge_current > 0.0f &&
			conf->min_charge_current < conf->max_charge_current &&
			(!conf->fast_charge_oc_en ||
				conf->max_charge_current < conf->fast_charge_oc_a) &&
			conf->fast_charge_oc_a <= JFBMS_FAST_OC_MAX_A &&
			conf->soc_filter_const >= 0.0f && conf->soc_filter_const <= 1.0f &&
			conf->num_slaves >= 1 && conf->num_slaves <= MAX_SLAVES;
}

bool jfbms_master_apply_config(void) {
	const main_config_t *cfg = (const main_config_t *)&backup.config;
	if (!jfbms_master_validate_config(cfg)) {
		return false;
	}
	int new_slave_count = configured_slave_count();
	bool runtime_reconfigure = m_applied_slave_count >= 0;
	int stop_count = m_applied_slave_count > new_slave_count ?
			m_applied_slave_count : new_slave_count;
	if (runtime_reconfigure) {
		GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
		if (!stop_balancing_for_config_change(stop_count)) return false;
		reset_pack_state_for_config_change();
	}

	bool monitor_changed = !m_fast_oc_config_valid || !jfbms_fast_adc_ready() ||
			m_fast_oc_config_en != cfg->fast_charge_oc_en ||
			m_fast_oc_config_trip_a != cfg->fast_charge_oc_a;
	if (monitor_changed) {
		GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
		// Rebuild both raw thresholds immediately; a lower configured trip must
		// never wait for a reboot. Before the first pre-charge zero this uses the
		// conservative 1.65 V midpoint.
		if (!jfbms_fast_adc_set_current_offset(m_current_offset)) {
			return false;
		}
		remember_fast_oc_config(cfg);
	}
	m_applied_slave_count = new_slave_count;
	return true;
}

// The configured topology is deliberately contiguous: slave IDs 1..N.
// Frames from other IDs are rejected in the receive path and must never leak
// into pack state or the UI.
static int configured_slave_count(void) {
	int count = ((main_config_t *)&backup.config)->num_slaves;
	if (count < 1) return 1;
	if (count > MAX_SLAVES) return MAX_SLAVES;
	return count;
}

// ============================================================================
// Section B: Master CAN Protocol
// ============================================================================

// CAN ID macros (matching slave TX format)
#define CAN_ID_CELLS(type, slave_id)  (((type) << 7) | (slave_id))
#define CAN_ID_TEMPS(slave_id)        (0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)       (0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)      (0x500 | (slave_id))

// Status frame byte 7 is an external-temperature-sensor enable mask.
#define STATUS_TEMP_BQ1_ENABLED       (1U << 0)
#define STATUS_TEMP_BQ2_ENABLED       (1U << 1)

// CAN RX circular buffer for 11-bit messages
#define CAN_BUF_SIZE 128
#define SLAVE_BROADCAST_ASSEMBLY_TIMEOUT_MS 75U
// Charge remains guarded by the independent 300 ms hardware callback. Normal
// pack control gets a wider window because balancing and BQ I2C service can
// occasionally delay a nominal 100 ms broadcast without meaning the slave is
// gone. Presence is deliberately slower and based on elapsed time, not on how
// often concurrent Lisp threads happen to call master-check-timeouts.
#define SLAVE_CHARGE_FRESHNESS_TIMEOUT_MS 300U
#define SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS 1000U
#define SLAVE_PRESENCE_TIMEOUT_MS 3000U
_Static_assert(SLAVE_CHARGE_FRESHNESS_TIMEOUT_MS < SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS &&
		SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS <= SLAVE_PRESENCE_TIMEOUT_MS,
		"JFBMS slave timeout ordering must preserve the charge safety boundary");
#define SLAVE_CAN_BUS_ESC      0
#define SLAVE_CAN_BUS_PRIVATE  1
#define SLAVE_CAN_BUS_UNKNOWN  0xFF

#define STAGE_CELL_FRAME_MASK  0x00FFU
#define STAGE_TEMP_FRAME_BIT   0x0100U

typedef struct {
	uint32_t id;
	uint32_t rx_ms;
	uint8_t data[8];
	uint8_t len;
	uint8_t bus;
} can_msg_t;

static can_msg_t can_rx_buf[CAN_BUF_SIZE];
static volatile int can_rx_head = 0;
static volatile int can_rx_tail = 0;
static volatile uint32_t can_rx_overflow = 0;
static volatile uint32_t can_rx_total = 0;
static volatile uint32_t can_rx_esc_total = 0;
static volatile uint32_t can_rx_private_total = 0;
static volatile uint32_t can_rx_filtered_total = 0;
static volatile uint32_t can_rx_filtered_last_id = 0;
static volatile uint32_t can_rx_malformed_total = 0;
static volatile uint32_t can_rx_malformed_slave[MAX_SLAVES] = {0};
static uint32_t debug_rate_last_ms = 0;
static uint32_t debug_status_last[MAX_SLAVES] = {0};
static uint32_t debug_complete_last[MAX_SLAVES] = {0};

static volatile bool slave_can_running = false;
static volatile uint32_t slave_can_tx_ok_cnt = 0;
static volatile uint32_t slave_can_tx_fail_cnt = 0;
static volatile uint32_t slave_can_tx_timeout_cnt = 0;
static volatile esp_err_t slave_can_last_error = ESP_OK;
static uint8_t slave_can_rx_bus[MAX_SLAVES];

// Master slave data
// Keep each broadcast private until all required frames and its final status
// have arrived. The live snapshot is only replaced by one complete commit.
typedef struct {
	uint16_t cell_voltages[CELLS_PER_SLAVE];
	int16_t temperatures[TEMPS_PER_SLAVE];
	uint32_t balance_mask;
	uint8_t fault_flags;
	uint8_t cells_ic1;
	uint8_t cells_ic2;
	uint8_t temp_sensor_flags;
	uint8_t bus;
	uint16_t received_mask;
	uint16_t last_missing_mask;
	uint32_t start_ms;
	uint32_t last_frame_ms;
	uint32_t last_complete_ms;
	uint32_t raw_status_count;
	uint32_t complete_count;
	uint32_t incomplete_count;
	uint32_t timeout_count;
	uint32_t restart_count;
	uint32_t orphan_count;
	uint32_t invalid_count;
	uint32_t bus_mismatch_count;
	uint32_t duplicate_count;
	uint32_t generation;
	bool settled;
	bool in_progress;
} slave_broadcast_stage_t;

static master_bms_data_t m_bms_data;
static slave_broadcast_stage_t m_slave_stage[MAX_SLAVES];
static SemaphoreHandle_t m_data_mutex;
static SemaphoreHandle_t m_balance_tx_mutex;
static StaticSemaphore_t m_data_mutex_storage;
static StaticSemaphore_t m_balance_tx_mutex_storage;
static volatile bool m_balance_inhibit;
static volatile bool m_balance_requested;
static uint32_t m_balance_stop_ms;
static uint32_t m_pack_generation;
static volatile uint32_t m_slave_complete_ms[MAX_SLAVES];
static volatile bool m_slave_snapshot_safe[MAX_SLAVES];
static volatile bool m_slave_charge_safe[MAX_SLAVES];
static esp_timer_handle_t m_pack_safety_timer;
static volatile bool m_pack_watchdog_ready;

static bool slave_cell_counts_valid_values(int cells_ic1, int cells_ic2) {
	return cells_ic1 >= 3 && cells_ic1 <= 16 &&
			(cells_ic2 == 0 || (cells_ic2 >= 3 && cells_ic2 <= 16));
}

static uint16_t required_cell_frame_mask(int cells_ic1, int cells_ic2) {
	uint16_t mask = (uint16_t)((1U << ((cells_ic1 + 3) / 4)) - 1U);
	if (cells_ic2 > 0) {
		mask |= (uint16_t)(((1U << ((cells_ic2 + 3) / 4)) - 1U) << 4);
	}
	return mask;
}

static void clear_slave_stage_state_locked(int idx) {
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	memset(stage->cell_voltages, 0, sizeof(stage->cell_voltages));
	for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
		stage->temperatures[t] = 0x7FFF;
	}
	stage->balance_mask = 0;
	stage->fault_flags = 0;
	stage->cells_ic1 = 0;
	stage->cells_ic2 = 0;
	stage->temp_sensor_flags = 0;
	stage->bus = SLAVE_CAN_BUS_UNKNOWN;
	stage->received_mask = 0;
	stage->start_ms = 0;
	stage->last_frame_ms = 0;
	stage->settled = false;
	stage->in_progress = false;
}

static void clear_slave_stage_diagnostics_locked(int idx) {
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	stage->last_missing_mask = 0;
	stage->last_complete_ms = 0;
	stage->raw_status_count = 0;
	stage->complete_count = 0;
	stage->incomplete_count = 0;
	stage->timeout_count = 0;
	stage->restart_count = 0;
	stage->orphan_count = 0;
	stage->invalid_count = 0;
	stage->bus_mismatch_count = 0;
	stage->duplicate_count = 0;
	stage->generation = 0;
}

static uint16_t stage_expected_mask_locked(int idx) {
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	if (slave_cell_counts_valid_values(stage->cells_ic1, stage->cells_ic2)) {
		return required_cell_frame_mask(stage->cells_ic1, stage->cells_ic2) |
				STAGE_TEMP_FRAME_BIT;
	}
	if (slave_cell_counts_valid_values(m_bms_data.cells_ic1[idx],
			m_bms_data.cells_ic2[idx])) {
		return required_cell_frame_mask(m_bms_data.cells_ic1[idx],
				m_bms_data.cells_ic2[idx]) | STAGE_TEMP_FRAME_BIT;
	}
	return STAGE_CELL_FRAME_MASK | STAGE_TEMP_FRAME_BIT;
}

static void discard_slave_stage_locked(int idx) {
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	stage->last_missing_mask = stage_expected_mask_locked(idx) &
			(uint16_t)~stage->received_mask;
	stage->incomplete_count++;
	clear_slave_stage_state_locked(idx);
}

static bool expire_slave_stage_locked(int idx, uint32_t now_ms) {
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	if (!stage->in_progress ||
			(now_ms - stage->start_ms) <= SLAVE_BROADCAST_ASSEMBLY_TIMEOUT_MS) {
		return false;
	}

	stage->timeout_count++;
	discard_slave_stage_locked(idx);
	return true;
}

static void clear_slave_data(int idx) {
	memset(m_bms_data.cell_voltages[idx], 0, sizeof(m_bms_data.cell_voltages[idx]));
	memset(m_bms_data.cell_last_seen_ms[idx], 0, sizeof(m_bms_data.cell_last_seen_ms[idx]));
	for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
		m_bms_data.temperatures[idx][t] = 0x7FFF;
	}
	m_bms_data.balance_mask[idx] = 0;
	m_bms_data.fault_flags[idx] = 0;
	m_bms_data.cells_ic1[idx] = 0;
	m_bms_data.cells_ic2[idx] = 0;
	m_bms_data.temp_sensor_flags[idx] = 0;
	m_bms_data.last_seen_ms[idx] = 0;
	m_bms_data.temp_last_seen_ms[idx] = 0;
	m_bms_data.status_last_seen_ms[idx] = 0;
	m_bms_data.frame_rx_count[idx] = 0;
	m_bms_data.status_rx_count[idx] = 0;
	m_bms_data.settled[idx] = false;
	m_bms_data.fresh[idx] = false;
	m_bms_data.active[idx] = false;
	m_bms_data.stale_checks[idx] = 0;
	slave_can_rx_bus[idx] = SLAVE_CAN_BUS_UNKNOWN;
	clear_slave_stage_state_locked(idx);
	clear_slave_stage_diagnostics_locked(idx);
}

static bool timestamp_fresh(uint32_t timestamp, uint32_t now, uint32_t timeout) {
	return timestamp != 0 && (now - timestamp) <= timeout;
}

static bool slave_cell_counts_valid_locked(int idx) {
	return slave_cell_counts_valid_values(m_bms_data.cells_ic1[idx],
			m_bms_data.cells_ic2[idx]);
}

// Keep the hardware filter independent of runtime topology. The receive path
// validates protocol IDs and configured slave range in software, so changing
// Number of Slaves cannot strand new devices behind an old boot-time filter.
static void configured_slave_filter(uint32_t *id, uint32_t *mask) {
	*id = 0;
	*mask = 0;
}

// Cell frames reserve types 0-3 for IC1 and 4-7 for IC2. Convert a compact
// logical cell index (IC1 cells followed by IC2 cells) to that wire layout.
static int slave_cell_wire_index(int logical_index, int cells_ic1) {
	return logical_index < cells_ic1 ? logical_index :
			16 + (logical_index - cells_ic1);
}

static bool slave_data_fresh_locked(int idx, uint32_t now, uint32_t timeout) {
	if (!timestamp_fresh(m_bms_data.status_last_seen_ms[idx], now, timeout) ||
			!timestamp_fresh(m_bms_data.temp_last_seen_ms[idx], now, timeout)) {
		return false;
	}

	if (!slave_cell_counts_valid_locked(idx)) {
		return false;
	}

	int ic1_frames = (m_bms_data.cells_ic1[idx] + 3) / 4;
	for (int frame = 0; frame < ic1_frames; frame++) {
		if (!timestamp_fresh(m_bms_data.cell_last_seen_ms[idx][frame], now, timeout)) {
			return false;
		}
	}

	int ic2_frames = (m_bms_data.cells_ic2[idx] + 3) / 4;
	for (int frame = 0; frame < ic2_frames; frame++) {
		if (!timestamp_fresh(m_bms_data.cell_last_seen_ms[idx][4 + frame], now, timeout)) {
			return false;
		}
	}

	return true;
}

static bool slave_snapshot_values_safe_locked(int idx) {
	if (!slave_cell_counts_valid_locked(idx) ||
			(m_bms_data.fault_flags[idx] & 0x0B) != 0) {
		return false;
	}

	int cells_ic1 = m_bms_data.cells_ic1[idx];
	int cell_count = cells_ic1 + m_bms_data.cells_ic2[idx];
	for (int cell = 0; cell < cell_count; cell++) {
		int wire = slave_cell_wire_index(cell, cells_ic1);
		uint16_t mv = m_bms_data.cell_voltages[idx][wire];
		if (mv < 1000 || mv > 5000) return false;
	}

	bool temp_required[TEMPS_PER_SLAVE] = {
		true,
		(m_bms_data.temp_sensor_flags[idx] & STATUS_TEMP_BQ1_ENABLED) != 0,
		m_bms_data.cells_ic2[idx] > 0,
		m_bms_data.cells_ic2[idx] > 0 &&
				(m_bms_data.temp_sensor_flags[idx] & STATUS_TEMP_BQ2_ENABLED) != 0,
	};
	for (int i = 0; i < TEMPS_PER_SLAVE; i++) {
		if (!temp_required[i]) continue;
		int16_t raw = m_bms_data.temperatures[idx][i];
		if (raw == 0x7FFF || raw < -400 || raw > 1200) return false;
	}
	return true;
}

static bool slave_snapshot_charge_safe_locked(int idx) {
	if (!slave_snapshot_values_safe_locked(idx)) return false;
	const main_config_t *cfg = (const main_config_t *)&backup.config;
	int cells_ic1 = m_bms_data.cells_ic1[idx];
	int cell_count = cells_ic1 + m_bms_data.cells_ic2[idx];
	for (int cell = 0; cell < cell_count; cell++) {
		int wire = slave_cell_wire_index(cell, cells_ic1);
		float voltage = (float)m_bms_data.cell_voltages[idx][wire] / 1000.0f;
		if (voltage <= cfg->vc_charge_min || voltage >= cfg->vc_charge_end) {
			return false;
		}
	}

	if (!cfg->t_charge_mon_en) return true;
	bool temp_required[TEMPS_PER_SLAVE] = {
		true,
		(m_bms_data.temp_sensor_flags[idx] & STATUS_TEMP_BQ1_ENABLED) != 0,
		m_bms_data.cells_ic2[idx] > 0,
		m_bms_data.cells_ic2[idx] > 0 &&
				(m_bms_data.temp_sensor_flags[idx] & STATUS_TEMP_BQ2_ENABLED) != 0,
	};
	for (int i = 0; i < TEMPS_PER_SLAVE; i++) {
		if (!temp_required[i]) continue;
		float temp_c = (float)m_bms_data.temperatures[idx][i] / 10.0f;
		if ((i == 0 || i == 2) ? temp_c >= cfg->t_charge_max_mos :
				(temp_c <= cfg->t_charge_min || temp_c >= cfg->t_charge_max)) {
			return false;
		}
	}
	return true;
}

static bool pack_safety_ready_locked(uint32_t now_ms) {
	int count = configured_slave_count();
	for (int idx = 0; idx < count; idx++) {
		bool fresh = slave_data_fresh_locked(idx, now_ms,
				SLAVE_CHARGE_FRESHNESS_TIMEOUT_MS);
		m_bms_data.fresh[idx] = fresh;
		if (!fresh || !slave_snapshot_charge_safe_locked(idx)) return false;
	}
	return true;
}

static bool pack_charge_fresh_locked(uint32_t now_ms) {
	int count = configured_slave_count();
	for (int idx = 0; idx < count; idx++) {
		if (!slave_data_fresh_locked(idx, now_ms,
				SLAVE_CHARGE_FRESHNESS_TIMEOUT_MS)) {
			return false;
		}
	}
	return count > 0;
}

static bool pack_safety_ready(void) {
	uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool ready = pack_safety_ready_locked(now_ms);
	xSemaphoreGive(m_data_mutex);
	return ready;
}

// A 1 ms independent watchdog forces CHG_EN low if a configured slave has no
// complete safe snapshot within the 300 ms safety window.
static void pack_safety_timer_cb(void *arg) {
	(void)arg;
	// m_slave_complete_ms is recorded from the FreeRTOS tick clock. Comparing it
	// with esp_timer_get_time() mixes two different boot epochs; once their
	// offset exceeds 300 ms the watchdog cuts every valid charge-enable pulse.
	uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	int count = configured_slave_count();
	for (int idx = 0; idx < count; idx++) {
		uint32_t commit_ms = m_slave_complete_ms[idx];
		uint32_t age_ms = commit_ms == 0 ? UINT32_MAX : now_ms - commit_ms;
		uint8_t reason = commit_ms == 0 ? 1 :
				(age_ms > SLAVE_CHARGE_FRESHNESS_TIMEOUT_MS ? 2 :
				(!m_slave_snapshot_safe[idx] ? 3 :
				(!m_slave_charge_safe[idx] ? 4 : 0)));
		if (reason != 0) {
			GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
			return;
		}
	}
}


// ============================================================================
// Section B: Slave CAN Bus, RX Buffer & Message Parsing
// ============================================================================

static bool decode_slave_can_id(uint32_t id, uint8_t *slave_id, uint8_t *msg_type) {
	if (id > 0x7FF) {
		return false;
	}

	uint8_t sid = id & 0x0F;
	uint8_t subtype = (id >> 4) & 0x07;
	uint8_t type = (id >> 7) & 0x0F;

	if (subtype != 0 || sid < 1 || sid > MAX_SLAVES || type > 0x09) {
		return false;
	}

	*slave_id = sid;
	*msg_type = type;
	return true;
}

static bool valid_slave_msg_len(uint8_t msg_type, int len) {
	if (msg_type <= 0x08) {
		return len == 8;
	}

	return msg_type == 0x09 && len == 8;
}

static void slave_can_buffer_rx(uint32_t id, const uint8_t *data, int len, bool is_ext, uint8_t bus) {
	if (is_ext) {
		return;  // Only handle 11-bit standard IDs
	}

	uint8_t slave_id = 0;
	uint8_t msg_type = 0xFF;
	if (!decode_slave_can_id(id, &slave_id, &msg_type)) {
		return;
	}
	if (!valid_slave_msg_len(msg_type, len)) {
		can_rx_malformed_total++;
		can_rx_malformed_slave[slave_id - 1]++;
		return;
	}

	if (slave_id > configured_slave_count()) {
		can_rx_filtered_total++;
		can_rx_filtered_last_id = id;
		return;
	}

	int next_head = (can_rx_head + 1) % CAN_BUF_SIZE;
	if (next_head == can_rx_tail) {
		can_rx_overflow++;
		return;  // Buffer full
	}

	can_rx_buf[can_rx_head].id = id;
	can_rx_buf[can_rx_head].rx_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	can_rx_buf[can_rx_head].len = (len > 8) ? 8 : len;
	can_rx_buf[can_rx_head].bus = bus;
	memcpy(can_rx_buf[can_rx_head].data, data, can_rx_buf[can_rx_head].len);
	can_rx_head = next_head;
	can_rx_total++;

	if (bus == SLAVE_CAN_BUS_ESC) {
		can_rx_esc_total++;
	} else if (bus == SLAVE_CAN_BUS_PRIVATE) {
		can_rx_private_total++;
	}
}

// One-bus mode handles slave frames from the normal comm_can RX hook. Dedicated
// TWAI1 mode ignores primary-bus slave frames unless a fallback build explicitly
// enables the legacy shared-bus path.
void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
#if JFBMS_USE_DEDICATED_SLAVE_TWAI && !JFBMS_ALLOW_SHARED_SLAVE_CAN_FALLBACK
	(void)id;
	(void)data;
	(void)len;
	(void)is_ext;
#else
	slave_can_buffer_rx(id, data, len, is_ext, SLAVE_CAN_BUS_ESC);
#endif
}

void hw_can2_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	slave_can_buffer_rx(id, data, len, is_ext, SLAVE_CAN_BUS_PRIVATE);
}

static bool __attribute__((unused)) slave_can_start(void) {
#if !JFBMS_USE_DEDICATED_SLAVE_TWAI
	slave_can_running = false;
	slave_can_last_error = JFBMS_DEDICATED_SLAVE_TWAI_PIN_COLLISION ?
			ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE;
	return false;
#elif !defined(CONFIG_IDF_TARGET_ESP32C6) || SOC_TWAI_CONTROLLER_NUM < 2
	slave_can_running = false;
	slave_can_last_error = ESP_ERR_NOT_SUPPORTED;
	return false;
#else
	if (comm_can2_is_running()) {
		slave_can_running = true;
		slave_can_last_error = ESP_OK;
		return true;
	}

	uint32_t filter_id = 0;
	uint32_t filter_mask = 0;
	configured_slave_filter(&filter_id, &filter_mask);
	comm_can2_set_mask_filter(filter_id, filter_mask, false);
	comm_can2_use_vesc_decoder(false);
	comm_can2_start(JFBMS_SLAVE_CAN_TX_GPIO_NUM, JFBMS_SLAVE_CAN_RX_GPIO_NUM,
			JFBMS_SLAVE_CAN_BAUD_KBITS);

	slave_can_running = comm_can2_is_running();
	if (!slave_can_running) {
		comm_can2_debug_info_t can2_dbg;
		comm_can2_get_debug_info(&can2_dbg);
		slave_can_last_error = (can2_dbg.last_error != ESP_OK) ?
				can2_dbg.last_error : ESP_FAIL;
		return false;
	}

	slave_can_last_error = ESP_OK;
	return true;
#endif
}

static bool slave_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

#if !JFBMS_USE_DEDICATED_SLAVE_TWAI
	esp_err_t res = comm_can_transmit_sid_sync(id, data, len, 10);
	if (res != ESP_OK) {
		slave_can_last_error = res;
		slave_can_tx_fail_cnt++;
		if (res == ESP_ERR_TIMEOUT) slave_can_tx_timeout_cnt++;
		return false;
	}
	slave_can_last_error = ESP_OK;
	slave_can_tx_ok_cnt++;
	return true;
#else
#if JFBMS_ALLOW_SHARED_SLAVE_CAN_FALLBACK
	uint8_t slave_id = id & 0x0F;
	uint8_t bus = SLAVE_CAN_BUS_UNKNOWN;
	if (slave_id >= 1 && slave_id <= MAX_SLAVES) {
		bus = slave_can_rx_bus[slave_id - 1];
	}

	if (bus == SLAVE_CAN_BUS_ESC) {
		esp_err_t res = comm_can_transmit_sid_sync(id, data, len, 10);
		if (res != ESP_OK) {
			slave_can_last_error = res;
			slave_can_tx_fail_cnt++;
			if (res == ESP_ERR_TIMEOUT) slave_can_tx_timeout_cnt++;
			return false;
		}
		slave_can_last_error = ESP_OK;
		slave_can_tx_ok_cnt++;
		return true;
	}
#endif

	if (!comm_can2_is_running() && !slave_can_start()) {
		slave_can_tx_fail_cnt++;
		return false;
	}

	esp_err_t res = comm_can2_transmit_sid_sync(id, data, len, 10);
	if (res != ESP_OK) {
		slave_can_last_error = res;
		slave_can_tx_fail_cnt++;
		if (res == ESP_ERR_TIMEOUT) {
			slave_can_tx_timeout_cnt++;
		}
		return false;
	}

	slave_can_last_error = ESP_OK;
	slave_can_tx_ok_cnt++;
	return true;
#endif
}

// Checked standard-ID transmit used by the balance stop handoff. Success means
// the frame completed on the physical TWAI bus, not merely that it entered a
// software queue.
static bool slave_can_transmit_sid_sync(uint32_t id, const uint8_t *data,
		uint8_t len, int timeout_ms) {
	if (len > 8) len = 8;
	if (timeout_ms <= 0) {
		slave_can_last_error = ESP_ERR_TIMEOUT;
		slave_can_tx_fail_cnt++;
		slave_can_tx_timeout_cnt++;
		return false;
	}

	esp_err_t res = ESP_ERR_INVALID_STATE;
#if !JFBMS_USE_DEDICATED_SLAVE_TWAI
	res = comm_can_transmit_sid_sync(id, data, len, timeout_ms);
#else
#if JFBMS_ALLOW_SHARED_SLAVE_CAN_FALLBACK
	uint8_t slave_id = id & 0x0F;
	if (slave_id >= 1 && slave_id <= MAX_SLAVES &&
			slave_can_rx_bus[slave_id - 1] == SLAVE_CAN_BUS_ESC) {
		res = comm_can_transmit_sid_sync(id, data, len, timeout_ms);
	} else
#endif
	{
		if (!comm_can2_is_running() && !slave_can_start()) {
			slave_can_tx_fail_cnt++;
			return false;
		}
		res = comm_can2_transmit_sid_sync(id, data, len, timeout_ms);
	}
#endif

	slave_can_last_error = res;
	if (res == ESP_OK) {
		slave_can_tx_ok_cnt++;
		return true;
	}
	slave_can_tx_fail_cnt++;
	if (res == ESP_ERR_TIMEOUT) slave_can_tx_timeout_cnt++;
	return false;
}

static bool stop_balancing_for_config_change(int slave_count) {
	if (!m_balance_tx_mutex) return false;
	if (slave_count < 1) slave_count = 1;
	if (slave_count > MAX_SLAVES) slave_count = MAX_SLAVES;

	bool any_known_slave = false;
	if (m_data_mutex) {
		xSemaphoreTake(m_data_mutex, portMAX_DELAY);
		for (int idx = 0; idx < slave_count; idx++) {
			if (m_bms_data.active[idx]) any_known_slave = true;
		}
		xSemaphoreGive(m_data_mutex);
	}
	if (!any_known_slave) return true;

	uint8_t zero_cmd[5] = {0, 0, 0, 0, 0};
	m_balance_inhibit = true;
	m_balance_requested = true;
	GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
	if (xSemaphoreTake(m_balance_tx_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
		return false;
	}

	bool success = true;
	for (int pass = 0; pass < 3 && success; pass++) {
		for (int slave = 1; slave <= slave_count; slave++) {
			if (!slave_can_transmit_sid_sync(CAN_ID_BAL_CMD(slave), zero_cmd,
					sizeof(zero_cmd), 10)) {
				success = false;
				break;
			}
		}
	}
	if (success) {
		m_balance_stop_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
		m_balance_inhibit = false;
		m_balance_requested = false;
	}
	xSemaphoreGive(m_balance_tx_mutex);
	return success;
}

static void reset_pack_state_for_config_change(void) {
	if (!m_data_mutex) return;
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	for (int idx = 0; idx < MAX_SLAVES; idx++) {
		clear_slave_data(idx);
		m_slave_complete_ms[idx] = 0;
		m_slave_snapshot_safe[idx] = false;
		m_slave_charge_safe[idx] = false;
	}
	m_pack_generation = 0;
	can_rx_head = 0;
	can_rx_tail = 0;
	xSemaphoreGive(m_data_mutex);
}

// Parse a single CAN message from a slave. Cell frame 0 starts a candidate;
// the status frame commits it only after every required frame is present.
static void parse_slave_message(uint32_t id, uint8_t *data, int len, uint8_t bus,
		uint32_t rx_ms) {
	uint8_t slave_id = 0;
	uint8_t msg_type = 0xFF;

	if (!decode_slave_can_id(id, &slave_id, &msg_type) ||
			!valid_slave_msg_len(msg_type, len)) {
		return;
	}

	uint8_t idx = slave_id - 1;  // 0-based index
	uint32_t now_ms = rx_ms;
	bool force_balance_inhibit = false;
	bool balance_report_changed = false;
	uint32_t balance_report_mask = 0;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	m_bms_data.frame_rx_count[idx]++;
	expire_slave_stage_locked(idx, now_ms);
	slave_broadcast_stage_t *stage = &m_slave_stage[idx];
	if (msg_type == 0x09) {
		stage->raw_status_count++;
	}

	if (msg_type == 0x00) {
		if (stage->in_progress) {
			stage->duplicate_count++;
			if (stage->bus != bus) {
				stage->bus_mismatch_count++;
			} else {
				stage->restart_count++;
			}
			discard_slave_stage_locked(idx);
		}

		clear_slave_stage_state_locked(idx);
		stage->in_progress = true;
		stage->bus = bus;
		stage->start_ms = now_ms;
		stage->last_frame_ms = now_ms;
	}

	if (!stage->in_progress) {
		stage->orphan_count++;
		xSemaphoreGive(m_data_mutex);
		return;
	}

	if (stage->bus != bus) {
		stage->bus_mismatch_count++;
		discard_slave_stage_locked(idx);
		xSemaphoreGive(m_data_mutex);
		return;
	}

	stage->last_frame_ms = now_ms;
	if (msg_type <= 0x07) {
		// Cell voltage message: 4 cells per message, little-endian uint16 mV.
		uint8_t base_cell = msg_type * 4;
		for (int i = 0; i < 4 && (base_cell + i) < CELLS_PER_SLAVE; i++) {
			stage->cell_voltages[base_cell + i] =
				(uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
		}
		uint16_t frame_bit = (uint16_t)(1U << msg_type);
		if ((stage->received_mask & frame_bit) != 0) {
			stage->duplicate_count++;
		}
		stage->received_mask |= frame_bit;
	} else if (msg_type == 0x08) {
		// Temperature message: 4 temps, little-endian int16 0.1 deg C.
		for (int i = 0; i < TEMPS_PER_SLAVE; i++) {
			stage->temperatures[i] =
				(int16_t)((uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
		}
		if ((stage->received_mask & STAGE_TEMP_FRAME_BIT) != 0) {
			stage->duplicate_count++;
		}
		stage->received_mask |= STAGE_TEMP_FRAME_BIT;
	} else if (msg_type == 0x09) {
		// Status is the protocol-defined final frame. Its cell counts define the
		// exact cell-frame set required for this candidate.
		stage->balance_mask =
				(uint32_t)data[0] |
				((uint32_t)data[1] << 8) |
				((uint32_t)data[2] << 16) |
				((uint32_t)data[3] << 24);
		stage->fault_flags = data[4];
		stage->settled = (data[4] & 0x04) != 0;
		stage->temp_sensor_flags = data[7] &
				(STATUS_TEMP_BQ1_ENABLED | STATUS_TEMP_BQ2_ENABLED);
		stage->cells_ic1 = data[5];
		stage->cells_ic2 = data[6];

		if (!slave_cell_counts_valid_values(stage->cells_ic1, stage->cells_ic2)) {
			stage->invalid_count++;
			discard_slave_stage_locked(idx);
			xSemaphoreGive(m_data_mutex);
			return;
		}

		uint16_t expected_cells = required_cell_frame_mask(stage->cells_ic1,
				stage->cells_ic2);
		uint16_t expected = expected_cells | STAGE_TEMP_FRAME_BIT;
		uint16_t received_cells = stage->received_mask & STAGE_CELL_FRAME_MASK;
		bool missing = (stage->received_mask & expected) != expected;
		bool unexpected = (received_cells & (uint16_t)~expected_cells) != 0;
		if (missing || unexpected ||
				(now_ms - stage->start_ms) > SLAVE_BROADCAST_ASSEMBLY_TIMEOUT_MS) {
			if (unexpected) {
				stage->invalid_count++;
			}
			if ((now_ms - stage->start_ms) > SLAVE_BROADCAST_ASSEMBLY_TIMEOUT_MS) {
				stage->timeout_count++;
			}
			discard_slave_stage_locked(idx);
			xSemaphoreGive(m_data_mutex);
			return;
		}

		// Publish all values and freshness timestamps together. No partial frame
		// can update the live snapshot used by the rest of the BMS.
		memcpy(m_bms_data.cell_voltages[idx], stage->cell_voltages,
				sizeof(m_bms_data.cell_voltages[idx]));
		memcpy(m_bms_data.temperatures[idx], stage->temperatures,
				sizeof(m_bms_data.temperatures[idx]));
		balance_report_changed = m_bms_data.balance_mask[idx] != stage->balance_mask;
		balance_report_mask = stage->balance_mask;
		m_bms_data.balance_mask[idx] = balance_report_mask;
		m_bms_data.fault_flags[idx] = stage->fault_flags;
		m_bms_data.settled[idx] = stage->settled;
		m_bms_data.temp_sensor_flags[idx] = stage->temp_sensor_flags;
		m_bms_data.cells_ic1[idx] = stage->cells_ic1;
		m_bms_data.cells_ic2[idx] = stage->cells_ic2;
		m_bms_data.last_seen_ms[idx] = now_ms;
		memset(m_bms_data.cell_last_seen_ms[idx], 0,
				sizeof(m_bms_data.cell_last_seen_ms[idx]));
		for (int frame = 0; frame < 8; frame++) {
			if ((expected_cells & (1U << frame)) != 0) {
				m_bms_data.cell_last_seen_ms[idx][frame] = now_ms;
			}
		}
		m_bms_data.temp_last_seen_ms[idx] = now_ms;
		m_bms_data.status_last_seen_ms[idx] = now_ms;
		m_bms_data.status_rx_count[idx]++;
		slave_can_rx_bus[idx] = stage->bus;

		stage->last_missing_mask = 0;
		stage->last_complete_ms = now_ms;
		stage->complete_count++;
		stage->generation++;
		m_pack_generation++;
		m_slave_complete_ms[idx] = now_ms;
		m_slave_snapshot_safe[idx] = slave_snapshot_values_safe_locked(idx);
		m_slave_charge_safe[idx] = slave_snapshot_charge_safe_locked(idx);
		if (stage->balance_mask != 0 &&
				(m_balance_stop_ms == 0 ||
				(int32_t)(stage->start_ms - m_balance_stop_ms) >= 0)) {
			force_balance_inhibit = true;
		}
		clear_slave_stage_state_locked(idx);
	}

	xSemaphoreGive(m_data_mutex);
	if (balance_report_changed) {
		commands_printf_lisp("BAL REPORT S%d IC1=0x%04lX IC2=0x%04lX",
				slave_id,
				(unsigned long)(balance_report_mask & 0xFFFFU),
				(unsigned long)((balance_report_mask >> 16) & 0xFFFFU));
	}
	if (force_balance_inhibit) {
		xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
		m_balance_inhibit = true;
		m_balance_requested = true;
		gpio_set_level(PIN_CHG_EN, 0);
		xSemaphoreGive(m_balance_tx_mutex);
	}
}

// Send balance command with buzzer beep code to slave
static bool send_balance_cmd(uint8_t slave_id, uint32_t mask, uint8_t beep_code) {
	uint8_t buf[5];
	buf[0] = (mask >> 0) & 0xFF;
	buf[1] = (mask >> 8) & 0xFF;
	buf[2] = (mask >> 16) & 0xFF;
	buf[3] = (mask >> 24) & 0xFF;
	buf[4] = beep_code;
	if (mask != 0) {
		m_balance_inhibit = true;
		gpio_set_level(PIN_CHG_EN, 0);
	}
	return slave_can_transmit_sid(CAN_ID_BAL_CMD(slave_id), buf, 5);
}


// ============================================================================
// Configuration symbol table
// ============================================================================

typedef struct {
	lbm_uint can_baud_rate;
	lbm_uint can_status_rate_hz;
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
	lbm_uint sleep;
	lbm_uint shutdown;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint fast_charge_oc_en;
	lbm_uint fast_charge_oc_a;
	// Master-specific
	lbm_uint num_slaves;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.can_baud_rate) {
			lbm_add_symbol_const("can_baud_rate", comp);
		} else if (comp == &syms_vesc.can_status_rate_hz) {
			lbm_add_symbol_const("can_status_rate_hz", comp);
		} else if (comp == &syms_vesc.cells_ic1) {
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
		} else if (comp == &syms_vesc.sleep) {
			lbm_add_symbol_const("sleep", comp);
		} else if (comp == &syms_vesc.shutdown) {
			lbm_add_symbol_const("shutdown", comp);
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
		} else if (comp == &syms_vesc.fast_charge_oc_en) {
			lbm_add_symbol_const("fast_charge_oc_en", comp);
		} else if (comp == &syms_vesc.fast_charge_oc_a) {
			lbm_add_symbol_const("fast_charge_oc_a", comp);
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

static lbm_value get_or_set_can_baud(bool set, CAN_BAUD *val, lbm_value *lbm_val) {
	if (set) {
		*val = (CAN_BAUD)lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i((int)*val);
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
	main_config_t previous = *cfg;

	if (compare_symbol(name, &syms_vesc.can_baud_rate)) {
		res = get_or_set_can_baud(set, &cfg->can_baud_rate, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.can_status_rate_hz)) {
		res = get_or_set_i(set, &cfg->can_status_rate_hz, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cells_ic1)) {
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
	} else if (compare_symbol(name, &syms_vesc.sleep)) {
		res = get_or_set_float(set, &cfg->sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.shutdown)) {
		res = get_or_set_i(set, &cfg->shutdown, &set_arg);
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
	} else if (compare_symbol(name, &syms_vesc.fast_charge_oc_en)) {
		res = get_or_set_bool(set, &cfg->fast_charge_oc_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.fast_charge_oc_a)) {
		res = get_or_set_float(set, &cfg->fast_charge_oc_a, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.num_slaves)) {
		res = get_or_set_i(set, &cfg->num_slaves, &set_arg);
	}

	if (set && res == ENC_SYM_TRUE) {
		if (!jfbms_master_validate_config(cfg) || !jfbms_master_apply_config()) {
			*cfg = previous;
			(void)jfbms_master_apply_config();
			lbm_set_error_reason("Unsafe or unapplied BMS setting");
			return ENC_SYM_EERROR;
		}
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
	if (!jfbms_master_validate_config((const main_config_t *)&backup.config)) {
		lbm_set_error_reason("Invalid BMS configuration");
		return ENC_SYM_EERROR;
	}
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
		parse_slave_message(msg->id, msg->data, msg->len, msg->bus, msg->rx_ms);
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

	if (!slave_cell_counts_valid_locked(idx)) {
		xSemaphoreGive(m_data_mutex);
		return ENC_SYM_NIL;
	}

	int cells_ic1 = m_bms_data.cells_ic1[idx];
	int num_cells = cells_ic1 + m_bms_data.cells_ic2[idx];

	for (int i = num_cells - 1; i >= 0; i--) {
		int wire_index = slave_cell_wire_index(i, cells_ic1);
		uint16_t mv = m_bms_data.cell_voltages[idx][wire_index];
		if (mv != 0 && mv != 0xFFFF) {
			vc_list = lbm_cons(lbm_enc_float((float)mv / 1000.0f), vc_list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return vc_list;
}

// (master-get-slave-temps slave-id)
// Return positional temperatures in deg C:
//   single IC: (BQ1-die BQ1-external)
//   dual IC:   (BQ1-die BQ1-external BQ2-die BQ2-external)
// Missing/invalid entries are returned as -300 C instead of being removed so
// Lisp can match each reading to its explicit enable bit.
static lbm_value ext_master_get_slave_temps(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;
	lbm_value ts_list = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	if (!slave_cell_counts_valid_locked(idx)) {
		xSemaphoreGive(m_data_mutex);
		return ENC_SYM_NIL;
	}

	int temp_count = m_bms_data.cells_ic2[idx] > 0 ? 4 : 2;
	for (int i = temp_count - 1; i >= 0; i--) {
		int16_t raw = m_bms_data.temperatures[idx][i];
		float temp = raw == 0x7FFF ? -300.0f : (float)raw / 10.0f;
		ts_list = lbm_cons(lbm_enc_float(temp), ts_list);
	}

	xSemaphoreGive(m_data_mutex);

	return ts_list;
}

// (master-get-slave-status slave-id)
// Get (balance-mask faults cells-ic1 cells-ic2 temp-sensor-flags).
static lbm_value ext_master_get_slave_status(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	lbm_value res = ENC_SYM_NIL;
	res = lbm_cons(lbm_enc_i(m_bms_data.temp_sensor_flags[idx]), res);
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

// (master-slave-fresh? slave-id) - Operational freshness for display/balancing.
// Charging has a separate, stricter 300 ms C-side gate and watchdog.
static lbm_value ext_master_slave_fresh(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	int idx = slave_id - 1;

	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool fresh = slave_data_fresh_locked(idx, now, SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS);
	m_bms_data.fresh[idx] = fresh;
	xSemaphoreGive(m_data_mutex);

	return fresh ? ENC_SYM_TRUE : ENC_SYM_NIL;
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
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	bool settled = m_bms_data.active[idx] &&
			slave_data_fresh_locked(idx, now, SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS) &&
			m_bms_data.settled[idx] &&
			(m_bms_data.fault_flags[idx] & 0x0B) == 0;
	xSemaphoreGive(m_data_mutex);

	return settled ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-pack-charge-fresh?) - Expose the strict charge freshness gate so
// Lisp can report CAN_CHG_STALE instead of a generic CHG_BLOCK. CHG_EN safety
// does not depend on this diagnostic extension; it remains enforced in C.
static lbm_value ext_master_pack_charge_fresh(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool fresh = m_pack_watchdog_ready && pack_charge_fresh_locked(now);
	xSemaphoreGive(m_data_mutex);
	return fresh ? ENC_SYM_TRUE : ENC_SYM_NIL;
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
	xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
	bool sent = send_balance_cmd(slave_id, mask, beep_code);
	xSemaphoreGive(m_balance_tx_mutex);
	return sent ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-stop-balance-sync)
// Send three complete zero-mask passes to every configured slave. The 20 ms
// deadline covers the whole handoff; a failed transmission leaves the C
// inhibit asserted and CHG_EN low so Lisp can retry safely.
static lbm_value ext_master_stop_balance_sync(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint8_t zero_cmd[5] = {0, 0, 0, 0, 0};
	const uint32_t timeout_ms = 20U;
	uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	bool success = true;

	m_balance_inhibit = true;
	gpio_set_level(PIN_CHG_EN, 0);
	TickType_t handoff_wait = pdMS_TO_TICKS(timeout_ms);
	if (handoff_wait == 0) handoff_wait = 1;
	if (xSemaphoreTake(m_balance_tx_mutex, handoff_wait) != pdTRUE) {
		return ENC_SYM_NIL;
	}

	for (int pass = 0; pass < 3 && success; pass++) {
		for (int slave = 1; slave <= configured_slave_count(); slave++) {
			uint32_t elapsed_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms;
			if (elapsed_ms >= timeout_ms ||
					!slave_can_transmit_sid_sync(CAN_ID_BAL_CMD(slave), zero_cmd,
							sizeof(zero_cmd), (int)(timeout_ms - elapsed_ms))) {
				success = false;
				break;
			}
		}
	}

	if (success) {
		m_balance_stop_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
		m_balance_inhibit = false;
	} else {
		gpio_set_level(PIN_CHG_EN, 0);
	}
	xSemaphoreGive(m_balance_tx_mutex);
	return success ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-balance-request enable)
// The C request latch closes the scheduling window between Lisp state checks
// and a charge-enable attempt.
static lbm_value ext_master_balance_request(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	bool enable = lbm_dec_as_i32(args[0]) != 0;
	xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
	if (enable) {
		m_balance_requested = true;
		gpio_set_level(PIN_CHG_EN, 0);
		xSemaphoreGive(m_balance_tx_mutex);
		return ENC_SYM_TRUE;
	}
	if (m_balance_inhibit) {
		gpio_set_level(PIN_CHG_EN, 0);
		xSemaphoreGive(m_balance_tx_mutex);
		return ENC_SYM_NIL;
	}
	m_balance_requested = false;
	xSemaphoreGive(m_balance_tx_mutex);
	return ENC_SYM_TRUE;
}

// (master-balance-inhibited?) - True while a C-side balance request or
// inhibit latch requires the Lisp controller to remain fail-closed.
static lbm_value ext_master_balance_inhibited(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
	bool inhibited = m_balance_inhibit || m_balance_requested;
	xSemaphoreGive(m_balance_tx_mutex);
	return inhibited ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-set-chg enable) -- the only permitted charge-enable path. Charge is
// fail-closed until the continuous ADC monitor and pack freshness watchdog are
// both armed, and the fast overcurrent latch has been cleared.
static lbm_value ext_master_set_chg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	bool enable = lbm_dec_as_i32(args[0]) != 0;
	if (!enable) {
		gpio_set_level(PIN_CHG_EN, 0);
		return ENC_SYM_TRUE;
	}
	jfbms_charger_status_t charger_status;
	jfbms_charger_get_status(&charger_status);
	const main_config_t *cfg = (const main_config_t *)&backup.config;

	if (!jfbms_fast_adc_ready() || jfbms_fast_oc_latched() ||
			!m_current_offset_calibrated || m_calibration_in_progress ||
			!m_pack_watchdog_ready ||
			!m_current_valid || (cfg->t_charge_mon_en &&
			(!m_temp_pcb_valid || m_temp_pcb >= cfg->t_charge_max_mos)) ||
			!charger_status.valid || !charger_status.detected ||
			!pack_safety_ready()) {
		gpio_set_level(PIN_CHG_EN, 0);
		return ENC_SYM_NIL;
	}

	xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
	if (m_balance_inhibit || m_balance_requested ||
			!jfbms_fast_adc_ready() || jfbms_fast_oc_latched() ||
			!m_current_offset_calibrated || m_calibration_in_progress ||
			!m_pack_watchdog_ready) {
		gpio_set_level(PIN_CHG_EN, 0);
		xSemaphoreGive(m_balance_tx_mutex);
		return ENC_SYM_NIL;
	}

	gpio_set_level(PIN_CHG_EN, 1);
	if (m_balance_inhibit || m_balance_requested) {
		gpio_set_level(PIN_CHG_EN, 0);
		xSemaphoreGive(m_balance_tx_mutex);
		return ENC_SYM_NIL;
	}
	xSemaphoreGive(m_balance_tx_mutex);
	return ENC_SYM_TRUE;
}

// (master-check-timeouts timeout-ms) - Update operational freshness and
// elapsed-time-based presence. The charge watchdog is independent of this.
static lbm_value ext_master_check_timeouts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t timeout = lbm_dec_as_u32(args[0]);
	if (timeout > SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS) {
		timeout = SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS;
	}
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0; i < MAX_SLAVES; i++) {
		expire_slave_stage_locked(i, now);
		bool fresh = slave_data_fresh_locked(i, now, timeout);
		m_bms_data.fresh[i] = fresh;

		bool present = timestamp_fresh(m_bms_data.last_seen_ms[i], now,
				SLAVE_PRESENCE_TIMEOUT_MS);
		if (fresh) {
			m_bms_data.active[i] = true;
			m_bms_data.stale_checks[i] = 0;
		} else if (m_bms_data.active[i]) {
			if (m_bms_data.stale_checks[i] < 255) {
				m_bms_data.stale_checks[i]++;
			}
			if (!present) {
				m_bms_data.active[i] = false;
			}
		} else {
			m_bms_data.stale_checks[i] = 0;
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

	for (int s = 0; s < MAX_SLAVES; s++) {
		clear_slave_data(s);
		m_slave_complete_ms[s] = 0;
		m_slave_snapshot_safe[s] = false;
		m_slave_charge_safe[s] = false;
	}
	m_pack_generation = 0;

	xSemaphoreGive(m_data_mutex);
	xSemaphoreTake(m_balance_tx_mutex, portMAX_DELAY);
	m_balance_requested = false;
	m_balance_inhibit = false;
	m_balance_stop_ms = 0;
	gpio_set_level(PIN_CHG_EN, 0);
	xSemaphoreGive(m_balance_tx_mutex);

	// Also reset CAN buffer
	can_rx_head = 0;
	can_rx_tail = 0;
	can_rx_overflow = 0;
	can_rx_total = 0;
	can_rx_esc_total = 0;
	can_rx_private_total = 0;
	can_rx_filtered_total = 0;
	can_rx_filtered_last_id = 0;
	can_rx_malformed_total = 0;
	memset((void *)can_rx_malformed_slave, 0, sizeof(can_rx_malformed_slave));
	debug_rate_last_ms = 0;
	memset(debug_status_last, 0, sizeof(debug_status_last));
	memset(debug_complete_last, 0, sizeof(debug_complete_last));

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

// (master-get-pack-generation) - Monotonic generation of complete pack
// snapshots. Lisp uses this in SOC diagnostics to identify the source sample.
static lbm_value ext_master_get_pack_generation(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	uint32_t generation = m_pack_generation;
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_u32(generation);
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

	float staged_cells[BMS_MAX_CELLS];
	bool staged_bal_state[BMS_MAX_CELLS];
	float staged_external_temps[MAX_SLAVES * 2];
	int staged_cell_count = 0;
	int staged_external_temp_count = 0;
	bool cell_snapshot_complete = true;
	float v_tot = 0.0f;
	float v_min = 9999.0f;
	float v_max = 0.0f;
	float t_ic_max = -300.0f;
	float t_cell_min = 9999.0f;
	float t_cell_max = -300.0f;
	bool have_ic_temp = false;
	bool have_cell_temp = false;
	bool any_balancing = false;
	int expected_slaves = configured_slave_count();

	// Stage one complete, positional pack snapshot. Never compact invalid cells:
	// doing so shifts every later cell index and makes cells appear/disappear in
	// VESC Tool. If any configured slave is missing, stale, or unsafe, retain the
	// last complete published cell snapshot instead.
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	if (expected_slaves <= 0) cell_snapshot_complete = false;
	for (int s = 0; s < expected_slaves; s++) {
		if (!m_bms_data.active[s] ||
				!slave_data_fresh_locked(s, now_ms, SLAVE_CONTROL_FRESHNESS_TIMEOUT_MS) ||
				!m_slave_snapshot_safe[s] ||
				!slave_snapshot_values_safe_locked(s)) {
			cell_snapshot_complete = false;
			break;
		}
		int num_cells = m_bms_data.cells_ic1[s] + m_bms_data.cells_ic2[s];
		if (num_cells > (BMS_MAX_CELLS - staged_cell_count)) {
			cell_snapshot_complete = false;
			break;
		}
		staged_cell_count += num_cells;
	}

	if (cell_snapshot_complete) {
		staged_cell_count = 0;
		for (int s = 0; s < expected_slaves; s++) {
			int ic1_cnt = m_bms_data.cells_ic1[s];
			int num_cells = ic1_cnt + m_bms_data.cells_ic2[s];
			// Only the bitmap received in the slave's completed status frame is
			// authoritative. Split it explicitly into IC masks before mapping the
			// compact pack-cell index used by VESC Tool.
			uint16_t reported_ic1 = m_bms_data.balance_mask[s] & 0xFFFFU;
			uint16_t reported_ic2 = (m_bms_data.balance_mask[s] >> 16) & 0xFFFFU;
			for (int c = 0; c < num_cells; c++) {
				int wire_index = slave_cell_wire_index(c, ic1_cnt);
				float v = (float)m_bms_data.cell_voltages[s][wire_index] / 1000.0f;
				staged_cells[staged_cell_count] = v;
				staged_bal_state[staged_cell_count] = c < ic1_cnt ?
						((reported_ic1 >> c) & 1U) :
						((reported_ic2 >> (c - ic1_cnt)) & 1U);
				if (staged_bal_state[staged_cell_count]) any_balancing = true;
				v_tot += v;
				if (v < v_min) v_min = v;
				if (v > v_max) v_max = v;
				staged_cell_count++;
			}
		}
	}

	for (int s = 0; s < expected_slaves; s++) {
		if (!m_bms_data.active[s] || !slave_cell_counts_valid_locked(s)) continue;
		// Aggregate slave temperatures for VESC 6.06 convention
		// Temp order per slave: [0]=BQ1 IC, [1]=BQ1 TS1 (cell), [2]=BQ2 IC, [3]=BQ2 TS1 (cell)
		for (int t = 0; t < TEMPS_PER_SLAVE; t++) {
			int16_t raw = m_bms_data.temperatures[s][t];
			if (raw == 0x7FFF) continue;
			float temp_c = (float)raw / 10.0f;
			if (!bms_temp_valid(temp_c)) continue;
			if (t == 0 || t == 2) {
				if (temp_c > t_ic_max) t_ic_max = temp_c;
				have_ic_temp = true;
			} else {
				if (temp_c < t_cell_min) t_cell_min = temp_c;
				if (temp_c > t_cell_max) t_cell_max = temp_c;
				have_cell_temp = true;
				if (staged_external_temp_count < (MAX_SLAVES * 2)) {
					staged_external_temps[staged_external_temp_count++] = temp_c;
				}
			}
		}
	}

	xSemaphoreGive(m_data_mutex);

	if (cell_snapshot_complete) {
		for (int i = 0; i < staged_cell_count; i++) {
			bms->v_cell[i] = staged_cells[i];
			bms->bal_state[i] = staged_bal_state[i];
		}
		for (int i = staged_cell_count; i < BMS_MAX_CELLS; i++) {
			bms->v_cell[i] = 0.0f;
			bms->bal_state[i] = 0;
		}
		bms->v_tot = v_tot;
		bms->v_cell_min = staged_cell_count > 0 ? v_min : 0.0f;
		bms->v_cell_max = staged_cell_count > 0 ? v_max : 0.0f;
		// Publish the count last so readers never see a new topology before its
		// positional voltage array has been fully populated.
		bms->cell_num = staged_cell_count;
	}

	{
		float v = adc_get_voltage(HW_ADC_CH3);
		m_vchg_valid = v >= 0.0f && isfinite(v);
		if (m_vchg_valid) {
			float raw_vchg = v * VCHG_DIV_SCALE;
			if (!m_vchg_filter_init) {
				m_vchg_filtered = raw_vchg;
				m_vchg_filter_init = true;
			} else {
				m_vchg_filtered = VCHG_EMA_ALPHA * m_vchg_filtered
					+ (1.0f - VCHG_EMA_ALPHA) * raw_vchg;
			}
		}
		bms->v_charge = m_vchg_valid ? m_vchg_filtered : 0.0f;
	}
	// Read pack current from the continuous ADC cache, then apply a slow EMA and
	// small zero deadband to suppress idle current wander.
	{
		float v = isense_read_voltage();
		m_current_valid = v >= 0.0f && isfinite(v);
		if (m_current_valid && m_current_offset_calibrated) {
			float raw_a = (v - m_current_offset) * ISENSE_SCALE;
			if (!m_current_filter_init) {
				m_current_filtered    = raw_a;
				m_current_filter_init = true;
			} else {
				m_current_filtered = ISENSE_EMA_ALPHA * m_current_filtered
				                   + (1.0f - ISENSE_EMA_ALPHA) * raw_a;
			}
			if (fabsf(m_current_filtered) <= ISENSE_ZERO_DEADBAND_A) {
				m_current_filtered = 0.0f;
			}
			// Match JFBMS32/VESC BMS current convention: discharge is positive
			// and charging is negative. Lisp derives charge current as -iout.
			bms->i_in    = m_current_filtered;
			bms->i_in_ic = m_current_filtered;
		} else {
			m_current_filter_init = false;
			m_current_filtered = 0.0f;
			bms->i_in = 0.0f;
			bms->i_in_ic = 0.0f;
		}
	}

	{
		float v = adc_get_voltage(HW_ADC_CH4);
		m_temp_pcb_valid = false;
		if (v > 0.01f && v < (NTC_VREF - 0.01f) && isfinite(v)) {
			float r_ntc = (v * NTC_R_PULL) / (NTC_VREF - v);
			float temp_c = (1.0f / ((logf(r_ntc / NTC_R25) / NTC_BETA) + NTC_T0_INV)) - 273.15f;
			if (isfinite(temp_c) && bms_temp_valid(temp_c)) {
				m_temp_pcb_valid = true;
				if (!m_temp_pcb_filter_init) {
					m_temp_pcb = temp_c;
					m_temp_pcb_filter_init = true;
				} else {
					m_temp_pcb = NTC_EMA_ALPHA * m_temp_pcb
						+ (1.0f - NTC_EMA_ALPHA) * temp_c;
				}
			}
		}
	}

	// VESC 6.06 temperature sensor convention (indices 0-4)
	bms->temps_adc[0] = have_ic_temp ? t_ic_max : 0.0f;                 // Balance IC
	bms->temps_adc[1] = have_cell_temp ? t_cell_min : 0.0f;             // Cell Min
	bms->temps_adc[2] = have_cell_temp ? t_cell_max : 0.0f;             // Cell Max
	bms->temps_adc[3] = m_temp_pcb_valid ? m_temp_pcb : 0.0f;           // Mosfet / PCB NTC
	bms->temps_adc[4] = 0.0f;                                           // Ambient N/A
	int temps_count = 5;
	for (int i = 0; i < staged_external_temp_count && temps_count < BMS_MAX_TEMPS; i++) {
		bms->temps_adc[temps_count++] = staged_external_temps[i];
	}

	bms->temp_adc_num = temps_count;

	bms->temp_ic = have_ic_temp ? t_ic_max : 0.0f;
	bms->temp_max_cell = have_cell_temp ? t_cell_max : 0.0f;
	bms->is_charging = gpio_get_level(PIN_CHG_EN) ? 1 : 0;
	bms->is_balancing = cell_snapshot_complete && any_balancing ? 1 : 0;
	bms->data_version = 1;

	// SOC is owned by the Lisp controller. Do not overwrite it from voltage here;
	// that would race the coulomb/voltage policy and allow foreign BMS traffic to
	// replace the local value.
	bms->soh = 1.0f;

	// Do not make retained cell data look fresh after slave CAN becomes stale.
	if (cell_snapshot_complete) {
		bms->update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
	}

	return ENC_SYM_TRUE;
}

// ============================================================================
// Current Sense Extensions
// ============================================================================

// (master-calibrate-current) -- commit the latest continuous-ADC sample as the
// measurement zero. This runs synchronously from the calibration command so a
// Lisp calibration thread cannot wait forever for a periodic BMS refresh that
// may be blocked by another pack operation. The ADC reconfiguration itself is
// serialized by jfbms_fast_adc's mutex.
static lbm_value ext_master_calibrate_current(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (m_calibration_in_progress) {
		commands_printf_lisp("Current calibration is already running");
		return ENC_SYM_NIL;
	}

	m_calibration_in_progress = true;
	gpio_set_level(PIN_CHG_EN, 0);

	float offset_v = isense_read_voltage();
	if (!isense_offset_valid(offset_v)) {
		m_calibration_in_progress = false;
		commands_printf_lisp("CAL rejected: current ADC reference is invalid");
		return ENC_SYM_NIL;
	}

	bool protection_ready = isense_apply_offset(offset_v);
	m_current_valid = true;
	m_current_offset_calibrated = true;
	bool stored = isense_store_offset(offset_v);

	m_calibration_in_progress = false;
	commands_printf_lisp("CAL complete: offset=%.4f V stored=%d protection=%d",
			(double)offset_v, stored ? 1 : 0, protection_ready ? 1 : 0);
	if (!stored) {
		commands_printf_lisp("CAL warning: NVS store failed; zero is valid only until reboot");
	}
	if (!protection_ready) {
		commands_printf_lisp("CAL warning: fast current protection is not ready; charging remains off");
	}
	return ENC_SYM_TRUE;
}

// (master-clear-fast-oc) -- clear the persistent hardware overcurrent latch
// only after the charger has been absent continuously and current is near zero.
static lbm_value ext_master_clear_fast_oc(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	jfbms_fast_oc_status_t status;
	jfbms_fast_oc_get_status(&status);
	if (!status.latched) return ENC_SYM_TRUE;

	float charger_detect_v = ((main_config_t *)&backup.config)->v_charge_detect;
	if (!jfbms_fast_oc_clear_allowed(charger_detect_v) ||
			!jfbms_fast_oc_clear_if_unchanged(status.trip_count)) {
		return ENC_SYM_NIL;
	}
	return ENC_SYM_TRUE;
}

// Suspend only the fast comparator callback while COM_EN is powered down for
// deep sleep. The ADC driver, thresholds and any existing fault latch remain.
static lbm_value ext_master_sleep_disarm_fast_oc(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return jfbms_fast_oc_sleep_disarm() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// Rearm after a cancelled sleep. Lisp restores COM_EN and waits for the
// current-reference settling interval before calling this extension.
static lbm_value ext_master_sleep_rearm_fast_oc(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return jfbms_fast_oc_sleep_rearm() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (master-fast-oc-status) --
// (latched armed trip-count raw current trip-time-s direction)
static lbm_value ext_master_fast_oc_status(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	jfbms_fast_oc_status_t status;
	jfbms_fast_oc_get_status(&status);
	lbm_value result = lbm_cons(lbm_enc_i(status.direction), ENC_SYM_NIL);
	result = lbm_cons(lbm_enc_float((float)status.trip_time_us / 1000000.0f), result);
	result = lbm_cons(lbm_enc_float(status.last_current_a), result);
	result = lbm_cons(lbm_enc_u32(status.last_raw), result);
	result = lbm_cons(lbm_enc_u32(status.trip_count), result);
	result = lbm_cons(status.armed ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	result = lbm_cons(status.latched ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	return result;
}

// (master-charger-status) -- (valid detected raw-voltage sample-age-ms)
static lbm_value ext_master_charger_status(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	jfbms_charger_status_t status;
	jfbms_charger_get_status(&status);
	lbm_value result = ENC_SYM_NIL;
	result = lbm_cons(lbm_enc_u32(status.sample_age_ms), result);
	result = lbm_cons(lbm_enc_float(status.voltage_v), result);
	result = lbm_cons(status.detected ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	result = lbm_cons(status.valid ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	return result;
}

// (master-current-calibration) -- (valid offset-v raw-current-a running)
static lbm_value ext_master_current_calibration(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	float voltage = isense_read_voltage();
	bool valid = m_current_offset_calibrated &&
			isense_offset_valid(m_current_offset) &&
			voltage >= 0.0f && isfinite(voltage);
	float current = valid && voltage >= 0.0f && isfinite(voltage) ?
			(voltage - m_current_offset) * ISENSE_SCALE : 0.0f;
	lbm_value result = ENC_SYM_NIL;
	result = lbm_cons(m_calibration_in_progress ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	result = lbm_cons(lbm_enc_float(current), result);
	result = lbm_cons(lbm_enc_float(m_current_offset), result);
	result = lbm_cons(valid ? ENC_SYM_TRUE : ENC_SYM_NIL, result);
	return result;
}

// (master-boot-status) -- (boot-count reset-reason)
static lbm_value ext_master_boot_status(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	lbm_value result = ENC_SYM_NIL;
	result = lbm_cons(lbm_enc_u32(m_previous_reset_reason), result);
	result = lbm_cons(lbm_enc_u32(m_boot_count), result);
	return result;
}

// (master-get-current) — returns EMA-filtered current (A); updated by master-update-vesc-bms
static lbm_value ext_master_get_current(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float((m_current_valid && m_current_offset_calibrated) ?
			m_current_filtered : 0.0f);
}

// (master-get-vchg) — returns EMA-filtered charger voltage (V); updated by master-update-vesc-bms
static lbm_value ext_master_get_vchg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(m_vchg_valid ? m_vchg_filtered : 0.0f);
}

// (master-probe-vchg-off) -- force the charge path open, then return the raw
// charger-port voltage. With CHG_EN on, the pack can back-feed the port sense
// divider and make an unplugged connector look like a connected charger.
// This probe is deliberately fail-closed: it never re-enables CHG_EN.
static lbm_value ext_master_probe_vchg_off(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	gpio_set_level(PIN_CHG_EN, 0);
	vTaskDelay(pdMS_TO_TICKS(25) + 1);
	float v = adc_get_voltage(HW_ADC_CH3);
	if (v < 0.0f || !isfinite(v)) {
		return ENC_SYM_NIL;
	}
	return lbm_enc_float(v * VCHG_DIV_SCALE);
}

// (master-get-temp-pcb) — returns EMA-filtered PCB NTC temp (°C); updated by master-update-vesc-bms
static lbm_value ext_master_get_temp_pcb(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(m_temp_pcb_valid ? m_temp_pcb : -300.0f);
}

static lbm_value ext_master_local_sensors_valid(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return (m_current_valid && m_current_offset_calibrated &&
			m_vchg_valid && m_temp_pcb_valid) ?
			ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_master_local_sensor_status(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	int status = 0;
	if (m_current_valid && m_current_offset_calibrated) status |= 0x01;
	if (m_vchg_valid) status |= 0x02;
	if (m_temp_pcb_valid) status |= 0x04;
	return lbm_enc_i(status);
}

static lbm_value ext_master_get_enable(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(gpio_get_level(PIN_ENABLE) == 0 ? 0 : 1);
}

static void master_set_enable_wakeup_state(int state) {
	gpio_set_direction(PIN_ENABLE, GPIO_MODE_INPUT);

	switch (state) {
	case 0:
		esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_LOW);
		break;

	case 1:
		esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH);
		break;

	default:
		gpio_wakeup_disable_on_hp_periph_powerdown_sleep(PIN_ENABLE);
		break;
	}
}

static lbm_value ext_master_set_enable_wakeup_state(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	master_set_enable_wakeup_state(lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_master_get_time_of_day_s(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	struct timeval now;
	gettimeofday(&now, NULL);
	return lbm_enc_i32(now.tv_sec);
}

static lbm_value ext_master_wakeup_source(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();
	if (wakeup_causes & (BIT(ESP_SLEEP_WAKEUP_EXT0) | BIT(ESP_SLEEP_WAKEUP_GPIO))) {
		return lbm_enc_i(1);
	}
	if (wakeup_causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
		return lbm_enc_i(2);
	}

	return lbm_enc_i(0);
}

static lbm_value ext_master_fail_close_local(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_direction(PIN_CHG_EN, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CHG_EN, 0);
	return ENC_SYM_TRUE;
}

// ============================================================================
// Debug Extensions
// ============================================================================

// (can-debug) - TWAI driver status
static lbm_value ext_can_debug(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	jfbms_fast_oc_status_t fast_oc;
	jfbms_fast_oc_get_status(&fast_oc);
	commands_printf_lisp("Fast OC: armed=%d latched=%d trips=%lu raw=%lu current=%.2fA trip_time_ms=%ld pack_watchdog=%d",
			fast_oc.armed ? 1 : 0,
			fast_oc.latched ? 1 : 0,
			(unsigned long)fast_oc.trip_count,
			(unsigned long)fast_oc.last_raw,
			(double)fast_oc.last_current_a,
			(long)(fast_oc.trip_time_us / 1000),
			m_pack_watchdog_ready ? 1 : 0);
	commands_printf_lisp("ADC: status=0x%02X current_v=%.3f vchg_v=%.3f pcb_v=%.3f pcb_temp=%.1fC",
			(m_current_valid ? 0x01 : 0) |
			(m_vchg_valid ? 0x02 : 0) |
			(m_temp_pcb_valid ? 0x04 : 0),
			(double)hw_adc_get_voltage(HW_ADC_CH2),
			(double)hw_adc_get_voltage(HW_ADC_CH3),
			(double)hw_adc_get_voltage(HW_ADC_CH4),
			(double)(m_temp_pcb_valid ? m_temp_pcb : -300.0f));
	volatile bms_values *ui_bms = bms_get_values();
	uint32_t ui_balance_low = 0;
	int ui_cell_count = ui_bms->cell_num;
	if (ui_cell_count < 0) ui_cell_count = 0;
	if (ui_cell_count > BMS_MAX_CELLS) ui_cell_count = BMS_MAX_CELLS;
	for (int i = 0; i < ui_cell_count && i < 32; i++) {
		if (ui_bms->bal_state[i]) ui_balance_low |= (uint32_t)1U << i;
	}
	commands_printf_lisp("VESC BMS UI: cells=%d balancing=%d balance_low=0x%08lX age=%.3fs",
			ui_cell_count, ui_bms->is_balancing ? 1 : 0,
			(unsigned long)ui_balance_low,
			(double)UTILS_AGE_S(ui_bms->update_time));

	comm_can_debug_info_t can_dbg;
	comm_can_get_debug_info(&can_dbg);
#if JFBMS_USE_DEDICATED_SLAVE_TWAI
	comm_can2_debug_info_t can2_dbg;
	comm_can2_get_debug_info(&can2_dbg);
	uint32_t filter_id = 0;
	uint32_t filter_mask = 0;
	configured_slave_filter(&filter_id, &filter_mask);
#endif
	uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	uint32_t rate_elapsed_ms = debug_rate_last_ms != 0 ?
			(now_ms - debug_rate_last_ms) : now_ms;

	commands_printf_lisp(
		"Primary CAN TWAI0: tx=%d rx=%d recovery=%d tx_retry_eid=%lu tx_drain=%lu tx_fail_eid=%lu tx_fail_sid=%lu fwd_fail=%lu drain_fail=%lu",
		CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM,
		comm_can_get_rx_recovery_cnt(),
		(unsigned long)can_dbg.tx_eid_retry,
		(unsigned long)can_dbg.tx_drain_retry,
		(unsigned long)can_dbg.tx_eid_fail,
		(unsigned long)can_dbg.tx_sid_fail,
		(unsigned long)can_dbg.tx_send_buffer_fail,
		(unsigned long)can_dbg.tx_drain_fail);
	commands_printf_lisp(
		"Primary CAN rx err: overflow=%lu no-buf=%lu crc=%lu bad-len=%lu",
		(unsigned long)can_dbg.rx_overflow,
		(unsigned long)can_dbg.rx_no_buffer,
		(unsigned long)can_dbg.rx_crc_fail,
		(unsigned long)can_dbg.rx_bad_len);

#if JFBMS_USE_DEDICATED_SLAVE_TWAI
	commands_printf_lisp("Slave CAN TWAI%d: tx=%d rx=%d baud=%d running=%d filter_id=0x%03lX filter_mask=0x%03lX rx_total=%lu filtered=%lu malformed=%lu filtered_last=0x%03lX esc_rx=%lu priv_rx=%lu buf_head=%d buf_tail=%d overflow=%lu recovery=%d tx_ok=%lu tx_fail=%lu tx_timeout=%lu last_err=%d",
		JFBMS_SLAVE_CAN_TWAI_ID, JFBMS_SLAVE_CAN_TX_GPIO_NUM, JFBMS_SLAVE_CAN_RX_GPIO_NUM,
		JFBMS_SLAVE_CAN_BAUD_KBITS,
		slave_can_running ? 1 : 0,
		(unsigned long)filter_id, (unsigned long)filter_mask,
		(unsigned long)can_rx_total,
		(unsigned long)can_rx_filtered_total,
		(unsigned long)can_rx_malformed_total,
		(unsigned long)can_rx_filtered_last_id,
		(unsigned long)can_rx_esc_total,
		(unsigned long)can_rx_private_total,
		can_rx_head, can_rx_tail, (unsigned long)can_rx_overflow,
		comm_can2_get_rx_recovery_cnt(),
		(unsigned long)slave_can_tx_ok_cnt,
		(unsigned long)slave_can_tx_fail_cnt,
		(unsigned long)slave_can_tx_timeout_cnt,
		(int)slave_can_last_error);
	commands_printf_lisp("Slave CAN TWAI%d driver: rx_total=%lu rx_overflow=%lu tx_sid_ok=%lu tx_sid_fail=%lu tx_sid_timeout=%lu last_rx=0x%lX last_tx=0x%lX last_err=%d",
		JFBMS_SLAVE_CAN_TWAI_ID,
		(unsigned long)can2_dbg.rx_total,
		(unsigned long)can2_dbg.rx_overflow,
		(unsigned long)can2_dbg.tx_sid_ok,
		(unsigned long)can2_dbg.tx_sid_fail,
		(unsigned long)can2_dbg.tx_sid_timeout,
		(unsigned long)can2_dbg.last_rx_id,
		(unsigned long)can2_dbg.last_tx_sid,
		(int)can2_dbg.last_error);
#else
	commands_printf_lisp("Slave CAN primary TWAI0: tx=%d rx=%d baud_cfg=%d one_bus=1 twai1_disabled_pin_collision=%d rx_total=%lu malformed=%lu primary_rx=%lu dedicated_rx=%lu buf_head=%d buf_tail=%d overflow=%lu tx_ok=%lu tx_fail=%lu tx_timeout=%lu last_err=%d",
		CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM,
		(int)backup.config.can_baud_rate,
		JFBMS_DEDICATED_SLAVE_TWAI_PIN_COLLISION,
		(unsigned long)can_rx_total,
		(unsigned long)can_rx_malformed_total,
		(unsigned long)can_rx_esc_total,
		(unsigned long)can_rx_private_total,
		can_rx_head, can_rx_tail, (unsigned long)can_rx_overflow,
		(unsigned long)slave_can_tx_ok_cnt,
		(unsigned long)slave_can_tx_fail_cnt,
		(unsigned long)slave_can_tx_timeout_cnt,
		(int)slave_can_last_error);
#endif

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int expected_slaves = configured_slave_count();
	for (int i = 0; i < expected_slaves; i++) {
		uint32_t status_delta = m_slave_stage[i].raw_status_count - debug_status_last[i];
		float status_hz = rate_elapsed_ms > 0 ?
				((float)status_delta * 1000.0f / (float)rate_elapsed_ms) : 0.0f;
		uint32_t complete_delta = m_slave_stage[i].complete_count - debug_complete_last[i];
		float complete_hz = rate_elapsed_ms > 0 ?
				((float)complete_delta * 1000.0f / (float)rate_elapsed_ms) : 0.0f;
		int32_t status_age = m_bms_data.status_last_seen_ms[i] != 0 ?
				(int32_t)(now_ms - m_bms_data.status_last_seen_ms[i]) : -1;
		int32_t temp_age = m_bms_data.temp_last_seen_ms[i] != 0 ?
				(int32_t)(now_ms - m_bms_data.temp_last_seen_ms[i]) : -1;
		int32_t stage_age = m_slave_stage[i].in_progress ?
				(int32_t)(now_ms - m_slave_stage[i].start_ms) : -1;

		commands_printf_lisp("Slave %d: active=%d fresh=%d stale_checks=%u frames=%lu raw_status=%lu raw_status_rate=%.2fHz complete=%lu complete_rate=%.2fHz incomplete=%lu timeout=%lu restart=%lu orphan=%lu duplicate=%lu malformed=%lu invalid=%lu bus_mismatch=%lu generation=%lu commit_age=%ldms temp_age=%ldms stage_age=%ldms stage_mask=0x%03X last_missing=0x%03X ic1=%d ic2=%d balance=0x%08lX faults=0x%02X temp_flags=0x%02X temps=%.1f,%.1f,%.1f,%.1f bus=%s",
			i + 1, m_bms_data.active[i] ? 1 : 0,
			m_bms_data.fresh[i] ? 1 : 0,
			(unsigned int)m_bms_data.stale_checks[i],
			(unsigned long)m_bms_data.frame_rx_count[i],
			(unsigned long)m_slave_stage[i].raw_status_count,
			(double)status_hz,
			(unsigned long)m_slave_stage[i].complete_count,
			(double)complete_hz,
			(unsigned long)m_slave_stage[i].incomplete_count,
			(unsigned long)m_slave_stage[i].timeout_count,
			(unsigned long)m_slave_stage[i].restart_count,
			(unsigned long)m_slave_stage[i].orphan_count,
			(unsigned long)m_slave_stage[i].duplicate_count,
			(unsigned long)can_rx_malformed_slave[i],
			(unsigned long)m_slave_stage[i].invalid_count,
			(unsigned long)m_slave_stage[i].bus_mismatch_count,
			(unsigned long)m_slave_stage[i].generation,
			(long)status_age, (long)temp_age, (long)stage_age,
			(unsigned int)m_slave_stage[i].received_mask,
			(unsigned int)m_slave_stage[i].last_missing_mask,
			m_bms_data.cells_ic1[i], m_bms_data.cells_ic2[i],
			(unsigned long)m_bms_data.balance_mask[i],
			m_bms_data.fault_flags[i],
			(unsigned int)m_bms_data.temp_sensor_flags[i],
			(double)(m_bms_data.temperatures[i][0] == 0x7FFF ? -300.0f : (float)m_bms_data.temperatures[i][0] / 10.0f),
			(double)(m_bms_data.temperatures[i][1] == 0x7FFF ? -300.0f : (float)m_bms_data.temperatures[i][1] / 10.0f),
			(double)(m_bms_data.temperatures[i][2] == 0x7FFF ? -300.0f : (float)m_bms_data.temperatures[i][2] / 10.0f),
			(double)(m_bms_data.temperatures[i][3] == 0x7FFF ? -300.0f : (float)m_bms_data.temperatures[i][3] / 10.0f),
			slave_can_rx_bus[i] == SLAVE_CAN_BUS_ESC ? "primary/TWAI0" :
			(slave_can_rx_bus[i] == SLAVE_CAN_BUS_PRIVATE ? "BMS/TWAI1" : "unknown"));
		debug_status_last[i] = m_slave_stage[i].raw_status_count;
		debug_complete_last[i] = m_slave_stage[i].complete_count;
	}
	xSemaphoreGive(m_data_mutex);
	debug_rate_last_ms = now_ms;

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_debug_reset(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	comm_can_reset_debug_info();
	can_rx_overflow = 0;
	can_rx_total = 0;
	can_rx_esc_total = 0;
	can_rx_private_total = 0;
	can_rx_filtered_total = 0;
	can_rx_filtered_last_id = 0;
	can_rx_malformed_total = 0;
	memset((void *)can_rx_malformed_slave, 0, sizeof(can_rx_malformed_slave));
	debug_rate_last_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	memset(debug_status_last, 0, sizeof(debug_status_last));
	memset(debug_complete_last, 0, sizeof(debug_complete_last));
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	for (int i = 0; i < MAX_SLAVES; i++) {
		m_bms_data.frame_rx_count[i] = 0;
		m_bms_data.status_rx_count[i] = 0;
		clear_slave_stage_diagnostics_locked(i);
	}
	xSemaphoreGive(m_data_mutex);
	slave_can_tx_ok_cnt = 0;
	slave_can_tx_fail_cnt = 0;
	slave_can_tx_timeout_cnt = 0;
	slave_can_last_error = ESP_OK;
#ifdef CONFIG_IDF_TARGET_ESP32C6
	comm_can2_reset_debug_info();
#endif

	return ENC_SYM_TRUE;
}

// ============================================================================
// Section E: Extension Registration & hw_init
// ============================================================================

void hw_shutdown(void) {
	commands_printf("Shutdown: disabling outputs, then driving GPIO%d high", PIN_SHUTDOWN);

	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_direction(PIN_COM_EN, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_COM_EN, 1);
	vTaskDelay(pdMS_TO_TICKS(50) + 1);
	gpio_set_level(PIN_SHUTDOWN, 1);
	gpio_set_direction(PIN_SHUTDOWN, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_SHUTDOWN, 1);
}

static void terminal_shutdown(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	hw_shutdown();
}

static lbm_value ext_master_shutdown(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	hw_shutdown();
	return ENC_SYM_TRUE;
}

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
	lbm_add_extension("master-slave-fresh?", ext_master_slave_fresh);
	lbm_add_extension("master-get-slave-settled?", ext_master_get_slave_settled);
	lbm_add_extension("master-pack-charge-fresh?", ext_master_pack_charge_fresh);
	lbm_add_extension("master-get-active-slaves", ext_master_get_active_slaves);
	lbm_add_extension("master-get-cell-count", ext_master_get_cell_count);
	lbm_add_extension("master-get-pack-generation", ext_master_get_pack_generation);
	lbm_add_extension("master-get-cells-ic1", ext_master_get_cells_ic1);
	lbm_add_extension("master-get-cells-ic2", ext_master_get_cells_ic2);

	// Slave control
	lbm_add_extension("master-send-balance", ext_master_send_balance);
	lbm_add_extension("master-stop-balance-sync", ext_master_stop_balance_sync);
	lbm_add_extension("master-balance-request", ext_master_balance_request);
	lbm_add_extension("master-balance-inhibited?", ext_master_balance_inhibited);
	lbm_add_extension("master-check-timeouts", ext_master_check_timeouts);
	lbm_add_extension("master-reset-slaves", ext_master_reset_slaves);

	// VESC BMS display
	lbm_add_extension("master-update-vesc-bms", ext_master_update_vesc_bms);

	// Current sense
	lbm_add_extension("master-calibrate-current", ext_master_calibrate_current);
	lbm_add_extension("master-current-calibration", ext_master_current_calibration);
	lbm_add_extension("master-boot-status", ext_master_boot_status);
	lbm_add_extension("master-clear-fast-oc", ext_master_clear_fast_oc);
	lbm_add_extension("master-sleep-disarm-fast-oc", ext_master_sleep_disarm_fast_oc);
	lbm_add_extension("master-sleep-rearm-fast-oc", ext_master_sleep_rearm_fast_oc);
	lbm_add_extension("master-fast-oc-status", ext_master_fast_oc_status);
	lbm_add_extension("master-charger-status", ext_master_charger_status);
	lbm_add_extension("master-set-chg", ext_master_set_chg);
	lbm_add_extension("master-get-current", ext_master_get_current);
	lbm_add_extension("master-get-vchg", ext_master_get_vchg);
	lbm_add_extension("master-probe-vchg-off", ext_master_probe_vchg_off);
	lbm_add_extension("master-get-temp-pcb", ext_master_get_temp_pcb);
	lbm_add_extension("master-local-sensors-valid?", ext_master_local_sensors_valid);
	lbm_add_extension("master-local-sensor-status", ext_master_local_sensor_status);
	lbm_add_extension("master-get-enable", ext_master_get_enable);
	lbm_add_extension("master-set-enable-wakeup-state", ext_master_set_enable_wakeup_state);
	lbm_add_extension("master-get-time-of-day-s", ext_master_get_time_of_day_s);
	lbm_add_extension("master-wakeup-source", ext_master_wakeup_source);
	lbm_add_extension("master-fail-close-local", ext_master_fail_close_local);
	lbm_add_extension("master-shutdown", ext_master_shutdown);

	// Debug
	lbm_add_extension("can-debug", ext_can_debug);
	lbm_add_extension("can-debug-reset", ext_can_debug_reset);
}

void hw_init(void) {
	if (m_boot_diag_magic != JFBMS_BOOT_DIAG_MAGIC) {
		m_boot_diag_magic = JFBMS_BOOT_DIAG_MAGIC;
		m_boot_count = 0;
	}
	m_boot_count++;
	m_previous_reset_reason = (uint32_t)esp_reset_reason();
	m_data_mutex = xSemaphoreCreateMutexStatic(&m_data_mutex_storage);
	m_balance_tx_mutex = xSemaphoreCreateMutexStatic(&m_balance_tx_mutex_storage);
	if (!jfbms_master_validate_config((const main_config_t *)&backup.config)) {
		main_config_t repaired = backup.config;
		jfbms_master_confparser_set_defaults_main_config_t(&repaired);
		memcpy((uint8_t *)&backup.config + offsetof(main_config_t, batt_ah),
				(uint8_t *)&repaired + offsetof(main_config_t, batt_ah),
				sizeof(main_config_t) - offsetof(main_config_t, batt_ah));
		main_store_backup_data();
		commands_printf("JFBMS invalid safety configuration replaced with defaults");
	}
	m_balance_inhibit = false;
	m_balance_requested = false;
	m_balance_stop_ms = 0;

	// Initialize master slave data
	memset(&m_bms_data, 0, sizeof(m_bms_data));
	m_pack_generation = 0;
	m_pack_watchdog_ready = false;
	for (int s = 0; s < MAX_SLAVES; s++) {
		clear_slave_data(s);
		m_slave_complete_ms[s] = 0;
		m_slave_snapshot_safe[s] = false;
		m_slave_charge_safe[s] = false;
	}

	// GPIO setup
	gpio_config_t gpconf = {0};

	// Push-pull outputs. CHG is active high; COM_EN is active low.
	gpio_hold_dis(PIN_COM_EN);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_COM_EN, 0);

	gpconf.pin_bit_mask = BIT(PIN_CHG_EN) | BIT(PIN_COM_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_COM_EN, 0);

	// External ENABLE wake input on GPIO7. Leave pulls to the board circuit.
	gpconf.pin_bit_mask = BIT(PIN_ENABLE);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// High-Z idle: SHUTDOWN on GPIO19. Only drive push-pull high in hw_shutdown().
	// GPIO8 (buzzer) is driven by the PWM peripheral — not configured here

	gpconf.pin_bit_mask = BIT(PIN_SHUTDOWN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_DISABLE;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// GPIO2/3/4 are ADC inputs; do not configure them as digital I/O.

	terminal_register_command_callback(
			"shutdown",
			"Drive GPIO19 shutdown high",
			0,
			terminal_shutdown);

	// Restore the one-time current zero when available. A new controller uses the
	// safe midpoint internally but displays 0 A until the first charge captures
	// and stores its actual zero. The manual VESC Tool command can replace it.
	{
		bool adc_ok = jfbms_fast_adc_init();
		if (!adc_ok) {
			commands_printf("JFBMS continuous ADC failed; CHG_EN locked off");
		}

		float startup_offset = ISENSE_DEFAULT_OFFSET_V;
		bool stored_offset = isense_load_stored_offset(&startup_offset);
		bool protection_armed = false;
		if (adc_ok) {
			// COM_EN powers 3V3COM and the RC-filtered 1.65 V current
			// reference. Keep CHG_EN fail-closed and leave the raw monitor
			// disarmed until the reference has had time to settle.
			vTaskDelay(pdMS_TO_TICKS(ISENSE_POWER_SETTLE_MS));
			protection_armed = isense_apply_offset(startup_offset);
			// Stored current calibration remains valid independently of the
			// optional fast-OC monitor. CHG_EN checks monitor readiness itself.
			m_current_offset_calibrated = stored_offset;
			if (m_current_offset_calibrated) {
				commands_printf("JFBMS current zero restored: %.4f V",
						(double)m_current_offset);
			} else {
				commands_printf("JFBMS no stored current zero; current reads 0 A until first charge");
			}
		}
		if (adc_ok && !protection_armed) {
			commands_printf("JFBMS current protection not armed; CHG_EN locked off");
		}

		// The midpoint applied the fast-overcurrent settings from backup.config
		// (or left charging safely locked off if ADC setup failed).
		// Remember that snapshot so writing an unrelated setting, such as a cell
		// charge voltage, does not needlessly rebuild the ADC monitor and reject
		// the complete VESC Tool configuration transaction.
		remember_fast_oc_config((const main_config_t *)&backup.config);
	}

#if JFBMS_USE_DEDICATED_SLAVE_TWAI
	bool slave_can_ok = slave_can_start();
	commands_printf("JFBMS slave CAN: dedicated mode SOC_TWAI_CONTROLLER_NUM=%d TWAI%d tx=%d rx=%d baud=%d start=%d err=%d",
			SOC_TWAI_CONTROLLER_NUM, JFBMS_SLAVE_CAN_TWAI_ID,
			JFBMS_SLAVE_CAN_TX_GPIO_NUM, JFBMS_SLAVE_CAN_RX_GPIO_NUM,
			JFBMS_SLAVE_CAN_BAUD_KBITS, slave_can_ok ? 1 : 0,
			(int)slave_can_last_error);
#else
	slave_can_last_error = ESP_OK;
	commands_printf("JFBMS slave CAN: one-bus mode TWAI0 tx=%d rx=%d dedicated_twai=0 pin_collision=%d",
			CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM,
			JFBMS_DEDICATED_SLAVE_TWAI_PIN_COLLISION);
#endif
	m_applied_slave_count = configured_slave_count();

	esp_timer_create_args_t safety_timer_args = {
		.callback = pack_safety_timer_cb,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "jfbms-pack-safe",
		.skip_unhandled_events = true,
	};
	if (esp_timer_create(&safety_timer_args, &m_pack_safety_timer) != ESP_OK ||
			esp_timer_start_periodic(m_pack_safety_timer, 1000) != ESP_OK) {
		gpio_set_level(PIN_CHG_EN, 0);
		commands_printf("JFBMS pack safety watchdog failed; CHG_EN locked off");
	} else {
		m_pack_watchdog_ready = true;
	}

	lispif_add_ext_load_callback(load_extensions);
}

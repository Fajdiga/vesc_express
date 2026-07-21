/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se
	Copyright 2025 JetFleet

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

#ifndef MAIN_HWCONF_JETFLEET_JFBMS_MASTER_H_
#define MAIN_HWCONF_JETFLEET_JFBMS_MASTER_H_

// JFBMS Master hardware: ESP32-C6-MINI-N4 variant. No local BQ76952 — cell data comes
// from slaves over CAN. On-board: shunt + amp current sense, NTC PCB temp,
// charger voltage divider, charge FET drive, buzzer, COM_EN, shutdown.

#include "hal/adc_types.h"
#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#include <stddef.h>

#define HW_NAME						"JFBMS_MASTER"
#define HW_TARGET					"esp32c6_fh4"

#define PCB_VERSION					1

#define HW_NO_UART
#define HW_EARLY_LBM_INIT
#define HW_ADC_CUSTOM_DRIVER
#define HW_INIT_HOOK()				hw_init()
#define HW_SHUTDOWN_HOOK()			hw_shutdown()
#define USER_EXTENSION_STORAGE_SIZE	50
// The Lisp application is stored in the flash image. Keep only enough runtime
// space for controller state and transient multi-slave data.
#define HW_LBM_HEAP_CELLS			8192
#define HW_LBM_MEMORY_KB			96

// CAN: Normal ACK mode for reliable master-slave communication
#define HW_CAN_NO_ACK_MODE			0

// This board is the pack BMS and owns the global VESC BMS value set. Do not
// let received/echoed standard BMS CAN frames replace its locally assembled
// slave snapshot (especially cell_num and v_cell[]).
#define HW_BMS_CAN_VALUES_LOCAL_OWNER

// Configuration overrides
#define OVR_CONF_PARSER_C			"jfbms_master_confparser.c"
#define OVR_CONF_PARSER_H			"jfbms_master_confparser.h"
#define OVR_CONF_XML_C				"jfbms_master_confxml.c"
#define OVR_CONF_XML_H				"jfbms_master_confxml.h"
#define OVR_CONF_DEFAULT			"jfbms_master_conf_default.h"
#define OVR_CONF_SERIALIZE			jfbms_master_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		jfbms_master_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		jfbms_master_confparser_set_defaults_main_config_t
#define OVR_CONF_MIGRATE_LEGACY(signature, config) \
	jfbms_master_migrate_legacy_config((signature), (config))
#define OVR_CONF_VALIDATE(config) \
	jfbms_master_validate_config((config))
#define OVR_CONF_APPLY() \
	jfbms_master_apply_config()
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763463

typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;

	// Cells on first balance IC
	int cells_ic1;

	// Cells on second balance IC
	int cells_ic2;

	// Number of external temperature sensors
	int temp_num;

	// Battery amp hours
	float batt_ah;

	// Maximum simultaneous balancing channels
	int max_bal_ch;

	// Use amp hours for columb counting
	bool soc_use_ah;

	// Block sleep mode
	bool block_sleep;

	// Cell voltage when empty
	float vc_empty;

	// Cell voltage when full
	float vc_full;

	// Start balancing if cell voltage is this much above the minimum cell voltage
	float vc_balance_start;

	// Stop balancing when cell voltage is this much above the minimum cell voltage
	float vc_balance_end;

	// Start charging when max cell voltage is below this voltage
	float vc_charge_start;

	// End charging when max cell voltage is above this voltage
	float vc_charge_end;

	// Only allow charging if all cells are above this voltage
	float vc_charge_min;

	// Only allow balancing if all cells are above this voltage
	float vc_balance_min;

	// Only allow balancing when the current magnitude is below this value
	float balance_max_current;

	// Current must be above this magnitude for the Ah and Wh couters to run
	float min_current_ah_wh_cnt;

	// Enter sleep mode when the current magnitude is below this value
	float min_current_sleep;

	// Charge port voltage at which a charger is considered plugged in
	float v_charge_detect;

	// Only allow charging when the cell temperature is below this value
	float t_charge_max;

	// Only allow charging when the MOSFET temperature is below this value
	float t_charge_max_mos;

	// Sleep time in hours
	float sleep;

	// Shutdown time in days
	int shutdown;

	// Stop charging when the charge current goes below this value
	float min_charge_current;

	// Maximum allowed charging current
	float max_charge_current;

	// Filter constant for SoC filter
	float soc_filter_const;

	// Do not allow balancing above this cell temperature
	float t_bal_max_cell;

	// Do not allow balancing above this balance IC temperature
	float t_bal_max_ic;

	// Only allow charging when the cell temperature is above this value
	float t_charge_min;

	// Enable temperature monitoring during charging
	bool t_charge_mon_en;

	// These fields deliberately reuse the raw-NVS offsets of the obsolete
	// power-switch members. Do not reorder them: deployed settings blobs depend on
	// the ESP32 4-byte ABI offsets documented by the static assertions below.
	float fast_charge_oc_a;
	bool fast_charge_oc_en;
	float charge_confirm_time_s;
	uint8_t config_reserved_0;
	float charge_taper_time_s;
	uint8_t config_reserved_1;

	// --- Master-specific parameters ---

	// Number of expected slave devices (1-8)
	int num_slaves;
} main_config_t;

_Static_assert(sizeof(main_config_t) == 404,
		"JFBMS master config ABI changed; add an explicit NVS migration");
_Static_assert(offsetof(main_config_t, num_slaves) == 400,
		"JFBMS master num_slaves offset changed; legacy NVS would be lost");

#define JFBMS_MASTER_CONFIG_SIGNATURE_LEGACY 1155088901U
bool jfbms_master_migrate_legacy_config(uint32_t signature, main_config_t *conf);
bool jfbms_master_validate_config(const main_config_t *conf);
bool jfbms_master_apply_config(void);

// Default setting Overrides
#define HW_DEFAULT_ID				3

// External VESC CAN bus: primary TWAI0 on GPIO20/21.
#define CAN_TX_GPIO_NUM				20
#define CAN_RX_GPIO_NUM				21

// Private master<->slave BMS CAN bus: TWAI1 on GPIO22/23.
#define JFBMS_WANT_DEDICATED_SLAVE_TWAI	1
#define JFBMS_SLAVE_CAN_TX_GPIO_NUM	22
#define JFBMS_SLAVE_CAN_RX_GPIO_NUM	23
#define JFBMS_SLAVE_CAN_TWAI_ID		1
#define JFBMS_SLAVE_CAN_BAUD_KBITS	500
#define JFBMS_ALLOW_SHARED_SLAVE_CAN_FALLBACK	0

#if JFBMS_WANT_DEDICATED_SLAVE_TWAI && \
	(JFBMS_SLAVE_CAN_TX_GPIO_NUM != CAN_TX_GPIO_NUM) && \
	(JFBMS_SLAVE_CAN_RX_GPIO_NUM != CAN_RX_GPIO_NUM)
#define JFBMS_USE_DEDICATED_SLAVE_TWAI	1
#define JFBMS_DEDICATED_SLAVE_TWAI_PIN_COLLISION 0
#else
#define JFBMS_USE_DEDICATED_SLAVE_TWAI	0
#define JFBMS_DEDICATED_SLAVE_TWAI_PIN_COLLISION \
	(JFBMS_WANT_DEDICATED_SLAVE_TWAI ? 1 : 0)
#endif

// Pins
#define PIN_CHG_EN					5	// Charge FET (active high)
#define PIN_COM_EN					6	// COM enable (active low)
#define PIN_ENABLE					7	// External enable/wake input (active high)
#define PIN_BUZZER					8	// Buzzer (PWM output)
#define PIN_SHUTDOWN				19	// Shutdown drive, high-Z idle, push-pull high when active

// ADC channels
// GPIO2 = current sense amp output (1 mΩ shunt, INA181A3 100× gain, center ~1.65 V)
// GPIO3 = charger voltage divider (300 kΩ : 4.7 kΩ → 64.83×)
// GPIO4 = NTC NCP18XH103F03RB (10 k @ 25 °C, B25/85 = 3434), 10 kΩ pull-up to 3.3 V
#define HW_ADC_CH2					ADC_CHANNEL_2 // Current sense
#define HW_ADC_CH3					ADC_CHANNEL_3 // Charger voltage divider
#define HW_ADC_CH4					ADC_CHANNEL_4 // PCB NTC

// Fast hardware protection is independently configured and capped below the
// ADC full-scale current.
#define JFBMS_FAST_OC_MAX_A		16.0f

float hw_adc_get_voltage(adc_channel_t channel);

// VESC charge-detect helper: read divider scaled back to charger voltage.
// (300 kΩ + 4.7 kΩ) / 4.7 kΩ = 64.83×. Returns -1.0 × 64.83 if ADC unavailable.
#define HW_GET_VCHG()				((adc_get_voltage(ADC_CHANNEL_3) * (300.0e3 + 4.7e3)) / 4.7e3)
#define HW_GET_VOUT()				(0.0)

// Master slave data storage
#define MAX_SLAVES					8
#define CELLS_PER_SLAVE				32
#define TEMPS_PER_SLAVE				4

typedef struct {
	uint16_t cell_voltages[MAX_SLAVES][CELLS_PER_SLAVE];  // mV
	int16_t  temperatures[MAX_SLAVES][TEMPS_PER_SLAVE];   // 0.1 deg C
	uint32_t balance_mask[MAX_SLAVES];
	uint8_t  fault_flags[MAX_SLAVES];
	uint8_t  cells_ic1[MAX_SLAVES];
	uint8_t  cells_ic2[MAX_SLAVES];
	uint8_t  temp_sensor_flags[MAX_SLAVES]; // bit 0: BQ1 ext NTC, bit 1: BQ2 ext NTC
	uint32_t last_seen_ms[MAX_SLAVES];
	uint32_t cell_last_seen_ms[MAX_SLAVES][8];
	uint32_t temp_last_seen_ms[MAX_SLAVES];
	uint32_t status_last_seen_ms[MAX_SLAVES];
	uint32_t frame_rx_count[MAX_SLAVES];
	uint32_t status_rx_count[MAX_SLAVES]; // One status frame completes each broadcast burst
	bool     fresh[MAX_SLAVES];   // All required frames are inside the configured timeout
	bool     active[MAX_SLAVES];
	uint8_t  stale_checks[MAX_SLAVES]; // Consecutive freshness misses before inactive
	bool     settled[MAX_SLAVES];  // Voltage-settled flag from slave (bit 2 of faults byte)
} master_bms_data_t;

// Functions
void hw_init(void);
void hw_shutdown(void);

#endif /* MAIN_HWCONF_JETFLEET_JFBMS_MASTER_H_ */

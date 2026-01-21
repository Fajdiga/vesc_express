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

#ifndef MAIN_HWCONF_VESC_JFBMS_MASTER_H_
#define MAIN_HWCONF_VESC_JFBMS_MASTER_H_

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME						"JFBMS_MASTER"

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_IS_MASTER
#define HW_INIT_HOOK()				hw_init()
#define HW_CAN_PING_SCAN_ENABLED	0  // Disable VESC CAN ping scan - master uses 11-bit protocol
#define HW_CAN_NO_ACK_MODE			0  // Normal ACK mode for reliable communication

// Configuration overrides
#define OVR_CONF_PARSER_C			"jfbms_master_confparser.c"
#define OVR_CONF_PARSER_H			"jfbms_master_confparser.h"
#define OVR_CONF_XML_C				"jfbms_master_confxml.c"
#define OVR_CONF_XML_H				"jfbms_master_confxml.h"
#define OVR_CONF_DEFAULT			"jfbms_master_conf_default.h"
#define OVR_CONF_SERIALIZE			jfbms_master_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		jfbms_master_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		jfbms_master_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763460

// Master-specific constants
#define MAX_SLAVES					8
#define CELLS_PER_SLAVE				32
#define TEMPS_PER_SLAVE				4

// Master BMS data structure - stores aggregated data from all slaves
typedef struct {
	uint16_t cell_voltages[MAX_SLAVES][CELLS_PER_SLAVE];  // mV per cell
	int16_t temperatures[MAX_SLAVES][TEMPS_PER_SLAVE];    // 0.1 deg C
	uint32_t balance_mask[MAX_SLAVES];                     // Current balance state
	uint8_t fault_flags[MAX_SLAVES];                       // Fault flags per slave
	uint8_t cell_count[MAX_SLAVES];                        // Actual cell count per slave
	uint32_t last_seen_ms[MAX_SLAVES];                     // Last message timestamp
	bool active[MAX_SLAVES];                               // Slave is responding
} master_bms_data_t;

typedef struct {
	int num_slaves;          // Number of expected slaves (1-8)
	CAN_BAUD can_baud_rate;  // CAN baud rate (should be 500K per protocol)
	int slave_timeout_ms;    // Timeout for detecting offline slaves (default: 1000ms)

	// Compatibility fields (not used by master, but needed for compilation)
	int controller_id;      // Not used - master aggregates from all slaves
	int can_status_rate_hz; // Not used by master
	WIFI_MODE wifi_mode;    // Not used by master
	char wifi_sta_ssid[36]; // Not used by master
	char wifi_sta_key[26];  // Not used by master
	char wifi_ap_ssid[36];  // Not used by master
	char wifi_ap_key[26];   // Not used by master
	bool use_tcp_local;     // Not used by master
	bool use_tcp_hub;       // Not used by master
	char tcp_hub_url[36];   // Not used by master
	uint16_t tcp_hub_port;  // Not used by master
	char tcp_hub_id[26];    // Not used by master
	char tcp_hub_pass[26];  // Not used by master
	BLE_MODE ble_mode;      // Not used by master
	char ble_name[9];       // Not used by master
	uint32_t ble_pin;       // Not used by master
	uint32_t ble_service_capacity;    // Not used by master
	uint32_t ble_chr_descr_capacity;  // Not used by master
} main_config_t;

// Default setting Overrides
#define HW_DEFAULT_ID				1

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// Parameters
#define HW_R_SHUNT					0.0002

// Functions
void hw_init(void);

// Master data access functions
master_bms_data_t* hw_master_get_data(void);

#endif /* MAIN_HWCONF_VESC_JFBMS_MASTER_H_ */

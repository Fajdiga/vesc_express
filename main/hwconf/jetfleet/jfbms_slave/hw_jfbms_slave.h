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

#ifndef MAIN_HWCONF_VESC_JFBMS_SLAVE_H_
#define MAIN_HWCONF_VESC_JFBMS_SLAVE_H_

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME						"JFBMS_SLAVE"

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_INIT_HOOK()				hw_init()
//#define HW_POST_LISPIF_HOOK()		vTaskDelay(200);

// Configuration overrides
#define OVR_CONF_PARSER_C			"jfbms_slave_confparser.c"
#define OVR_CONF_PARSER_H			"jfbms_slave_confparser.h"
#define OVR_CONF_XML_C				"jfbms_slave_confxml.c"
#define OVR_CONF_XML_H				"jfbms_slave_confxml.h"
#define OVR_CONF_DEFAULT			"jfbms_slave_conf_default.h"
#define OVR_CONF_SERIALIZE			jfbms_slave_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		jfbms_slave_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		jfbms_slave_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763459

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

	// Cells on balance IC
	int cells_ic1;
	int cells_ic2;

	// Number of external temperature sensors
	int temp_num;
} main_config_t;

// Default setting Overrides
#define HW_DEFAULT_ID				3

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// I2C pins
#define PIN_SDA						21
#define PIN_SCL						20

// BQ communication enable pins (active LOW, cannot both be LOW at same time)
#define PIN_BQ1_EN					0	// Pull LOW to enable BQ1 communication
#define PIN_BQ2_EN					1	// Pull LOW to enable BQ2 communication

// Buzzer
#define PIN_BUZZER					3

// Parameters
#define HW_R_SHUNT					0.0002

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_JFBMS_SLAVE_H_ */

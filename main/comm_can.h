/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_COMM_CAN_H_
#define MAIN_COMM_CAN_H_

#include "datatypes.h"
#include "esp_err.h"

#define CAN_STATUS_MSGS_TO_STORE	10

typedef struct {
	uint32_t tx_process_short;
	uint32_t tx_fill_rx;
	uint32_t tx_fill_rx_long;
	uint32_t tx_process_rx;
	uint32_t tx_eid_retry;
	uint32_t tx_drain_retry;
	uint32_t tx_send_buffer_fail;
	uint32_t tx_eid_fail;
	uint32_t tx_sid_fail;
	uint32_t tx_drain_fail;
	uint32_t rx_fill_rx;
	uint32_t rx_fill_rx_long;
	uint32_t rx_process_short;
	uint32_t rx_process_rx;
	uint32_t rx_forward_reply_short;
	uint32_t rx_forward_reply_rx;
	uint32_t rx_forward_request_short;
	uint32_t rx_forward_request_rx;
	uint32_t rx_overflow;
	uint32_t rx_no_buffer;
	uint32_t rx_crc_fail;
	uint32_t rx_bad_len;
	uint32_t last_rx_eid;
	uint16_t last_tx_len;
	uint16_t last_rx_len;
	uint8_t last_tx_dst_id;
	uint8_t last_tx_src_id;
	uint8_t last_tx_send;
	uint8_t last_rx_id;
	uint8_t last_rx_packet;
	uint8_t last_rx_last_id;
	uint8_t last_rx_send;
} comm_can_debug_info_t;

typedef struct {
	uint32_t tx_eid_ok;
	uint32_t tx_sid_ok;
	uint32_t tx_eid_fail;
	uint32_t tx_sid_fail;
	uint32_t tx_eid_timeout;
	uint32_t tx_sid_timeout;
	uint32_t rx_total;
	uint32_t rx_overflow;
	uint32_t last_tx_eid;
	uint32_t last_tx_sid;
	uint32_t last_rx_id;
	uint16_t last_tx_len;
	uint16_t last_rx_len;
	uint8_t last_rx_ext;
	esp_err_t last_error;
} comm_can2_debug_info_t;

// Functions
void comm_can_start(int pin_tx, int pin_rx);
void comm_can_stop(void);
bool comm_can_has_listener(void);
int comm_can_get_rx_recovery_cnt(void);
void comm_can_get_debug_info(comm_can_debug_info_t *info);
void comm_can_reset_debug_info(void);
bool comm_can_send_buffer_recent(int msec);
void comm_can_use_vesc_decoder(bool use_vesc_dec);
CAN_BAUD comm_can_kbits_to_baud(int kbits);
void comm_can_update_baudrate(int delay_msec);
void comm_can_change_pins(int tx, int rx);
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
esp_err_t comm_can_transmit_sid_sync(uint32_t id, const uint8_t *data,
		uint8_t len, int timeout_ms);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);
// Like comm_can_ping(), but also reports whether the CAN frame completed on
// the wire. A false ping with tx_ok=true means that the bus is alive but the
// addressed node did not answer the VESC ping. tx_ok=false means that the
// controller could not complete the frame (for example, an empty bus with no
// ACK), so an ID scanner should stop instead of driving the controller bus-off.
bool comm_can_ping_ex(uint8_t controller_id, HW_TYPE *hw_type, bool *tx_ok);

void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);
void comm_can_send_update_baud(int kbits, int delay_msec);

can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_index(int index);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
can_status_msg_3 *comm_can_get_status_msg_3_index(int index);
can_status_msg_3 *comm_can_get_status_msg_3_id(int id);
can_status_msg_4 *comm_can_get_status_msg_4_index(int index);
can_status_msg_4 *comm_can_get_status_msg_4_id(int id);
can_status_msg_5 *comm_can_get_status_msg_5_index(int index);
can_status_msg_5 *comm_can_get_status_msg_5_id(int id);
can_status_msg_6 *comm_can_get_status_msg_6_index(int index);
can_status_msg_6 *comm_can_get_status_msg_6_id(int id);

io_board_adc_values *comm_can_get_io_board_adc_1_4_index(int index);
io_board_adc_values *comm_can_get_io_board_adc_1_4_id(int id);
io_board_adc_values *comm_can_get_io_board_adc_5_8_index(int index);
io_board_adc_values *comm_can_get_io_board_adc_5_8_id(int id);
io_board_digial_inputs *comm_can_get_io_board_digital_in_index(int index);
io_board_digial_inputs *comm_can_get_io_board_digital_in_id(int id);
void comm_can_io_board_set_output_digital(int id, int channel, bool on);
void comm_can_io_board_set_output_pwm(int id, int channel, float duty);

psw_status *comm_can_get_psw_status_index(int index);
psw_status *comm_can_get_psw_status_id(int id);
void comm_can_psw_switch(int id, bool is_on, bool plot);
void comm_can_update_pid_pos_offset(int id, float angle_now, bool store);

#ifdef CONFIG_IDF_TARGET_ESP32C6
void comm_can2_start(int pin_tx, int pin_rx, int baud_kbits);
void comm_can2_set_mask_filter(uint32_t id, uint32_t mask, bool is_ext);
void comm_can2_stop(void);
void comm_can2_use_vesc_decoder(bool use);
bool comm_can2_is_running(void);
int  comm_can2_get_rx_recovery_cnt(void);
void comm_can2_get_debug_info(comm_can2_debug_info_t *info);
void comm_can2_reset_debug_info(void);
esp_err_t comm_can2_transmit_eid_result(uint32_t id, const uint8_t *data, uint8_t len);
esp_err_t comm_can2_transmit_sid_result(uint32_t id, const uint8_t *data, uint8_t len);
esp_err_t comm_can2_transmit_sid_sync(uint32_t id, const uint8_t *data,
		uint8_t len, int timeout_ms);
void comm_can2_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can2_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can2_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send_type);
#endif

#endif /* MAIN_COMM_CAN_H_ */

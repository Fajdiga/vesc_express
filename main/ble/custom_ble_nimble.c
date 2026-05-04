/*
	NimBLE port of custom_ble.c — STAGE 3 STUB.

	Stage 2 ships only the VESC Tool BLE channel (comm_ble_nimble.c). This
	file exists so the linker can resolve every symbol declared in
	custom_ble.h on C6 builds; every entry point returns a safe
	"not started" / "internal error" result.

	JFBMS_MASTER_C6 uses BLE_MODE_OPEN or BLE_MODE_ENCRYPTED (handled by
	comm_ble_nimble.c), never BLE_MODE_SCRIPTING — so this stub is never
	hit at runtime in the current product. LispBM ble-* extensions called
	from a future C6 script will get CUSTOM_BLE_NOT_STARTED until
	custom_ble_nimble.c is fully implemented.
*/

#include "custom_ble.h"

#include <stdbool.h>
#include <string.h>

esp_ble_adv_params_t ble_adv_params = {
	.adv_int_min       = 0x20,
	.adv_int_max       = 0x40,
	.adv_type          = ADV_TYPE_IND,
	.own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
	.channel_map       = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void custom_ble_init(void) {
	// no-op until Stage 3
}

custom_ble_result_t custom_ble_start(void) {
	return CUSTOM_BLE_INIT_FAILED;
}

bool custom_ble_started(void) {
	return false;
}

custom_ble_result_t custom_ble_set_name(const char *name) {
	(void)name;
	return CUSTOM_BLE_INIT_FAILED;
}

custom_ble_result_t custom_ble_update_adv(
	bool use_raw, size_t adv_len, const uint8_t adv_data_raw[adv_len],
	size_t scan_rsp_len, const uint8_t scan_rsp_data_raw[scan_rsp_len]) {
	(void)use_raw; (void)adv_len; (void)adv_data_raw;
	(void)scan_rsp_len; (void)scan_rsp_data_raw;
	return CUSTOM_BLE_INIT_FAILED;
}

void custom_ble_set_attr_write_handler(attr_write_cb_t callback) {
	(void)callback;
}

custom_ble_result_t custom_ble_add_service(
	esp_bt_uuid_t service_uuid, uint16_t chr_count,
	const ble_chr_definition_t chr[chr_count],
	service_handles_cb_t handles_cb) {
	(void)service_uuid; (void)chr_count; (void)chr; (void)handles_cb;
	return CUSTOM_BLE_NOT_STARTED;
}

custom_ble_result_t custom_ble_remove_service(uint16_t service_handle) {
	(void)service_handle;
	return CUSTOM_BLE_NOT_STARTED;
}

custom_ble_result_t custom_ble_get_attr_value(
	uint16_t attr_handle, uint16_t *length, const uint8_t **value) {
	(void)attr_handle; (void)length; (void)value;
	return CUSTOM_BLE_NOT_STARTED;
}

custom_ble_result_t custom_ble_set_attr_value(
	uint16_t attr_handle, uint16_t length, const uint8_t value[length]) {
	(void)attr_handle; (void)length; (void)value;
	return CUSTOM_BLE_NOT_STARTED;
}

uint16_t custom_ble_service_count(void) {
	return 0;
}

uint16_t custom_ble_get_services(uint16_t capacity, uint16_t service_handles[capacity]) {
	(void)capacity; (void)service_handles;
	return 0;
}

int16_t custom_ble_attr_count(uint16_t service_handle) {
	(void)service_handle;
	return -1;
}

custom_ble_result_t custom_ble_get_attrs(
	uint16_t service_handle, uint16_t capacity,
	uint16_t service_handles[capacity], uint16_t *written_count) {
	(void)service_handle; (void)capacity; (void)service_handles;
	if (written_count) *written_count = 0;
	return CUSTOM_BLE_NOT_STARTED;
}

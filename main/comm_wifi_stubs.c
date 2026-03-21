/*
 * Weak stubs for WiFi/BLE functions when building slave/master firmware
 * without WiFi/BLE support. These get overridden by the real implementations
 * when comm_wifi.c / comm_ble.c are compiled.
 */

#include "esp_netif.h"
#include "esp_event_base.h"
#include <stdbool.h>
#include <stdint.h>

// comm_wifi stubs
__attribute__((weak)) void comm_wifi_init(void) {}
__attribute__((weak)) esp_ip4_addr_t comm_wifi_get_ip(void) { esp_ip4_addr_t ip = {0}; return ip; }
__attribute__((weak)) esp_ip4_addr_t comm_wifi_get_ip_client(void) { esp_ip4_addr_t ip = {0}; return ip; }
__attribute__((weak)) bool comm_wifi_is_client_connected(void) { return false; }
__attribute__((weak)) bool comm_wifi_is_connected_hub(void) { return false; }
__attribute__((weak)) bool comm_wifi_is_connecting(void) { return false; }
__attribute__((weak)) bool comm_wifi_is_connected(void) { return false; }
__attribute__((weak)) void comm_wifi_disconnect(void) {}
__attribute__((weak)) void comm_wifi_send_packet_local(unsigned char *data, unsigned int len) { (void)data; (void)len; }
__attribute__((weak)) void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len) { (void)data; (void)len; }
__attribute__((weak)) void comm_wifi_send_raw_local(unsigned char *buffer, unsigned int len) { (void)buffer; (void)len; }
__attribute__((weak)) void comm_wifi_send_raw_hub(unsigned char *buffer, unsigned int len) { (void)buffer; (void)len; }
__attribute__((weak)) void comm_wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	(void)arg; (void)event_base; (void)event_id; (void)event_data;
}

// comm_ble stubs
__attribute__((weak)) void comm_ble_init(void) {}
__attribute__((weak)) bool comm_ble_is_connected(void) { return false; }
__attribute__((weak)) int comm_ble_mtu_now(void) { return 0; }
__attribute__((weak)) void comm_ble_send_packet(unsigned char *data, unsigned int len) { (void)data; (void)len; }
__attribute__((weak)) void custom_ble_init(void) {}
__attribute__((weak)) bool custom_ble_started(void) { return false; }

// WiFi/BLE/RGBLED extension loader stubs
__attribute__((weak)) void lispif_load_wifi_extensions(void) {}
__attribute__((weak)) void lispif_load_ble_extensions(void) {}
__attribute__((weak)) void lispif_load_rgbled_extensions(void) {}

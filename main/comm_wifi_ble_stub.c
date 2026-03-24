// Stub implementations for WiFi/BLE symbols not needed in JFBMS slave/master builds.
// Satisfies linker references in upstream files (lispif_vesc_extensions, commands, terminal).

#include <stdbool.h>
#include "esp_event.h"
#include "esp_netif.h"

bool comm_ble_is_connected(void)                                  { return false; }
int  comm_ble_mtu_now(void)                                       { return 0; }
void comm_ble_send_packet(unsigned char *data, unsigned int len)  { (void)data; (void)len; }
bool custom_ble_started(void)                                     { return false; }
void lispif_load_ble_extensions(void)                             {}

bool comm_wifi_is_connected_hub(void)                             { return false; }
bool comm_wifi_is_client_connected(void)                          { return false; }
bool comm_wifi_is_connected(void)                                 { return false; }
bool comm_wifi_is_connecting(void)                                { return false; }
void comm_wifi_disconnect(void)                                   {}
void comm_wifi_send_packet_local(unsigned char *data, unsigned int len) { (void)data; (void)len; }
void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len)   { (void)data; (void)len; }
void comm_wifi_send_raw_local(unsigned char *buf, unsigned int len)     { (void)buf; (void)len; }
void comm_wifi_send_raw_hub(unsigned char *buf, unsigned int len)       { (void)buf; (void)len; }
esp_ip4_addr_t comm_wifi_get_ip(void)        { esp_ip4_addr_t a = {0}; return a; }
esp_ip4_addr_t comm_wifi_get_ip_client(void) { esp_ip4_addr_t a = {0}; return a; }
void lispif_load_wifi_extensions(void)                            {}
void lispif_load_rgbled_extensions(void)                          {}

void comm_wifi_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
    (void)arg; (void)event_base; (void)event_id; (void)event_data;
}

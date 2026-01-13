// JFBMS Master default configuration

#ifndef JFBMS_MASTER_CONF_DEFAULT_H_
#define JFBMS_MASTER_CONF_DEFAULT_H_

// Number of expected slaves (1-8)
#ifndef CONF_NUM_SLAVES
#define CONF_NUM_SLAVES 1
#endif

// CAN Baud Rate (2 = 500K per protocol spec)
#ifndef CONF_CAN_BAUD_RATE
#define CONF_CAN_BAUD_RATE 2
#endif

// Slave timeout in ms (default 1000ms = 1 second)
#ifndef CONF_SLAVE_TIMEOUT_MS
#define CONF_SLAVE_TIMEOUT_MS 1000
#endif

// Controller ID (compatibility field, not used)
#ifndef CONF_CONTROLLER_ID
#define CONF_CONTROLLER_ID 0
#endif

// CAN Status Rate Hz - Set to 0 to disable VESC protocol CAN broadcasts
// JFBMS Master only uses the custom 11-bit master-slave protocol via LispBM
#ifndef CONF_CAN_STATUS_RATE_HZ
#define CONF_CAN_STATUS_RATE_HZ 0
#endif

// WiFi/BLE compatibility fields (not used by master)
#ifndef CONF_WIFI_MODE
#define CONF_WIFI_MODE 0  // WIFI_MODE_DISABLED
#endif

#ifndef CONF_WIFI_STA_SSID
#define CONF_WIFI_STA_SSID ""
#endif

#ifndef CONF_WIFI_STA_KEY
#define CONF_WIFI_STA_KEY ""
#endif

#ifndef CONF_WIFI_AP_SSID
#define CONF_WIFI_AP_SSID "JFBMS_MASTER"
#endif

#ifndef CONF_WIFI_AP_KEY
#define CONF_WIFI_AP_KEY ""
#endif

#ifndef CONF_USE_TCP_LOCAL
#define CONF_USE_TCP_LOCAL false
#endif

#ifndef CONF_USE_TCP_hub
#define CONF_USE_TCP_hub false
#endif

#ifndef CONF_TCP_HUB_URL
#define CONF_TCP_HUB_URL ""
#endif

#ifndef CONF_TCP_HUB_PORT
#define CONF_TCP_HUB_PORT 0
#endif

#ifndef CONF_TCP_HUB_ID
#define CONF_TCP_HUB_ID ""
#endif

#ifndef CONF_TCP_HUB_PASS
#define CONF_TCP_HUB_PASS ""
#endif

#ifndef CONF_BLE_MODE
#define CONF_BLE_MODE 0  // BLE_MODE_DISABLED
#endif

#ifndef CONF_BLE_NAME
#define CONF_BLE_NAME "JFBMS_MA"
#endif

#ifndef CONF_BLE_PIN
#define CONF_BLE_PIN 123456
#endif

#ifndef CONF_BLE_SERVICE_CAPACITY
#define CONF_BLE_SERVICE_CAPACITY 0
#endif

#ifndef CONF_BLE_CHR_DESCR_CAPACITY
#define CONF_BLE_CHR_DESCR_CAPACITY 0
#endif

// JFBMS_MASTER_CONF_DEFAULT_H_
#endif

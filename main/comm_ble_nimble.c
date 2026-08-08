/*
	NimBLE implementation of the VESC Tool BLE channel.

	The GATT layout matches the Bluedroid version so VESC Tool sees the same
	Nordic UART-style service and RX / TX characteristics.
*/

#include "comm_ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "packet.h"
#include "commands.h"
#include "conf_general.h"
#include "main.h"

#define DEFAULT_BLE_MTU         20 // 23 - 3 ATT header
#define MAX_BLE_PAYLOAD         255
#define BLE_NOTIFY_MAX_RETRIES  8
#define BLE_NOTIFY_RETRY_MS     5
#define BLE_NOTIFY_CHUNK_GAP_MS 3

// Hardware profiles opt in to the connection leases and advertising watchdog.
#ifndef HW_BLE_QUALIFY_TIMEOUT_MS
#define HW_BLE_QUALIFY_TIMEOUT_MS 0U
#endif
#ifndef HW_BLE_IDLE_TIMEOUT_MS
#define HW_BLE_IDLE_TIMEOUT_MS 0U
#endif
#ifndef HW_BLE_ADV_WATCHDOG_MS
#define HW_BLE_ADV_WATCHDOG_MS 0U
#endif

#define BLE_SUPERVISOR_POLL_MS  1000U
#define BLE_TERMINATE_GRACE_MS  5000U
#define BLE_TERMINATE_RETRY_MS  1000U

#ifndef HW_BLE_PWR_LVL
#if CONFIG_IDF_TARGET_ESP32C6
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P9
#else
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P18
#endif
#endif
#ifndef HW_BLE_PWR_LVL_DEFAULT
#define HW_BLE_PWR_LVL_DEFAULT HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_ADV
#define HW_BLE_PWR_LVL_ADV HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_SCAN
#define HW_BLE_PWR_LVL_SCAN HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_CONN
#define HW_BLE_PWR_LVL_CONN HW_BLE_PWR_LVL
#endif

void ble_store_config_init(void);

typedef enum {
	BLE_LINK_DOWN,
	BLE_LINK_QUALIFYING,
	BLE_LINK_ACTIVE,
	BLE_LINK_TERMINATING,
} ble_link_phase_t;

typedef struct {
	bool connected;
	bool subscribed;
	bool terminating;
	uint16_t conn_handle;
	uint16_t mtu;
	uint32_t generation;
} ble_link_snapshot_t;

static const char *TAG = "comm_ble";
static bool host_synced             = false;
static bool identity_ready          = false;
static bool supervisor_initialized  = false;
static bool notify_subscribed       = false;
static bool valid_packet_seen       = false;
static bool reset_pending           = false;
static uint16_t ble_current_mtu      = DEFAULT_BLE_MTU;
static uint16_t conn_handle          = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_char_val_handle  = 0;
static uint8_t own_addr_type        = 0;
static uint32_t connection_started_ms = 0;
static uint32_t last_valid_packet_ms  = 0;
static uint32_t termination_started_ms = 0;
static uint32_t last_terminate_try_ms  = 0;
static uint32_t connection_generation  = 0;
static ble_link_phase_t link_phase     = BLE_LINK_DOWN;
static struct ble_npl_callout supervisor_callout;
static portMUX_TYPE link_mux = portMUX_INITIALIZER_UNLOCKED;
static ble_link_snapshot_t link_snapshot = {
	.conn_handle = BLE_HS_CONN_HANDLE_NONE,
	.mtu = DEFAULT_BLE_MTU,
};
static SemaphoreHandle_t tx_mutex = NULL;
static StaticSemaphore_t tx_mutex_storage;
static PACKET_STATE_t *packet_state = NULL;

/*
 * Service / characteristic UUIDs. Layout (LE byte order) is identical to
 * the Bluedroid build in comm_ble.c.
 *
 *   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E   (write / write-no-rsp)
 *   TX char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E   (read / notify)
 */
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x01, 0x00, 0x40, 0x6E
);

static const ble_uuid128_t rx_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x02, 0x00, 0x40, 0x6E
);

static const ble_uuid128_t tx_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x03, 0x00, 0x40, 0x6E
);

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void supervisor_cb(struct ble_npl_event *event);

static struct ble_gatt_chr_def gatt_chrs[] = {
	{
		.uuid      = &rx_uuid.u,
		.access_cb = gatt_access_cb,
		.flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
	},
	{
		.uuid       = &tx_uuid.u,
		.access_cb  = gatt_access_cb,
		.flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.val_handle = &tx_char_val_handle,
	},
	{0},
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
	{
		.type            = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid            = &svc_uuid.u,
		.characteristics = gatt_chrs,
	},
	{0},
};

static uint32_t now_ms(void) {
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static bool timeout_expired(uint32_t now, uint32_t since, uint32_t timeout) {
	return timeout != 0U && (uint32_t)(now - since) >= timeout;
}

static void publish_link_state(void) {
	portENTER_CRITICAL(&link_mux);
	link_snapshot.connected = link_phase == BLE_LINK_QUALIFYING
		|| link_phase == BLE_LINK_ACTIVE;
	link_snapshot.subscribed = link_snapshot.connected && notify_subscribed;
	link_snapshot.terminating = link_phase == BLE_LINK_TERMINATING;
	link_snapshot.conn_handle = conn_handle;
	link_snapshot.mtu = ble_current_mtu;
	link_snapshot.generation = connection_generation;
	portEXIT_CRITICAL(&link_mux);
}

static ble_link_snapshot_t get_link_state(void) {
	ble_link_snapshot_t state;
	portENTER_CRITICAL(&link_mux);
	state = link_snapshot;
	portEXIT_CRITICAL(&link_mux);
	return state;
}

static void schedule_supervisor(uint32_t delay_ms) {
	if (!supervisor_initialized) {
		return;
	}

	int rc = ble_npl_callout_reset(
		&supervisor_callout, ble_npl_time_ms_to_ticks32(delay_ms)
	);
	if (rc != 0) {
		ESP_LOGE(TAG, "BLE supervisor schedule failed: %d", rc);
	}
}

static void stop_supervisor(void) {
	if (supervisor_initialized) {
		ble_npl_callout_stop(&supervisor_callout);
	}
}

static void reset_packet_parser(void) {
	if (packet_state) {
		packet_reset(packet_state);
	}
}

static void clear_connection(void) {
	link_phase = BLE_LINK_DOWN;
	conn_handle = BLE_HS_CONN_HANDLE_NONE;
	ble_current_mtu = DEFAULT_BLE_MTU;
	notify_subscribed = false;
	valid_packet_seen = false;
	reset_pending = false;
	connection_started_ms = 0;
	last_valid_packet_ms = 0;
	termination_started_ms = 0;
	last_terminate_try_ms = 0;
	reset_packet_parser();
	publish_link_state();
	LED_BLUE_OFF();
}

static void schedule_connection_supervisor(void) {
	bool needed = link_phase == BLE_LINK_TERMINATING
		|| (link_phase == BLE_LINK_QUALIFYING
			&& HW_BLE_QUALIFY_TIMEOUT_MS != 0U)
		|| (link_phase == BLE_LINK_ACTIVE && HW_BLE_IDLE_TIMEOUT_MS != 0U);
	if (needed) {
		schedule_supervisor(BLE_SUPERVISOR_POLL_MS);
	} else {
		stop_supervisor();
	}
}

static void qualify_connection_if_ready(void) {
	if (link_phase == BLE_LINK_QUALIFYING && notify_subscribed
		&& valid_packet_seen) {
		link_phase = BLE_LINK_ACTIVE;
	}
	publish_link_state();
	schedule_connection_supervisor();
}

static void apply_tx_power(void) {
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, HW_BLE_PWR_LVL_ADV);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, HW_BLE_PWR_LVL_SCAN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, HW_BLE_PWR_LVL_DEFAULT);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, HW_BLE_PWR_LVL_CONN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, HW_BLE_PWR_LVL_CONN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, HW_BLE_PWR_LVL_CONN);
}

static uint8_t ble_name_len(void) {
	uint8_t len = 0;
	while (len < (sizeof(backup.config.ble_name) - 1)
		   && backup.config.ble_name[len] != '\0') {
		len++;
	}
	return len;
}

static int start_advertising(void) {
	struct ble_gap_adv_params adv_params = {0};
	struct ble_hs_adv_fields adv_fields  = {0};
	struct ble_hs_adv_fields rsp_fields  = {0};

	const char *name = (const char *)backup.config.ble_name;
	uint8_t name_len = ble_name_len();

	// Match the Bluedroid advertisement layout. The stored BLE name is capped
	// to 8 display bytes, so flags + complete VESC UART UUID + name fits in
	// the 31-byte legacy advertisement.
	adv_fields.flags        = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	adv_fields.uuids128     = (ble_uuid128_t *)&svc_uuid;
	adv_fields.num_uuids128 = 1;
	adv_fields.uuids128_is_complete = 1;
	adv_fields.name                  = (uint8_t *)name;
	adv_fields.name_len              = name_len;
	adv_fields.name_is_complete      = 1;

	int rc = ble_gap_adv_set_fields(&adv_fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "Advertising data setup failed: %d", rc);
		return rc;
	}

	rsp_fields.tx_pwr_lvl_is_present = 1;
	rsp_fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
	rsp_fields.name                  = (uint8_t *)name;
	rsp_fields.name_len              = name_len;
	rsp_fields.name_is_complete      = 1;
	rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "Scan response setup failed: %d", rc);
		return rc;
	}

	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	adv_params.itvl_min  = 0x20;
	adv_params.itvl_max  = 0x40;

	rc = ble_gap_adv_start(
		own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL
	);
	if (rc != 0) {
		ESP_LOGE(TAG, "Advertising start failed: %d", rc);
	}
	return rc;
}

static void ensure_advertising(void) {
	if (!host_synced || link_phase != BLE_LINK_DOWN) {
		return;
	}

	if (!identity_ready) {
		int rc = ble_hs_util_ensure_addr(0);
		if (rc == 0) {
			rc = ble_hs_id_infer_auto(0, &own_addr_type);
		}
		if (rc != 0) {
			ESP_LOGE(TAG, "BLE identity setup failed: %d", rc);
			schedule_supervisor(BLE_SUPERVISOR_POLL_MS);
			return;
		}
		identity_ready = true;
	}

	if (!ble_gap_adv_active() && start_advertising() != 0) {
		schedule_supervisor(BLE_SUPERVISOR_POLL_MS);
		return;
	}

	if (HW_BLE_ADV_WATCHDOG_MS != 0U) {
		schedule_supervisor(HW_BLE_ADV_WATCHDOG_MS);
	} else {
		stop_supervisor();
	}
}

static void begin_termination(const char *reason) {
	if (link_phase == BLE_LINK_DOWN || link_phase == BLE_LINK_TERMINATING) {
		return;
	}

	link_phase = BLE_LINK_TERMINATING;
	termination_started_ms = now_ms();
	last_terminate_try_ms = termination_started_ms;
	reset_packet_parser();
	publish_link_state();
	ESP_LOGW(TAG, "Terminating BLE connection: %s", reason);

	int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
	if (rc == BLE_HS_ENOTCONN) {
		clear_connection();
		ensure_advertising();
		return;
	}
	if (rc != 0) {
		ESP_LOGW(TAG, "BLE terminate request failed: %d", rc);
	}
	schedule_supervisor(BLE_SUPERVISOR_POLL_MS);
}

static void supervisor_cb(struct ble_npl_event *event) {
	(void)event;

	if (!host_synced) {
		return;
	}
	if (link_phase == BLE_LINK_DOWN) {
		ensure_advertising();
		return;
	}

	uint32_t now = now_ms();
	if (link_phase == BLE_LINK_QUALIFYING
		&& timeout_expired(now, connection_started_ms,
			HW_BLE_QUALIFY_TIMEOUT_MS)) {
		begin_termination("qualification timeout");
		return;
	}
	if (link_phase == BLE_LINK_ACTIVE
		&& timeout_expired(now, last_valid_packet_ms, HW_BLE_IDLE_TIMEOUT_MS)) {
		begin_termination("protocol idle timeout");
		return;
	}
	if (link_phase == BLE_LINK_TERMINATING) {
		if (timeout_expired(now, termination_started_ms,
			BLE_TERMINATE_GRACE_MS)) {
			struct ble_gap_conn_desc desc;
			if (ble_gap_conn_find(conn_handle, &desc) == BLE_HS_ENOTCONN) {
				clear_connection();
				ensure_advertising();
				return;
			}
			if (!reset_pending) {
				reset_pending = true;
				ESP_LOGE(TAG, "BLE disconnect stuck; resetting NimBLE host");
				ble_hs_sched_reset(BLE_HS_ECONTROLLER);
			}
		} else if (timeout_expired(now, last_terminate_try_ms,
			BLE_TERMINATE_RETRY_MS)) {
			last_terminate_try_ms = now;
			int rc = ble_gap_terminate(
				conn_handle, BLE_ERR_REM_USER_CONN_TERM
			);
			if (rc == BLE_HS_ENOTCONN) {
				clear_connection();
				ensure_advertising();
				return;
			}
		}
	}

	schedule_supervisor(BLE_SUPERVISOR_POLL_MS);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
	(void)arg;

	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status == 0) {
				stop_supervisor();
				conn_handle = event->connect.conn_handle;
				ble_current_mtu = DEFAULT_BLE_MTU;
				notify_subscribed = false;
				valid_packet_seen = false;
				reset_pending = false;
				connection_started_ms = now_ms();
				last_valid_packet_ms = 0;
				connection_generation++;
				link_phase = BLE_LINK_QUALIFYING;
				reset_packet_parser();
				publish_link_state();
				LED_BLUE_ON();
				apply_tx_power();

				if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
					int rc = ble_gap_security_initiate(conn_handle);
					if (rc != 0) {
						ESP_LOGE(TAG, "BLE security start failed: %d", rc);
						begin_termination("security start failed");
						return 0;
					}
				}
				schedule_connection_supervisor();
			} else {
				ensure_advertising();
			}
			return 0;

		case BLE_GAP_EVENT_DISCONNECT:
			if (conn_handle == BLE_HS_CONN_HANDLE_NONE
				|| event->disconnect.conn.conn_handle == conn_handle) {
				clear_connection();
			}
			ensure_advertising();
			return 0;

		case BLE_GAP_EVENT_MTU:
			if (event->mtu.conn_handle != conn_handle
				|| link_phase == BLE_LINK_DOWN) {
				return 0;
			}
			if (event->mtu.value <= 3) {
				ble_current_mtu = DEFAULT_BLE_MTU;
			} else {
				uint16_t payload_mtu = event->mtu.value - 3; // strip ATT header
				ble_current_mtu      = payload_mtu > MAX_BLE_PAYLOAD
						 ? MAX_BLE_PAYLOAD
						 : payload_mtu;
			}
			publish_link_state();
			return 0;

		case BLE_GAP_EVENT_SUBSCRIBE:
			if (event->subscribe.conn_handle != conn_handle
				|| event->subscribe.attr_handle != tx_char_val_handle
				|| link_phase == BLE_LINK_DOWN
				|| link_phase == BLE_LINK_TERMINATING) {
				return 0;
			}
			notify_subscribed = event->subscribe.cur_notify != 0;
			if (!notify_subscribed && link_phase == BLE_LINK_ACTIVE) {
				link_phase = BLE_LINK_QUALIFYING;
				valid_packet_seen = false;
				connection_started_ms = now_ms();
			}
			qualify_connection_if_ready();
			return 0;

		case BLE_GAP_EVENT_ENC_CHANGE:
			if (event->enc_change.conn_handle == conn_handle
				&& event->enc_change.status == 0
				&& link_phase == BLE_LINK_QUALIFYING) {
				// Pairing time is not charged against protocol qualification.
				connection_started_ms = now_ms();
			}
			return 0;

		case BLE_GAP_EVENT_ADV_COMPLETE:
			ensure_advertising();
			return 0;

		case BLE_GAP_EVENT_TERM_FAILURE:
			if (event->term_failure.conn_handle == conn_handle
				&& link_phase == BLE_LINK_TERMINATING) {
				ESP_LOGW(TAG, "BLE termination failed: %d",
					event->term_failure.status);
				schedule_connection_supervisor();
			}
			return 0;

		case BLE_GAP_EVENT_PASSKEY_ACTION: {
			if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
				struct ble_sm_io io = {
					.action  = BLE_SM_IOACT_DISP,
					.passkey = backup.config.ble_pin,
				};
				ble_sm_inject_io(event->passkey.conn_handle, &io);
			}
			return 0;
		}

		case BLE_GAP_EVENT_REPEAT_PAIRING: {
			struct ble_gap_conn_desc desc;
			if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc)
				== 0) {
				ble_store_util_delete_peer(&desc.peer_id_addr);
			}
			return BLE_GAP_REPEAT_PAIRING_RETRY;
		}

		case BLE_GAP_EVENT_NOTIFY_TX:
		case BLE_GAP_EVENT_CONN_UPDATE:
		default:
			return 0;
	}
}

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
) {
	(void)arg;
	(void)attr_h;

	switch (ctxt->op) {
		case BLE_GATT_ACCESS_OP_WRITE_CHR: {
			if (conn_h != conn_handle || link_phase == BLE_LINK_DOWN
				|| link_phase == BLE_LINK_TERMINATING
				|| ble_uuid_cmp(ctxt->chr->uuid, &rx_uuid.u) != 0) {
				return 0;
			}

			uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
			uint8_t buf[MAX_BLE_PAYLOAD];
			if (len > sizeof(buf))
				len = sizeof(buf);

			uint16_t out_len = 0;
			int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, len, &out_len);
			if (rc != 0) {
				return BLE_ATT_ERR_UNLIKELY;
			}

			if (packet_state) {
				for (uint16_t i = 0; i < out_len; i++) {
					packet_process_byte(buf[i], packet_state);
				}
			}
			return 0;
		}
		case BLE_GATT_ACCESS_OP_READ_CHR:
			// Reading the TX char returns no current value; VESC Tool only
			// reads it to discover the value handle; the data flows via notify.
			return 0;

		default:
			return BLE_ATT_ERR_UNLIKELY;
	}
}

static void on_sync(void) {
	host_synced = true;
	identity_ready = false;
	reset_pending = false;
	apply_tx_power();
	ensure_advertising();
}

static void on_reset(int reason) {
	ESP_LOGW(TAG, "NimBLE host reset: %d", reason);
	stop_supervisor();
	host_synced = false;
	identity_ready = false;
	clear_connection();
}

static void host_task(void *param) {
	(void)param;
	nimble_port_run();
	if (supervisor_initialized) {
		ble_npl_callout_deinit(&supervisor_callout);
		supervisor_initialized = false;
	}
	nimble_port_freertos_deinit();
}

static void process_packet(unsigned char *data, unsigned int len) {
	if (link_phase != BLE_LINK_QUALIFYING && link_phase != BLE_LINK_ACTIVE) {
		return;
	}

	last_valid_packet_ms = now_ms();
	valid_packet_seen = true;
	qualify_connection_if_ready();
	commands_process_packet(data, len, comm_ble_send_packet);
}

static void free_packet_state(void) {
	if (packet_state) {
		free(packet_state);
		packet_state = NULL;
	}
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	ble_link_snapshot_t initial = get_link_state();
	if (!initial.connected || !initial.subscribed || initial.terminating
		|| initial.conn_handle == BLE_HS_CONN_HANDLE_NONE
		|| tx_char_val_handle == 0) {
		return;
	}

	uint16_t bytes_sent = 0;
	while (bytes_sent < len) {
		ble_link_snapshot_t current = get_link_state();
		if (!current.connected || !current.subscribed || current.terminating
			|| current.conn_handle != initial.conn_handle
			|| current.generation != initial.generation) {
			return;
		}

		uint16_t chunk = len - bytes_sent;
		if (chunk > current.mtu) {
			chunk = current.mtu;
		}

		bool sent = false;

		for (int attempt = 0; attempt <= BLE_NOTIFY_MAX_RETRIES; attempt++) {
			current = get_link_state();
			if (!current.connected || !current.subscribed
				|| current.terminating
				|| current.conn_handle != initial.conn_handle
				|| current.generation != initial.generation) {
				return;
			}

			struct os_mbuf *om =
				ble_hs_mbuf_from_flat(buffer + bytes_sent, chunk);
			if (om == NULL) {
				vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_RETRY_MS));
				continue;
			}

			int rc = ble_gattc_notify_custom(
				current.conn_handle, tx_char_val_handle, om
			);
			if (rc == 0) {
				sent = true;
				break;
			}

			// notify_custom consumes the mbuf. Give the host/controller queues
			// time to drain before retrying long VESC packets such as config XML.
			vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_RETRY_MS));
		}

		if (!sent) {
			return;
		}

		bytes_sent += chunk;

		if (bytes_sent < len) {
			vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_CHUNK_GAP_MS));
		}
	}
}

void comm_ble_init(void) {
	tx_mutex = xSemaphoreCreateMutexStatic(&tx_mutex_storage);
	if (!tx_mutex) {
		return;
	}

	packet_state = calloc(1, sizeof(PACKET_STATE_t));
	if (!packet_state) {
		return;
	}

	packet_init(send_packet_raw, process_packet, packet_state);

	esp_err_t err = nimble_port_init();
	if (err != ESP_OK) {
		free_packet_state();
		return;
	}

	ble_hs_cfg.reset_cb          = on_reset;
	ble_hs_cfg.sync_cb           = on_sync;
	ble_hs_cfg.gatts_register_cb = NULL;
	ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		ble_hs_cfg.sm_io_cap       = BLE_SM_IO_CAP_DISP_ONLY;
		ble_hs_cfg.sm_bonding      = 1;
		ble_hs_cfg.sm_mitm         = 1;
		ble_hs_cfg.sm_sc           = 1;
		ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC
			| BLE_SM_PAIR_KEY_DIST_ID;
		ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC
			| BLE_SM_PAIR_KEY_DIST_ID;
	} else {
		ble_hs_cfg.sm_io_cap  = BLE_SM_IO_CAP_NO_IO;
		ble_hs_cfg.sm_bonding = 0;
		ble_hs_cfg.sm_mitm    = 0;
		ble_hs_cfg.sm_sc      = 0;
	}

	ble_svc_gap_init();
	ble_svc_gatt_init();
	ble_store_config_init();
	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		gatt_chrs[0].flags |= BLE_GATT_CHR_F_WRITE_ENC;
		gatt_chrs[1].flags |= BLE_GATT_CHR_F_READ_ENC;
	}

	int rc = ble_gatts_count_cfg(gatt_svcs);
	if (rc != 0) {
		nimble_port_deinit();
		free_packet_state();
		return;
	}
	rc = ble_gatts_add_svcs(gatt_svcs);
	if (rc != 0) {
		nimble_port_deinit();
		free_packet_state();
		return;
	}

	rc = ble_svc_gap_device_name_set((const char *)backup.config.ble_name);
	if (rc != 0) {
		nimble_port_deinit();
		free_packet_state();
		return;
	}

	rc = ble_npl_callout_init(
		&supervisor_callout, nimble_port_get_dflt_eventq(), supervisor_cb, NULL
	);
	if (rc == 0) {
		supervisor_initialized = true;
	} else {
		ESP_LOGE(TAG, "BLE supervisor initialization failed: %d", rc);
	}

	nimble_port_freertos_init(host_task);
}

bool comm_ble_is_connected(void) {
	return get_link_state().connected;
}

int comm_ble_mtu_now(void) {
	return get_link_state().mtu;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	if (!packet_state || !tx_mutex) {
		return;
	}

	if (xSemaphoreTake(tx_mutex, portMAX_DELAY) == pdTRUE) {
		packet_send_packet(data, len, packet_state);
		xSemaphoreGive(tx_mutex);
	}
}

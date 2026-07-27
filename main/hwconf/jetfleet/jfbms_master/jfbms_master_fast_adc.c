/*
 * JFBMS master continuous ADC sampler and hardware overcurrent backstop.
 *
 * The task-side cache supplies calibrated current, charger voltage and PCB
 * temperature readings. The ADC monitor callback only performs the latency-
 * critical fail-closed CHG_EN write and records a small diagnostic latch.
 */

#include "jfbms_master_fast_adc.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_monitor.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"
#include "soc/soc_caps.h"

#include <math.h>
#include <string.h>

#define JFBMS_ADC_FRAME_BYTES       256
#define JFBMS_ADC_STORE_BYTES       2048
#define JFBMS_ADC_PATTERN_LEN       8
// The control loop normally refreshes at 10 Hz, but Lisp scheduling and CAN
// work can move an individual read just beyond 100 ms. Keep the same
// fail-closed freshness model while allowing bounded scheduler jitter.
#define JFBMS_ADC_CACHE_MAX_AGE_MS  300
#define JFBMS_ADC_MAX_FRAMES_WAKE   8
#define JFBMS_ADC_TASK_PRIORITY     7
#define JFBMS_ADC_RECONFIG_TIMEOUT_MS 100U
#define JFBMS_FAST_OC_RTC_MAGIC     0x4A464F43U
#define JFBMS_CHARGER_CONNECT_DEBOUNCE_MS 75U
#define JFBMS_CHARGER_DISCONNECT_DEBOUNCE_MS 250U
#define JFBMS_CHARGER_HYSTERESIS_V  0.5f

static const adc_channel_t m_adc_pattern_channels[JFBMS_ADC_PATTERN_LEN] = {
	HW_ADC_CH2, HW_ADC_CH3, HW_ADC_CH2, HW_ADC_CH2,
	HW_ADC_CH4, HW_ADC_CH2, HW_ADC_CH2, HW_ADC_CH2,
};

static adc_continuous_handle_t m_adc_handle;
static adc_monitor_handle_t m_adc_monitor;
static adc_cali_handle_t m_adc_cali[5];
static TaskHandle_t m_adc_task_handle;
static SemaphoreHandle_t m_adc_mutex;
static StaticSemaphore_t m_adc_mutex_storage;
static volatile bool m_adc_started;
static volatile bool m_adc_reconfiguring;
static volatile bool m_fast_oc_armed;
static volatile bool m_adc_monitor_enabled;
static float m_adc_monitor_offset_v;
static float m_adc_monitor_trip_a;
static volatile bool m_fast_oc_latch;
static volatile int8_t m_fast_oc_direction;
static volatile uint32_t m_fast_oc_trip_count;
static volatile uint32_t m_fast_oc_diag_seq;
static volatile uint32_t m_fast_oc_last_raw;
static volatile float m_fast_oc_last_current_a;
static volatile int64_t m_fast_oc_trip_time_us;
static volatile float m_fast_current_offset_v = 1.65f;
static volatile uint32_t m_current_latest_raw;
static volatile float m_current_latest_a;
static volatile uint32_t m_charger_absent_since_ms;
static volatile bool m_charger_valid;
static volatile bool m_charger_detected;
static volatile bool m_charger_candidate;
static volatile uint32_t m_charger_candidate_since_ms;
static volatile float m_charger_latest_v;

static volatile float m_adc_voltage[5];
static volatile uint32_t m_adc_voltage_time_ms[5];

RTC_NOINIT_ATTR static uint32_t m_fast_oc_rtc_magic;

static uint32_t adc_now_ms(void) {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static float adc_raw_to_voltage(adc_channel_t channel, uint32_t raw) {
	if (channel >= 0 && channel < 5 && m_adc_cali[channel]) {
		int mv = 0;
		if (adc_cali_raw_to_voltage(m_adc_cali[channel], raw, &mv) == ESP_OK) {
			return (float)mv / 1000.0f;
		}
	}
	return -1.0f;
}

static int adc_voltage_to_raw(adc_channel_t channel, float voltage) {
	if (voltage <= 0.0f) return 0;
	if (voltage >= 3.3f) return 4095;

	int low = 0;
	int high = 4095;
	while (low < high) {
		int mid = low + ((high - low) / 2);
		if (adc_raw_to_voltage(channel, (uint32_t)mid) < voltage) {
			low = mid + 1;
		} else {
			high = mid;
		}
	}
	return low;
}

static bool IRAM_ATTR fast_oc_trip(adc_monitor_handle_t monitor,
		int8_t direction) {
	// This must remain scheduler-, logging- and mutex-free.
	// Disable the level-sensitive monitor on the first event. Leaving it enabled
	// while the current remains outside the window generates an interrupt for
	// every ADC conversion and can starve the continuous-reader task and UI.
	if (!m_adc_monitor_enabled || !m_fast_oc_armed ||
			m_adc_reconfiguring) {
		return false;
	}
	m_fast_oc_armed = false;
	if (adc_continuous_monitor_disable(monitor) == ESP_OK) {
		m_adc_monitor_enabled = false;
	}
	GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
	m_fast_oc_rtc_magic = JFBMS_FAST_OC_RTC_MAGIC;
	m_fast_oc_diag_seq++;
	m_fast_oc_latch = true;
	m_fast_oc_direction = direction;
	m_charger_absent_since_ms = 0;
	m_fast_oc_trip_count++;
	m_fast_oc_last_raw = m_current_latest_raw;
	m_fast_oc_last_current_a = m_current_latest_a;
	m_fast_oc_trip_time_us = esp_timer_get_time();
	m_fast_oc_diag_seq++;
	return false;
}

static bool IRAM_ATTR fast_oc_below_threshold_cb(adc_monitor_handle_t monitor,
		const adc_monitor_evt_data_t *event_data, void *user_data) {
	(void)event_data;
	(void)user_data;
	return fast_oc_trip(monitor, -1);
}

static bool IRAM_ATTR fast_oc_above_threshold_cb(adc_monitor_handle_t monitor,
		const adc_monitor_evt_data_t *event_data, void *user_data) {
	(void)event_data;
	(void)user_data;
	return fast_oc_trip(monitor, 1);
}

static void charger_detector_update(float voltage_v, uint32_t now_ms) {
	main_config_t *cfg = (main_config_t *)&backup.config;
	float threshold = cfg->v_charge_detect;
	if (!isfinite(voltage_v) || !isfinite(threshold) || threshold <= 0.0f) {
		m_charger_valid = false;
		m_charger_candidate_since_ms = 0;
		return;
	}

	m_charger_latest_v = voltage_v;
	m_charger_valid = true;
	// Maintain physical-absence time continuously, not only when the user asks
	// to clear a fault. This makes the first reset request work after the port
	// has already been safely disconnected for five seconds.
	if (voltage_v < (threshold - JFBMS_CHARGER_HYSTERESIS_V)) {
		if (m_charger_absent_since_ms == 0) m_charger_absent_since_ms = now_ms;
	} else {
		m_charger_absent_since_ms = 0;
	}
	float edge = threshold + (m_charger_detected ?
			-JFBMS_CHARGER_HYSTERESIS_V : JFBMS_CHARGER_HYSTERESIS_V);
	bool candidate = voltage_v > edge;
	if (candidate == m_charger_detected) {
		m_charger_candidate = candidate;
		m_charger_candidate_since_ms = now_ms;
		return;
	}

	if (candidate != m_charger_candidate || m_charger_candidate_since_ms == 0) {
		m_charger_candidate = candidate;
		m_charger_candidate_since_ms = now_ms;
		return;
	}

	uint32_t debounce_ms = candidate ? JFBMS_CHARGER_CONNECT_DEBOUNCE_MS :
			JFBMS_CHARGER_DISCONNECT_DEBOUNCE_MS;
	if ((now_ms - m_charger_candidate_since_ms) >= debounce_ms) {
		m_charger_detected = candidate;
		m_charger_candidate_since_ms = now_ms;
	}
}

static bool IRAM_ATTR adc_conversion_done_cb(adc_continuous_handle_t handle,
		const adc_continuous_evt_data_t *event_data, void *user_data) {
	(void)handle;
	(void)event_data;
	(void)user_data;
	BaseType_t wake = pdFALSE;
	if (m_adc_task_handle) {
		vTaskNotifyGiveFromISR(m_adc_task_handle, &wake);
	}
	return wake == pdTRUE;
}

static void adc_process_frame(const uint8_t *data, uint32_t length) {
	adc_continuous_data_t parsed[JFBMS_ADC_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES];
	uint32_t parsed_count = 0;
	if (adc_continuous_parse_data(m_adc_handle, data, length, parsed,
			&parsed_count) != ESP_OK) {
		return;
	}

	uint64_t sums[5] = {0};
	uint32_t counts[5] = {0};
	for (uint32_t i = 0; i < parsed_count; i++) {
		if (!parsed[i].valid || parsed[i].unit != ADC_UNIT_1) continue;
		int channel = parsed[i].channel;
		if (channel < 0 || channel >= 5) continue;
		sums[channel] += parsed[i].raw_data;
		counts[channel]++;
		if (channel == HW_ADC_CH2) m_current_latest_raw = parsed[i].raw_data;
	}

	uint32_t now_ms = adc_now_ms();
	for (int channel = 0; channel < 5; channel++) {
		if (counts[channel] == 0) continue;
		uint32_t average_raw = (uint32_t)(sums[channel] / counts[channel]);
		float voltage = adc_raw_to_voltage((adc_channel_t)channel, average_raw);
		if (voltage < 0.0f) continue;
		m_adc_voltage[channel] = voltage;
		m_adc_voltage_time_ms[channel] = now_ms;
		if (channel == HW_ADC_CH2) {
			m_current_latest_a =
					(voltage - m_fast_current_offset_v) * ISENSE_SCALE;
		} else if (channel == HW_ADC_CH3) {
			charger_detector_update(voltage * VCHG_DIV_SCALE, now_ms);
		}
	}
}

static void adc_reader_task(void *arg) {
	(void)arg;
	uint8_t frame[JFBMS_ADC_FRAME_BYTES];

	while (true) {
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
		if (m_adc_reconfiguring || !m_adc_started) continue;
		if (!m_adc_mutex || xSemaphoreTake(m_adc_mutex, portMAX_DELAY) != pdTRUE) continue;
		if (m_adc_reconfiguring || !m_adc_started) {
			xSemaphoreGive(m_adc_mutex);
			continue;
		}

		uint32_t frames_processed = 0;
		while (!m_adc_reconfiguring && frames_processed < JFBMS_ADC_MAX_FRAMES_WAKE) {
			uint32_t length = 0;
			esp_err_t res = adc_continuous_read(m_adc_handle, frame,
					sizeof(frame), &length, 0);
			if (res == ESP_ERR_TIMEOUT || res != ESP_OK || length == 0) break;
			adc_process_frame(frame, length);
			frames_processed++;
		}
		xSemaphoreGive(m_adc_mutex);
		if (frames_processed == JFBMS_ADC_MAX_FRAMES_WAKE) vTaskDelay(1);
	}
}

static void adc_init_cleanup(void) {
	m_adc_reconfiguring = true;
	if (m_adc_started && m_adc_handle) (void)adc_continuous_stop(m_adc_handle);
	m_adc_started = false;
	if (m_adc_task_handle) {
		vTaskDelete(m_adc_task_handle);
		m_adc_task_handle = NULL;
	}
	if (m_adc_monitor) {
		if (m_adc_monitor_enabled) {
			(void)adc_continuous_monitor_disable(m_adc_monitor);
		}
		(void)adc_del_continuous_monitor(m_adc_monitor);
		m_adc_monitor = NULL;
	}
	m_adc_monitor_enabled = false;
	m_adc_monitor_offset_v = 0.0f;
	m_adc_monitor_trip_a = 0.0f;
	for (int channel = 0; channel < 5; channel++) {
		if (m_adc_cali[channel]) {
			(void)adc_cali_delete_scheme_curve_fitting(m_adc_cali[channel]);
			m_adc_cali[channel] = NULL;
		}
	}
	if (m_adc_handle) {
		(void)adc_continuous_deinit(m_adc_handle);
		m_adc_handle = NULL;
	}
	m_fast_oc_armed = false;
	m_adc_reconfiguring = false;
}

static bool adc_set_current_monitor_enabled(bool enable) {
	if (!m_adc_monitor) return false;
	if (m_adc_monitor_enabled == enable) return true;

	esp_err_t res = enable ?
			adc_continuous_monitor_enable(m_adc_monitor) :
			adc_continuous_monitor_disable(m_adc_monitor);
	if (res != ESP_OK) return false;
	m_adc_monitor_enabled = enable;
	return true;
}

// The ADC driver must be stopped before this helper is called.
static bool adc_remove_current_monitor(void) {
	if (!m_adc_monitor) return true;
	if (m_adc_monitor_enabled &&
			adc_continuous_monitor_disable(m_adc_monitor) != ESP_OK) {
		return false;
	}
	m_adc_monitor_enabled = false;
	if (adc_del_continuous_monitor(m_adc_monitor) != ESP_OK) return false;
	m_adc_monitor = NULL;
	m_adc_monitor_offset_v = 0.0f;
	m_adc_monitor_trip_a = 0.0f;
	return true;
}

static bool adc_install_current_monitor(float offset_v) {
	main_config_t *cfg = (main_config_t *)&backup.config;
	float trip_current = cfg->fast_charge_oc_a;
	float low_voltage = offset_v - (trip_current / ISENSE_SCALE);
	float high_voltage = offset_v + (trip_current / ISENSE_SCALE);
	bool thresholds_valid = cfg->fast_charge_oc_en &&
			isfinite(offset_v) && isfinite(cfg->max_charge_current) &&
			isfinite(trip_current) &&
			cfg->max_charge_current < trip_current &&
			trip_current > 0.0f &&
			trip_current <= JFBMS_FAST_OC_MAX_A &&
			low_voltage > 0.02f && high_voltage < 3.28f;
	if (!thresholds_valid) return false;

	adc_monitor_config_t monitor_cfg = {
		.adc_unit = ADC_UNIT_1,
		.channel = HW_ADC_CH2,
		.h_threshold = adc_voltage_to_raw(HW_ADC_CH2, high_voltage),
		.l_threshold = adc_voltage_to_raw(HW_ADC_CH2, low_voltage),
	};
	adc_monitor_evt_cbs_t monitor_callbacks = {
		.on_below_low_thresh = fast_oc_below_threshold_cb,
		.on_over_high_thresh = fast_oc_above_threshold_cb,
	};
	if (adc_new_continuous_monitor(m_adc_handle, &monitor_cfg,
			&m_adc_monitor) != ESP_OK) {
		return false;
	}
	m_adc_monitor_enabled = false;
	if (adc_continuous_monitor_register_event_callbacks(m_adc_monitor,
			&monitor_callbacks, NULL) != ESP_OK ||
			!adc_set_current_monitor_enabled(true)) {
		(void)adc_remove_current_monitor();
		return false;
	}
	m_adc_monitor_offset_v = offset_v;
	m_adc_monitor_trip_a = trip_current;
	return true;
}

bool jfbms_fast_adc_init(void) {
	if (m_adc_handle && m_adc_started) return true;
	if (m_adc_handle) adc_init_cleanup();
	if (m_fast_oc_rtc_magic == JFBMS_FAST_OC_RTC_MAGIC) m_fast_oc_latch = true;
	if (!m_adc_mutex) m_adc_mutex = xSemaphoreCreateMutexStatic(&m_adc_mutex_storage);
	if (!m_adc_mutex) return false;

	adc_continuous_handle_cfg_t handle_cfg = {
		.max_store_buf_size = JFBMS_ADC_STORE_BYTES,
		.conv_frame_size = JFBMS_ADC_FRAME_BYTES,
	};
	if (adc_continuous_new_handle(&handle_cfg, &m_adc_handle) != ESP_OK) {
		m_adc_handle = NULL;
		return false;
	}

	adc_digi_pattern_config_t patterns[JFBMS_ADC_PATTERN_LEN] = {0};
	for (int i = 0; i < JFBMS_ADC_PATTERN_LEN; i++) {
		patterns[i].atten = ADC_ATTEN_DB_12;
		patterns[i].channel = m_adc_pattern_channels[i];
		patterns[i].unit = ADC_UNIT_1;
		patterns[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
	}
	adc_continuous_config_t continuous_cfg = {
		.pattern_num = JFBMS_ADC_PATTERN_LEN,
		.adc_pattern = patterns,
		.sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
	};
	if (adc_continuous_config(m_adc_handle, &continuous_cfg) != ESP_OK) {
		adc_init_cleanup();
		return false;
	}

	const adc_channel_t calibrated_channels[] = {HW_ADC_CH2, HW_ADC_CH3, HW_ADC_CH4};
	for (size_t i = 0; i < sizeof(calibrated_channels) / sizeof(calibrated_channels[0]); i++) {
		adc_channel_t channel = calibrated_channels[i];
		adc_cali_curve_fitting_config_t cali_cfg = {
			.unit_id = ADC_UNIT_1,
			.chan = channel,
			.atten = ADC_ATTEN_DB_12,
			.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &m_adc_cali[channel]) != ESP_OK ||
				adc_raw_to_voltage(channel, 2048) <= 0.0f) {
			adc_init_cleanup();
			return false;
		}
	}

	adc_continuous_evt_cbs_t adc_callbacks = {.on_conv_done = adc_conversion_done_cb};
	if (adc_continuous_register_event_callbacks(m_adc_handle, &adc_callbacks, NULL) != ESP_OK) {
		adc_init_cleanup();
		return false;
	}
	if (xTaskCreate(adc_reader_task, "jfbms-adc", 4096, NULL,
			JFBMS_ADC_TASK_PRIORITY, &m_adc_task_handle) != pdPASS) {
		m_adc_task_handle = NULL;
		adc_init_cleanup();
		return false;
	}
	if (adc_continuous_start(m_adc_handle) != ESP_OK) {
		adc_init_cleanup();
		return false;
	}
	m_adc_started = true;
	// hw_init waits for the powered current reference to settle before it calls
	// jfbms_fast_adc_set_current_offset() and permits protection to arm.
	m_fast_oc_armed = false;
	return true;
}

float hw_adc_get_voltage(adc_channel_t channel) {
	if (channel < 0 || channel >= 5 || !m_adc_started) return -1.0f;
	uint32_t sample_ms = m_adc_voltage_time_ms[channel];
	if (sample_ms == 0 || (adc_now_ms() - sample_ms) > JFBMS_ADC_CACHE_MAX_AGE_MS) return -1.0f;
	return m_adc_voltage[channel];
}

static bool adc_rebuild_current_monitor(float offset_v) {
	m_adc_reconfiguring = true;
	GPIO.out_w1tc.val = BIT(PIN_CHG_EN);

	TickType_t lock_wait = pdMS_TO_TICKS(JFBMS_ADC_RECONFIG_TIMEOUT_MS);
	if (lock_wait == 0) lock_wait = 1;
	if (!m_adc_mutex || xSemaphoreTake(m_adc_mutex, lock_wait) != pdTRUE) {
		m_adc_reconfiguring = false;
		return false;
	}

	if (adc_continuous_stop(m_adc_handle) != ESP_OK) {
		xSemaphoreGive(m_adc_mutex);
		m_adc_reconfiguring = false;
		return false;
	}
	m_adc_started = false;

	bool configured = adc_remove_current_monitor();
	main_config_t *cfg = (main_config_t *)&backup.config;
	if (configured && cfg->fast_charge_oc_en) {
		configured = adc_install_current_monitor(offset_v);
	}

	bool restarted = adc_continuous_start(m_adc_handle) == ESP_OK;
	m_adc_started = restarted;
	if (!restarted && m_adc_monitor_enabled) {
		(void)adc_set_current_monitor_enabled(false);
	}
	xSemaphoreGive(m_adc_mutex);
	m_adc_reconfiguring = false;
	m_fast_oc_armed = configured && restarted;
	return m_fast_oc_armed;
}

static bool adc_reconfigure_current_monitor(float offset_v, bool arm_protection) {
	if (!m_adc_handle || !m_adc_started || !isfinite(offset_v)) return false;

	// Diagnostics and current-based latch clearing use the same captured zero as
	// the UI, even if hardware-threshold reconfiguration subsequently fails.
	m_fast_current_offset_v = offset_v;
	m_fast_oc_armed = false;
	if (!arm_protection) {
		return !m_adc_monitor_enabled ||
				adc_set_current_monitor_enabled(false);
	}

	main_config_t *cfg = (main_config_t *)&backup.config;
	if (!cfg->fast_charge_oc_en) {
		// Protection disabled means no comparator callback may affect CHG_EN.
		// Leave an existing monitor allocated but physically disabled so normal
		// ADC sampling continues without a stop/restart.
		if (m_adc_monitor_enabled &&
				!adc_set_current_monitor_enabled(false)) {
			return false;
		}
		m_fast_oc_armed = true;
		return true;
	}

	bool monitor_matches = m_adc_monitor &&
			fabsf(offset_v - m_adc_monitor_offset_v) <= 0.001f &&
			fabsf(cfg->fast_charge_oc_a - m_adc_monitor_trip_a) <= 0.001f;
	if (monitor_matches) {
		if (!m_adc_monitor_enabled &&
				!adc_set_current_monitor_enabled(true)) {
			return false;
		}
		m_fast_oc_armed = true;
		return true;
	}

	// ESP-IDF permits monitor creation/deletion only while the continuous ADC is
	// stopped. Hold the reader mutex across this bounded rebuild and always
	// restart the sampler, including failure paths, so current UI data is not
	// coupled to the optional protection setting.
	return adc_rebuild_current_monitor(offset_v);
}

bool jfbms_fast_adc_disarm_current_monitor(void) {
	return adc_reconfigure_current_monitor(m_fast_current_offset_v, false);
}

bool jfbms_fast_adc_set_current_offset(float offset_v) {
	return adc_reconfigure_current_monitor(offset_v, true);
}

bool jfbms_fast_adc_ready(void) {
	main_config_t *cfg = (main_config_t *)&backup.config;
	return m_adc_started && m_fast_oc_armed &&
			(!cfg->fast_charge_oc_en || m_adc_monitor_enabled);
}

bool jfbms_fast_oc_sleep_disarm(void) {
	// CHG_EN is already fail-closed here. Disable the retained monitor without
	// stopping the continuous ADC while the current reference is powered down.
	if (!m_adc_started || m_adc_reconfiguring || !m_fast_oc_armed) return false;
	if (m_adc_monitor_enabled &&
			!adc_set_current_monitor_enabled(false)) {
		return false;
	}
	m_fast_oc_armed = false;
	return true;
}

bool jfbms_fast_oc_sleep_rearm(void) {
	// The caller must restore COM_EN and wait for the 1.65 V reference to settle
	// before restoring the configured monitor state.
	main_config_t *cfg = (main_config_t *)&backup.config;
	if (!m_adc_started || m_adc_reconfiguring) return false;
	if (!cfg->fast_charge_oc_en) {
		m_fast_oc_armed = true;
		return true;
	}
	if (!m_adc_monitor) return false;
	if (!m_adc_monitor_enabled &&
			!adc_set_current_monitor_enabled(true)) {
		return false;
	}
	m_fast_oc_armed = true;
	return true;
}

bool jfbms_fast_oc_latched(void) {
	return m_fast_oc_latch;
}

bool jfbms_fast_oc_clear_allowed(float charger_detect_v) {
	float vchg = hw_adc_get_voltage(HW_ADC_CH3);
	float current_v = hw_adc_get_voltage(HW_ADC_CH2);
	uint32_t now_ms = adc_now_ms();
	if (vchg < 0.0f || current_v < 0.0f ||
			(vchg * VCHG_DIV_SCALE) >= charger_detect_v) {
		m_charger_absent_since_ms = 0;
		return false;
	}
	if (m_charger_absent_since_ms == 0) {
		m_charger_absent_since_ms = now_ms;
		return false;
	}
	float current_a = (current_v - m_fast_current_offset_v) * ISENSE_SCALE;
	return (now_ms - m_charger_absent_since_ms) >= 5000 && fabsf(current_a) < 1.0f;
}

bool jfbms_fast_oc_clear_if_unchanged(uint32_t expected_trip_count) {
	if (m_fast_oc_trip_count != expected_trip_count) return false;

	// A trip turns the level-sensitive comparator off to prevent an interrupt
	// storm. Restore the configured state only while the charger is known absent
	// and the measured current is near zero (checked by the caller).
	main_config_t *cfg = (main_config_t *)&backup.config;
	if (cfg->fast_charge_oc_en) {
		if (!m_adc_monitor ||
				(!m_adc_monitor_enabled &&
				!adc_set_current_monitor_enabled(true))) {
			return false;
		}
	} else if (m_adc_monitor_enabled &&
			!adc_set_current_monitor_enabled(false)) {
		return false;
	}
	m_fast_oc_armed = true;
	if (m_fast_oc_trip_count != expected_trip_count) return false;

	m_fast_oc_latch = false;
	m_charger_absent_since_ms = 0;
	m_fast_oc_rtc_magic = 0;
	if (m_fast_oc_trip_count != expected_trip_count) {
		m_fast_oc_rtc_magic = JFBMS_FAST_OC_RTC_MAGIC;
		m_fast_oc_latch = true;
		return false;
	}
	return true;
}

void jfbms_fast_oc_get_status(jfbms_fast_oc_status_t *status) {
	if (!status) return;
	for (;;) {
		uint32_t before = m_fast_oc_diag_seq;
		if (before & 1U) continue;
		status->latched = m_fast_oc_latch;
		status->armed = m_fast_oc_armed;
		status->direction = m_fast_oc_direction;
		status->trip_count = m_fast_oc_trip_count;
		status->last_raw = m_fast_oc_last_raw;
		status->last_current_a = m_fast_oc_last_current_a;
		status->trip_time_us = m_fast_oc_trip_time_us;
		uint32_t after = m_fast_oc_diag_seq;
		if (before == after && !(after & 1U)) return;
	}
}

void jfbms_charger_get_status(jfbms_charger_status_t *status) {
	if (!status) return;
	uint32_t now_ms = adc_now_ms();
	uint32_t sample_ms = m_adc_voltage_time_ms[HW_ADC_CH3];
	bool fresh = sample_ms != 0 &&
			(now_ms - sample_ms) <= JFBMS_ADC_CACHE_MAX_AGE_MS;
	status->valid = m_adc_started && m_charger_valid && fresh;
	status->detected = status->valid && m_charger_detected;
	status->voltage_v = status->valid ? m_charger_latest_v : 0.0f;
	status->sample_age_ms = sample_ms == 0 ? UINT32_MAX : now_ms - sample_ms;
}

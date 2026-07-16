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
#define JFBMS_FAST_OC_RTC_MAGIC     0x4A464F43U

static const adc_channel_t m_adc_pattern_channels[JFBMS_ADC_PATTERN_LEN] = {
	HW_ADC_CH2, HW_ADC_CH3, HW_ADC_CH2, HW_ADC_CH2,
	HW_ADC_CH4, HW_ADC_CH2, HW_ADC_CH2, HW_ADC_CH2,
};

static adc_continuous_handle_t m_adc_handle;
static adc_monitor_handle_t m_adc_monitor;
static adc_cali_handle_t m_adc_cali[5];
static TaskHandle_t m_adc_task_handle;
static volatile bool m_adc_started;
static volatile bool m_adc_reconfiguring;
static volatile bool m_fast_oc_armed;
static volatile bool m_fast_oc_latch;
static volatile uint32_t m_fast_oc_trip_count;
static volatile uint32_t m_fast_oc_diag_seq;
static volatile uint32_t m_fast_oc_last_raw;
static volatile float m_fast_oc_last_current_a;
static volatile int64_t m_fast_oc_trip_time_us;
static volatile float m_fast_current_offset_v = 1.65f;
static volatile uint32_t m_current_latest_raw;
static volatile float m_current_latest_a;
static volatile uint32_t m_charger_absent_since_ms;

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

static bool IRAM_ATTR fast_oc_below_threshold_cb(adc_monitor_handle_t monitor,
		const adc_monitor_evt_data_t *event_data, void *user_data) {
	(void)monitor;
	(void)event_data;
	(void)user_data;

	// This must remain scheduler-, logging- and mutex-free.
	GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
	m_fast_oc_rtc_magic = JFBMS_FAST_OC_RTC_MAGIC;
	m_fast_oc_diag_seq++;
	m_fast_oc_latch = true;
	m_charger_absent_since_ms = 0;
	m_fast_oc_trip_count++;
	m_fast_oc_last_raw = m_current_latest_raw;
	m_fast_oc_last_current_a = m_current_latest_a;
	m_fast_oc_trip_time_us = esp_timer_get_time();
	m_fast_oc_diag_seq++;
	return false;
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
		}
	}
}

static void adc_reader_task(void *arg) {
	(void)arg;
	uint8_t frame[JFBMS_ADC_FRAME_BYTES];

	while (true) {
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
		if (m_adc_reconfiguring || !m_adc_started) continue;

		uint32_t frames_processed = 0;
		while (!m_adc_reconfiguring && frames_processed < JFBMS_ADC_MAX_FRAMES_WAKE) {
			uint32_t length = 0;
			esp_err_t res = adc_continuous_read(m_adc_handle, frame,
					sizeof(frame), &length, 0);
			if (res == ESP_ERR_TIMEOUT || res != ESP_OK || length == 0) break;
			adc_process_frame(frame, length);
			frames_processed++;
		}
		if (frames_processed == JFBMS_ADC_MAX_FRAMES_WAKE) vTaskDelay(1);
	}
}

bool jfbms_fast_adc_init(void) {
	if (m_adc_handle) return m_adc_started;
	if (m_fast_oc_rtc_magic == JFBMS_FAST_OC_RTC_MAGIC) m_fast_oc_latch = true;

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
	if (adc_continuous_config(m_adc_handle, &continuous_cfg) != ESP_OK) return false;

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
				adc_raw_to_voltage(channel, 2048) <= 0.0f) return false;
	}

	adc_continuous_evt_cbs_t adc_callbacks = {.on_conv_done = adc_conversion_done_cb};
	if (adc_continuous_register_event_callbacks(m_adc_handle, &adc_callbacks, NULL) != ESP_OK) return false;
	if (xTaskCreate(adc_reader_task, "jfbms-adc", 4096, NULL,
			JFBMS_ADC_TASK_PRIORITY, &m_adc_task_handle) != pdPASS) {
		m_adc_task_handle = NULL;
		return false;
	}
	if (adc_continuous_start(m_adc_handle) != ESP_OK) return false;
	m_adc_started = true;
	return true;
}

float hw_adc_get_voltage(adc_channel_t channel) {
	if (channel < 0 || channel >= 5 || !m_adc_started) return -1.0f;
	uint32_t sample_ms = m_adc_voltage_time_ms[channel];
	if (sample_ms == 0 || (adc_now_ms() - sample_ms) > JFBMS_ADC_CACHE_MAX_AGE_MS) return -1.0f;
	return m_adc_voltage[channel];
}

bool jfbms_fast_adc_set_current_offset(float offset_v) {
	if (!m_adc_handle || !m_adc_started || !isfinite(offset_v)) return false;

	m_fast_current_offset_v = offset_v;
	m_fast_oc_armed = false;
	m_adc_reconfiguring = true;
	GPIO.out_w1tc.val = BIT(PIN_CHG_EN);
	if (adc_continuous_stop(m_adc_handle) != ESP_OK) {
		m_adc_reconfiguring = false;
		return false;
	}
	m_adc_started = false;

	if (m_adc_monitor) {
		(void)adc_continuous_monitor_disable(m_adc_monitor);
		(void)adc_del_continuous_monitor(m_adc_monitor);
		m_adc_monitor = NULL;
	}

	main_config_t *cfg = (main_config_t *)&backup.config;
	float trip_current = cfg->fast_charge_oc_a;
	bool configured = !cfg->fast_charge_oc_en;
	if (cfg->fast_charge_oc_en) {
		configured = isfinite(cfg->max_charge_current) &&
				isfinite(trip_current) &&
				cfg->max_charge_current < trip_current &&
				trip_current > 0.0f &&
				trip_current <= JFBMS_FAST_OC_MAX_A &&
				offset_v - (trip_current / ISENSE_SCALE) > 0.02f;
	}
	if (configured && cfg->fast_charge_oc_en) {
		adc_monitor_config_t monitor_cfg = {
			.adc_unit = ADC_UNIT_1,
			.channel = HW_ADC_CH2,
			.h_threshold = -1,
			.l_threshold = adc_voltage_to_raw(HW_ADC_CH2,
					offset_v - (trip_current / ISENSE_SCALE)),
		};
		adc_monitor_evt_cbs_t monitor_callbacks = {.on_below_low_thresh = fast_oc_below_threshold_cb};
		configured = adc_new_continuous_monitor(m_adc_handle, &monitor_cfg,
				&m_adc_monitor) == ESP_OK &&
				adc_continuous_monitor_register_event_callbacks(m_adc_monitor,
				&monitor_callbacks, NULL) == ESP_OK &&
				adc_continuous_monitor_enable(m_adc_monitor) == ESP_OK;
	}

	if (adc_continuous_start(m_adc_handle) == ESP_OK) m_adc_started = true;
	else configured = false;
	m_adc_reconfiguring = false;
	m_fast_oc_armed = configured;
	return m_fast_oc_armed;
}

bool jfbms_fast_adc_ready(void) {
	return m_adc_started && m_fast_oc_armed;
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
		status->trip_count = m_fast_oc_trip_count;
		status->last_raw = m_fast_oc_last_raw;
		status->last_current_a = m_fast_oc_last_current_a;
		status->trip_time_us = m_fast_oc_trip_time_us;
		uint32_t after = m_fast_oc_diag_seq;
		if (before == after && !(after & 1U)) return;
	}
}

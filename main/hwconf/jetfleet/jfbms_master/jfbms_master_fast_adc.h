#ifndef JFBMS_MASTER_FAST_ADC_H_
#define JFBMS_MASTER_FAST_ADC_H_

#include <stdbool.h>
#include <stdint.h>

#include "hal/adc_types.h"

typedef struct {
	bool latched;
	bool armed;
	// -1 = below-offset/charge direction, +1 = above-offset/reverse direction.
	int8_t direction;
	uint32_t trip_count;
	uint32_t last_raw;
	float last_current_a;
	int64_t trip_time_us;
} jfbms_fast_oc_status_t;

typedef struct {
	bool valid;
	bool detected;
	float voltage_v;
	uint32_t sample_age_ms;
} jfbms_charger_status_t;

bool jfbms_fast_adc_init(void);
float hw_adc_get_voltage(adc_channel_t channel);
bool jfbms_fast_adc_disarm_current_monitor(void);
bool jfbms_fast_adc_set_current_offset(float offset_v);
bool jfbms_fast_adc_ready(void);
bool jfbms_fast_oc_sleep_disarm(void);
bool jfbms_fast_oc_sleep_rearm(void);
bool jfbms_fast_oc_latched(void);
bool jfbms_fast_oc_clear_allowed(float charger_detect_v);
bool jfbms_fast_oc_clear_if_unchanged(uint32_t expected_trip_count);
void jfbms_fast_oc_get_status(jfbms_fast_oc_status_t *status);
void jfbms_charger_get_status(jfbms_charger_status_t *status);

#endif

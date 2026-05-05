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

#include "adc.h"
#include "terminal.h"
#include "commands.h"

#include <math.h>

#if CONFIG_IDF_TARGET_ESP32C6
// ESP32-C6: continuous DMA mode at 20 kHz with hardware IIR filter (coeff 64).
// adc_get_voltage() drains the entire ring buffer and returns the average —
// ~2000 samples at 10 Hz update rate, zero blocking wait.
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_continuous_handle_t c6_cont_handle = NULL;
static adc_cali_handle_t       c6_cali_handle = NULL;
static bool                    c6_cali_ok     = false;

void adc_init(void) {
	// 1 kHz sample rate: 400 bytes/drain cycle (10 Hz) vs 16 KB ring buffer — no overflow.
	// conv_frame_size 64 bytes = 16 samples = 16 ms per frame; calibration 100 ms timeout
	// catches 6+ frames even right after a flush.
	adc_continuous_handle_cfg_t hcfg = {
		.max_store_buf_size = 4096,
		.conv_frame_size    = 64,
	};
	if (adc_continuous_new_handle(&hcfg, &c6_cont_handle) != ESP_OK) return;

	adc_digi_pattern_config_t patterns[5];
	int npat = 0;
#ifdef HW_ADC_CH0
	patterns[npat++] = (adc_digi_pattern_config_t){
		.atten = ADC_ATTEN_DB_12, .channel = (uint8_t)(HW_ADC_CH0),
		.unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH };
#endif
#ifdef HW_ADC_CH1
	patterns[npat++] = (adc_digi_pattern_config_t){
		.atten = ADC_ATTEN_DB_12, .channel = (uint8_t)(HW_ADC_CH1),
		.unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH };
#endif
#ifdef HW_ADC_CH2
	patterns[npat++] = (adc_digi_pattern_config_t){
		.atten = ADC_ATTEN_DB_12, .channel = (uint8_t)(HW_ADC_CH2),
		.unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH };
#endif
#ifdef HW_ADC_CH3
	patterns[npat++] = (adc_digi_pattern_config_t){
		.atten = ADC_ATTEN_DB_12, .channel = (uint8_t)(HW_ADC_CH3),
		.unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH };
#endif
#ifdef HW_ADC_CH4
	patterns[npat++] = (adc_digi_pattern_config_t){
		.atten = ADC_ATTEN_DB_12, .channel = (uint8_t)(HW_ADC_CH4),
		.unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH };
#endif
	if (npat == 0) return;

	adc_continuous_config_t ccfg = {
		.pattern_num    = (uint32_t)npat,
		.adc_pattern    = patterns,
		.sample_freq_hz = 1000,
		.conv_mode      = ADC_CONV_SINGLE_UNIT_1,
		.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
	};
	if (adc_continuous_config(c6_cont_handle, &ccfg) != ESP_OK) goto fail;

	// Hardware IIR filter — maximum smoothing before samples reach the ring buffer
#ifdef HW_ADC_CH2
	{
		static adc_iir_filter_handle_t iir_hdl;
		adc_continuous_iir_filter_config_t icfg = {
			.unit    = ADC_UNIT_1,
			.channel = (adc_channel_t)(HW_ADC_CH2),
			.coeff   = ADC_DIGI_IIR_FILTER_COEFF_64,
		};
		if (adc_new_continuous_iir_filter(c6_cont_handle, &icfg, &iir_hdl) == ESP_OK) {
			adc_continuous_iir_filter_enable(iir_hdl);
		}
	}
#endif

	// Curve-fitting calibration for ADC1 at 12 dB attenuation
	{
		adc_cali_curve_fitting_config_t calcfg = {
			.unit_id  = ADC_UNIT_1,
			.chan     = ADC_CHANNEL_0,
			.atten    = ADC_ATTEN_DB_12,
			.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
		};
		c6_cali_ok = (adc_cali_create_scheme_curve_fitting(&calcfg, &c6_cali_handle) == ESP_OK);
	}

	adc_continuous_start(c6_cont_handle);
	return;

fail:
	adc_continuous_deinit(c6_cont_handle);
	c6_cont_handle = NULL;
}

// Drains the entire DMA ring buffer, averages all samples for the requested channel.
// Blocks up to 100 ms on the first read so callers work correctly even right after
// the buffer was flushed (e.g. calibration). Subsequent reads use timeout=0 to drain.
// Returns calibrated voltage in V, or -1.0 if no data arrives within 100 ms.
float adc_get_voltage(adc1_channel_t ch) {
	if (!c6_cont_handle) return -1.0f;

	static uint8_t buf[256];  // static: not on stack; holds 4× conv_frame_size
	uint64_t sum   = 0;
	uint32_t count = 0, ret_num = 0;

	// First call: wait up to 100 ms for at least one conv_frame_size chunk
	if (adc_continuous_read(c6_cont_handle, buf, sizeof(buf), &ret_num, 100) != ESP_OK) {
		return -1.0f;
	}
	{
		int n = (int)(ret_num / sizeof(adc_digi_output_data_t));
		adc_digi_output_data_t *p = (adc_digi_output_data_t *)buf;
		for (int i = 0; i < n; i++) {
			if ((uint32_t)p[i].type2.channel == (uint32_t)ch) {
				sum += p[i].type2.data;
				count++;
			}
		}
	}

	// Drain the rest non-blocking
	while (adc_continuous_read(c6_cont_handle, buf, sizeof(buf), &ret_num, 0) == ESP_OK) {
		int n = (int)(ret_num / sizeof(adc_digi_output_data_t));
		adc_digi_output_data_t *p = (adc_digi_output_data_t *)buf;
		for (int i = 0; i < n; i++) {
			if ((uint32_t)p[i].type2.channel == (uint32_t)ch) {
				sum += p[i].type2.data;
				count++;
			}
		}
	}

	if (count == 0) return -1.0f;
	int raw = (int)(sum / count);

	if (c6_cali_ok && c6_cali_handle) {
		int mv = 0;
		if (adc_cali_raw_to_voltage(c6_cali_handle, raw, &mv) == ESP_OK) {
			return (float)mv / 1000.0f;
		}
	}
	return (float)raw * (3.3f / 4095.0f);
}

#else
// Non-C6 targets: legacy esp_adc_cal API
#include "esp_adc_cal.h"

static bool cal_ok = false;
static esp_adc_cal_characteristics_t adc1_chars;

void adc_init(void) {
	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

#ifdef HW_ADC_CH0
	adc1_config_channel_atten(HW_ADC_CH0, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH1
	adc1_config_channel_atten(HW_ADC_CH1, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH2
	adc1_config_channel_atten(HW_ADC_CH2, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH3
	adc1_config_channel_atten(HW_ADC_CH3, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH4
	adc1_config_channel_atten(HW_ADC_CH4, ADC_ATTEN_DB_12);
#endif

	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
		cal_ok = true;
	}
}

float adc_get_voltage(adc1_channel_t ch) {
	float res = -1.0;
	if (cal_ok) {
		res = (float)esp_adc_cal_raw_to_voltage(adc1_get_raw(ch), &adc1_chars) / 1000.0;
	}
	return res;
}
#endif

// SPDX-License-Identifier: GPL-3.0-or-later
// JetFleet Lisp task watchdog extensions.

#ifndef JETFLEET_WDT_H
#define JETFLEET_WDT_H

#include "sdkconfig.h"
#if CONFIG_ESP_SYSTEM_GDBSTUB_RUNTIME || \
    (!CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT && !CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT)
#error "JetFleet hardware requires reboot on panic with runtime GDBStub disabled."
#endif

#include "esp_task_wdt.h"
#include "lispbm.h"
#include <math.h>

static lbm_value wdt_result(esp_err_t result) {
	if (result == ESP_OK) return ENC_SYM_TRUE;
	lbm_set_error_reason((char *)esp_err_to_name(result));
	return ENC_SYM_EERROR;
}

static lbm_value ext_wdt_disable(lbm_value *args, lbm_uint argn) {
	(void)args;
	LBM_CHECK_ARGN(0);
	esp_err_t result = esp_task_wdt_status(NULL);
	return result == ESP_ERR_NOT_FOUND || result == ESP_ERR_INVALID_STATE ?
		ENC_SYM_TRUE : wdt_result(esp_task_wdt_delete(NULL));
}

static lbm_value ext_wdt_configure(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);
	bool enabled = args[0] == ENC_SYM_TRUE ||
		(lbm_is_number(args[0]) && lbm_dec_as_i32(args[0]) != 0);
	if (!enabled && args[0] != ENC_SYM_NIL && !lbm_is_number(args[0])) return ENC_SYM_TERROR;
	if (!enabled) return ext_wdt_disable(NULL, 0);
	if (!lbm_is_number(args[1])) return ENC_SYM_TERROR;
	double seconds = lbm_dec_as_double(args[1]);
	if (!isfinite(seconds) || seconds < 1 || seconds > UINT32_MAX / 1000U) return ENC_SYM_EERROR;
	esp_task_wdt_config_t config = {
		.timeout_ms = (uint32_t)(seconds * 1000.0),
		.trigger_panic = true,
	};
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
	config.idle_core_mask |= 1U;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
	config.idle_core_mask |= 2U;
#endif
	esp_err_t result = esp_task_wdt_reconfigure(&config);
	if (result == ESP_ERR_INVALID_STATE) result = esp_task_wdt_init(&config);
	if (result != ESP_OK) return wdt_result(result);
	return wdt_result(esp_task_wdt_status(NULL) == ESP_OK ? ESP_OK : esp_task_wdt_add(NULL));
}

static lbm_value ext_wdt_reset(lbm_value *args, lbm_uint argn) {
	(void)args;
	LBM_CHECK_ARGN(0);
	esp_err_t result = esp_task_wdt_status(NULL);
	if (result == ESP_ERR_NOT_FOUND || result == ESP_ERR_INVALID_STATE) {
		return ENC_SYM_TRUE;
	}
	return wdt_result(esp_task_wdt_reset());
}

static void jetfleet_load_wdt_extensions(void) {
	lbm_add_extension("wdt-configure", ext_wdt_configure);
	lbm_add_extension("wdt-disable", ext_wdt_disable);
	lbm_add_extension("wdt-reset", ext_wdt_reset);
}

#endif

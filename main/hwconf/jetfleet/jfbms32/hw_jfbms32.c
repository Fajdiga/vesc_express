/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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

#include HW_HEADER
#include "bq769x2_defs.h"
#include "jfbms32_safety.h"

#include "main.h"
#include "i2c_compat.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"

#include <math.h>
#include <sys/time.h>

// Settings
#define BQ_ADDR_1 0x10
#define BQ_ADDR_2 0x08
#define I2C_SPEED 100000
#define BMS_INIT_ADDR_RETRIES 3

// Bound all I2C / BQ mutex waits so a stuck holder cannot deadlock every
// caller forever. If this ever times out we bail out with an error instead of
// blocking - combined with the task WDT this is what lets a stuck bus recover.
#define I2C_MUTEX_TIMEOUT_MS 500
#define BMS_BALANCE_OFF_RETRIES 3
#define BMS_BALANCE_OFF_RETRY_DELAY_MS 20

// Macros
#define M_CELLS (m_cells_ic1 + m_cells_ic2)

// Variables
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t bq_mutex;
static unsigned int m_cells_ic1 = 16;
static unsigned int m_cells_ic2 = 16;
static uint16_t m_bal_state_ic1 = 0;
static uint16_t m_bal_state_ic2 = 0;
static portMUX_TYPE m_control_lock = portMUX_INITIALIZER_UNLOCKED;
static bms_safety_state m_control_state = {.inhibited = true};
static bool m_control_monitoring;
static TaskHandle_t m_control_watchdog_task;
// BQ mutex owns ready/lock state; the control spinlock owns the fault mask
// and I/O permission, including the final GPIO enable decision.
static uint16_t m_protection_faults;
static bool m_protection_ready, m_protection_locked, m_protection_io_ok;

// Error messages
static char *error_comm_bq1 = "BQ1 communication error";
static char *error_comm_bq2 = "BQ2 communication error";

static void bms_set_chg_hw(bool enable) {
	portENTER_CRITICAL(&m_control_lock);
	enable &= !m_protection_faults && (!m_protection_ready || m_protection_io_ok);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_CHG_EN, enable ? 1 : 0);
	portEXIT_CRITICAL(&m_control_lock);
}

static void bms_clear_balance_state(void) {
	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;
}

static esp_err_t i2c_tx_rx(
	uint8_t addr, const uint8_t *write_buffer, size_t write_size,
	uint8_t *read_buffer, size_t read_size
) {

	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t res;
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(
				0, addr, write_buffer, write_size, read_buffer, read_size, 500
			);
		} else {
			res = i2c_master_read_from_device(
				0, addr, read_buffer, read_size, 500
			);
		}
	} else {
		res =
			i2c_master_write_to_device(0, addr, write_buffer, write_size, 500);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static void i2c_bus_clear(void) {
	gpio_set_direction(PIN_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_set_direction(PIN_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_set_pull_mode(PIN_SDA, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(PIN_SCL, GPIO_PULLUP_ONLY);

	gpio_set_level(PIN_SDA, 1);
	gpio_set_level(PIN_SCL, 1);
	vTaskDelay(pdMS_TO_TICKS(1));

	for (int i = 0; i < 9 && gpio_get_level(PIN_SDA) == 0; i++) {
		gpio_set_level(PIN_SCL, 0);
		vTaskDelay(pdMS_TO_TICKS(1));
		gpio_set_level(PIN_SCL, 1);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// Generate a STOP condition in case a slave was left mid-transaction.
	gpio_set_level(PIN_SDA, 0);
	vTaskDelay(pdMS_TO_TICKS(1));
	gpio_set_level(PIN_SCL, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
	gpio_set_level(PIN_SDA, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void i2c_reinstall_driver(bool clear_bus) {
	i2c_driver_delete(0);
	if (clear_bus) {
		i2c_bus_clear();
	}

	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_SPEED,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	i2c_reset_tx_fifo(0);
	i2c_reset_rx_fifo(0);
}

static uint8_t crc8(uint8_t *ptr, uint8_t len) {
	uint8_t i;
	uint8_t crc = 0;

	while (len-- != 0) {
		for (i = 0x80; i != 0; i /= 2) {
			if ((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= 0x107;
			} else {
				crc *= 2;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x107;
			}
		}
		ptr++;
	}

	return (crc);
}

static bool bq_read_block(
	uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len
) {
	if (!buf || len == 0) return false;
	uint8_t read_data[2 * len];
	esp_err_t res          = i2c_tx_rx(dev_addr, &reg, 1, read_data, 2 * len);
	uint8_t *read_data_ptr = read_data;

	if (res != ESP_OK) {
		commands_printf_lisp("I2C Error: %d", res);
		return false;
	}

	uint8_t crcbuf[4];
	crcbuf[0]   = dev_addr << 1;
	crcbuf[1]   = reg;
	crcbuf[2]   = (dev_addr << 1) + 1;
	crcbuf[3]   = *read_data_ptr;
	uint8_t crc = crc8(crcbuf, 4);

	read_data_ptr++;
	if (crc != *read_data_ptr) {
		commands_printf_lisp("Bad CRC1");
		return false;
	} else {
		*buf = *(read_data_ptr - 1);
	}

	for (int i = 1; i < len; i++) {
		read_data_ptr++;
		crc = crc8(read_data_ptr, 1);
		read_data_ptr++;
		buf++;

		if (crc != *read_data_ptr) {
			commands_printf_lisp("Bad CRC2");
			return false;
		} else {
			*buf = *(read_data_ptr - 1);
		}
	}

	return true;
}

static bool bq_write_block(
	uint8_t dev_addr, uint8_t start_addr, uint8_t *buf, uint8_t len
) {
	if (!buf || len == 0) return false;
	uint8_t txbuf[2 * len + 2];
	txbuf[0] = dev_addr << 1;
	txbuf[1] = start_addr;
	txbuf[2] = buf[0];
	txbuf[3] = crc8(txbuf, 3);

	for (int i = 1; i < len; i++) {
		txbuf[2 + (2 * i)] = buf[i];
		txbuf[3 + (2 * i)] = crc8(&buf[i], 1);
	}

	esp_err_t res = i2c_tx_rx(dev_addr, txbuf + 1, 2 * len + 1, NULL, 0);

	return res == ESP_OK;
}

static uint8_t checksum(uint8_t *ptr, int len) {
	uint8_t sum = 0;

	for (int i = 0; i < len; i++) {
		sum += ptr[i];
	}

	return ~sum;
}

static bool bq_set_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t reg_data, uint8_t datalen
) {
	if (datalen != 1 && datalen != 2 && datalen != 4) return false;
	uint8_t payload[6] = {reg_addr & 0xff, reg_addr >> 8};
	for (unsigned i = 0; i < datalen; i++) {
		payload[i + 2] = reg_data >> (8 * i);
	}
	// Never commit a checksum after a failed payload write.
	if (!bq_write_block(dev_addr, 0x3E, payload, datalen + 2)) return false;
	vTaskDelay(2);
	uint8_t commit[2] = {checksum(payload, datalen + 2), datalen + 4};
	bool ok = bq_write_block(dev_addr, 0x60, commit, sizeof(commit));
	vTaskDelay(2);
	return ok;
}

static bool bq_read_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t *reg_data, uint8_t datalen
) {
	if (!reg_data) return false;
	*reg_data = 0;
	if (datalen == 0 || datalen > 4) return false;
	uint8_t address[2] = {reg_addr & 0xff, reg_addr >> 8};
	uint8_t data[4] = {0};
	// A failed select must not return data from the previous register.
	if (!bq_write_block(dev_addr, 0x3E, address, sizeof(address))) return false;
	vTaskDelay(2);
	if (!bq_read_block(dev_addr, 0x40, data, datalen)) return false;
	for (unsigned i = 0; i < datalen; i++) {
		*reg_data |= (uint32_t)data[i] << (8 * i);
	}
	return true;
}

// Called under bq_mutex; retry the entire write/readback transaction.
static bool bq_set_verified(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t reg_data, uint8_t datalen
) {
	if (datalen != 1 && datalen != 2 && datalen != 4) return false;
	uint32_t mask = UINT32_MAX >> (8 * (4 - datalen));
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		uint32_t actual = 0;
		if (bq_set_reg(dev_addr, reg_addr, reg_data, datalen) &&
				bq_read_reg(dev_addr, reg_addr, &actual, datalen) &&
				actual == (reg_data & mask)) return true;
	}
	commands_printf_lisp("BQ configuration readback failed: 0x%04x", reg_addr);
	return false;
}

static int16_t command_read(uint8_t dev_addr, uint8_t command, bool *ok) {
	if (ok) {
		*ok = false;
	}
	uint8_t RX_data[2] = {0, 0};
	if (bq_read_block(dev_addr, command, RX_data, 2)) {
		if (ok) {
			*ok = true;
		}
		return (int16_t)(((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0]);
	} else {
		return -1;
	}
}

static bool command_subcommands(uint8_t dev_addr, uint16_t command) {
	// For DEEPSLEEP/SHUTDOWN subcommand you will need to
	// call this function twice consecutively

	uint8_t TX_Reg[2] = {0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 2);
	vTaskDelay(2);
	return res;
}

static bool subcommands_write16(
	uint8_t dev_addr, uint16_t command, uint16_t data
) {
	uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;
	TX_Reg[2] = data & 0xff;
	TX_Reg[3] = (data >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 4);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	TX_Reg[0] = checksum(TX_Reg, 4);
	TX_Reg[1] = 0x06;

	res = bq_write_block(dev_addr, 0x60, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	return true;
}

static bool bms_disable_balancing_hw(bool include_ic2) {
	bms_clear_balance_state();

	for (int i = 0; i < BMS_BALANCE_OFF_RETRIES; i++) {
		bool res1 = subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, 0);
		bool res2 = true;

		if (include_ic2) {
			res2 = subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, 0);
		}

		if (res1 && res2) {
			return true;
		}

		vTaskDelay(pdMS_TO_TICKS(BMS_BALANCE_OFF_RETRY_DELAY_MS));
	}

	return false;
}

static bool bms_fail_close_outputs_hw(bool include_ic2) {
	bms_set_chg_hw(false);
	return bms_disable_balancing_hw(include_ic2);
}

// Called with bq_mutex held. ALL_FETS_OFF is supported with DDSG; do not
// use DSG_PDSG_OFF or FET_CONTROL, which TI excludes in DDSG mode.
static bool bms_protection_hold_hw(void) {
	bms_set_chg_hw(false); // Immediate MCU cutoff, before any I2C operation.
	if (!m_protection_ready) return true;
	if (!m_protection_locked) {
		m_protection_locked = command_subcommands(BQ_ADDR_1, ALL_FETS_OFF);
	}
	return m_protection_locked;
}

static uint16_t bms_protection_fault_mask(void) {
	portENTER_CRITICAL(&m_control_lock);
	uint16_t faults = m_protection_faults;
	portEXIT_CRITICAL(&m_control_lock);
	return faults;
}

static void bms_protection_latch(uint16_t faults) {
	portENTER_CRITICAL(&m_control_lock);
	m_protection_faults |= faults & BMS_CURRENT_FAULT_MASK;
	portEXIT_CRITICAL(&m_control_lock);
	if (bms_protection_fault_mask()) (void)bms_protection_hold_hw();
}

static bool bms_protection_read_hw(uint16_t *faults) {
	bool ok_c = false, ok_a = false;
	// Read watchdog status first: any valid communication starts its recovery.
	uint8_t c = command_read(BQ_ADDR_1, SafetyStatusC, &ok_c);
	uint8_t a = command_read(BQ_ADDR_1, SafetyStatusA, &ok_a);
	portENTER_CRITICAL(&m_control_lock);
	m_protection_io_ok = ok_a && ok_c;
	portEXIT_CRITICAL(&m_control_lock);
	if (!ok_a || !ok_c) {
		bms_set_chg_hw(false);
		return false;
	}
	*faults = ((uint16_t)c << 8 | a) & BMS_CURRENT_FAULT_MASK;
	return true;
}

static bool bms_protection_poll_hw(void) {
	uint16_t faults = 0;
	bool ok = bms_protection_read_hw(&faults);
	if (ok) bms_protection_latch(faults);
	return ok;
}

// Opt-in supervision for the normal control script. Raw hardware extensions
// neither arm nor renew this timer and remain usable without a control script.
static uint32_t control_time_ms(void) {
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void control_watchdog_disarm(void) {
	portENTER_CRITICAL(&m_control_lock);
	m_control_monitoring = false;
	bms_safety_inhibit(&m_control_state, false);
	portEXIT_CRITICAL(&m_control_lock);
}

static bool control_watchdog_tripped(void) {
	portENTER_CRITICAL(&m_control_lock);
	if (m_control_monitoring && bms_safety_expired(&m_control_state, control_time_ms())) {
		bms_safety_inhibit(&m_control_state, false);
	}
	bool tripped = m_control_monitoring && m_control_state.inhibited;
	portEXIT_CRITICAL(&m_control_lock);
	return tripped;
}

static void control_watchdog_task(void *arg) {
	(void)arg;
	bool balance_off_verified = false;
	bool reported = false;
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(BMS_PROTECTION_POLL_MS));
		// Reuse this task for current protection, independently of the Lisp
		// scan. Contention can delay a software cutoff; OCD/SCD stay autonomous.
		if (xSemaphoreTake(bq_mutex, 0) == pdTRUE) {
			if (m_control_monitoring && m_protection_ready) {
				int com_prev = gpio_get_level(PIN_COM_EN);
				gpio_set_level(PIN_COM_EN, 0);
				(void)bms_protection_poll_hw();
				gpio_set_level(PIN_COM_EN, com_prev);
			}
			xSemaphoreGive(bq_mutex);
		}
		if (!control_watchdog_tripped()) {
			balance_off_verified = false;
			reported = false;
			continue;
		}
		// Charge shutdown never waits for an I2C mutex or a failed balance write.
		bms_set_chg_hw(false);
		if (!reported) {
			commands_printf("BMS control timeout: charge off, stopping balancing");
			reported = true;
		}
		if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) continue;
		// A supervised restart may have completed while this task waited.
		if (control_watchdog_tripped()) {
			bms_set_chg_hw(false);
			if (!balance_off_verified || m_bal_state_ic1 || m_bal_state_ic2) {
				int com_prev = gpio_get_level(PIN_COM_EN);
				gpio_set_level(PIN_COM_EN, 0);
				balance_off_verified = bms_disable_balancing_hw(m_cells_ic2 != 0);
				gpio_set_level(PIN_COM_EN, com_prev);
			}
		}
		xSemaphoreGive(bq_mutex);
	}
}

static lbm_value ext_control_start(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0 || !m_control_watchdog_task) return ENC_SYM_EERROR;
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) return ENC_SYM_EERROR;
	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool off = bms_fail_close_outputs_hw(m_cells_ic2 != 0);
	gpio_set_level(PIN_COM_EN, com_prev);
	if (off) {
		portENTER_CRITICAL(&m_control_lock);
		bms_safety_ready(&m_control_state);
		(void)bms_safety_feed(&m_control_state, control_time_ms(), m_control_state.generation);
		m_control_monitoring = true;
		portEXIT_CRITICAL(&m_control_lock);
	}
	xSemaphoreGive(bq_mutex);
	return off ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_control_feed(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0) return ENC_SYM_EERROR;
	portENTER_CRITICAL(&m_control_lock);
	bool ok = m_control_monitoring && bms_safety_feed(&m_control_state,
			control_time_ms(), m_control_state.generation);
	portEXIT_CRITICAL(&m_control_lock);
	return ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// A check never feeds the timer. Used by the normal script before enabling
// outputs, so a delayed scan or the balancing worker cannot undo a timeout.
static lbm_value ext_control_ok(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0) return ENC_SYM_EERROR;
	return control_watchdog_tripped() ? ENC_SYM_NIL : ENC_SYM_TRUE;
}

// Stop the normal script before using this opt-out for manual hardware tests.
static lbm_value ext_control_stop(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0) return ENC_SYM_EERROR;
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) return ENC_SYM_EERROR;
	control_watchdog_disarm();
	xSemaphoreGive(bq_mutex);
	return ENC_SYM_TRUE;
}

static lbm_value ext_protection_status(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0) return ENC_SYM_EERROR;
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_set_chg_hw(false);
		return ENC_SYM_EERROR;
	}
	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool ok = m_protection_ready && bms_protection_poll_hw();
	uint16_t faults = bms_protection_fault_mask();
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);
	return ok ? lbm_enc_i(faults) : ENC_SYM_EERROR;
}

static lbm_value ext_protection_lock(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	uint32_t faults = lbm_dec_as_u32(args[0]);
	if (faults & ~BMS_CURRENT_FAULT_MASK) return ENC_SYM_EERROR;
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_set_chg_hw(false);
		return ENC_SYM_EERROR;
	}
	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bms_protection_latch(faults);
	bool ok = !bms_protection_fault_mask() || bms_protection_hold_hw();
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);
	return ok ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// Only the script's explicit Chg En recovery calls this. The MCU gate stays
// off throughout. A failed release reinstates ALL_FETS_OFF and needs a new press.
static lbm_value ext_protection_reset(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0) return ENC_SYM_EERROR;
	bms_set_chg_hw(false);
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) return ENC_SYM_EERROR;
	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	uint16_t live = 0;
	bool ok = m_protection_ready && bms_protection_read_hw(&live) && !live;
	for (unsigned reg = PFStatusA; ok && reg <= PFStatusD; reg += 2) {
		bool read_ok = false;
		uint8_t pf = command_read(BQ_ADDR_1, reg, &read_ok);
		ok = read_ok && !pf;
	}
	if (ok) {
		bool read_ok = false;
		int16_t current = command_read(BQ_ADDR_1, CC2Current, &read_ok);
		ok = read_ok && current >= -20 && current <= 20; // 10 mA/count
	}
	if (ok) ok = command_subcommands(BQ_ADDR_1, ALL_FETS_ON);
	bool released = false;
	// TI evaluates turn-on every 250 ms in NORMAL mode. Keep the GPIO off
	// while checking the actual DDSG signal, rather than trusting an ACK.
	for (unsigned attempt = 0; ok && attempt < 31; attempt++) {
		bool read_ok = false;
		uint8_t status = command_read(BQ_ADDR_1, FETStatus, &read_ok);
		ok = read_ok;
		if (ok && !(status & 0x20)) { released = true; break; }
		if (ok) vTaskDelay(pdMS_TO_TICKS(10));
	}
	ok = ok && released && bms_protection_read_hw(&live) && !live;
	if (ok) {
		portENTER_CRITICAL(&m_control_lock);
		m_protection_faults = 0;
		portEXIT_CRITICAL(&m_control_lock);
		m_protection_locked = false;
	} else {
		bms_protection_latch(live);
		m_protection_locked = false;
		(void)bms_protection_hold_hw();
	}
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);
	return lbm_enc_i(ok ? 1 : 0);
}

static uint32_t float_to_u(float number) {
	// Set subnormal numbers to 0 as they are not handled properly
	// using this method.
	if (fabsf(number) < 1.5e-38) {
		number = 0.0;
	}

	int e          = 0;
	float sig      = frexpf(number, &e);
	float sig_abs  = fabsf(sig);
	uint32_t sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e    += 126;
	}

	uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1U << 31;
	}

	return res;
}

static bool cell_counts_valid(unsigned int cells_ic1, unsigned int cells_ic2) {
	return cells_ic1 >= 3 && cells_ic1 <= 16 &&
			(cells_ic2 == 0 || (cells_ic2 >= 3 && cells_ic2 <= 16));
}

static uint8_t overcurrent_threshold_from_a(float current, uint8_t maximum) {
	// BQ76952 OCC/OCD thresholds are shunt voltage in 2 mV steps.
	float shunt_mv = current * HW_R_SHUNT * 1000.0f;
	// Clamp before converting: even a huge finite setting must not overflow int.
	return (uint8_t)fminf(fmaxf(shunt_mv / 2.0f + 0.5f, 2.0f), maximum);
}

static bool current_protection_config_valid(const main_config_t *cfg) {
	return isfinite(cfg->hw_occ_current) && cfg->hw_occ_current >= 4.0f &&
			cfg->hw_occ_current <= 124.0f && isfinite(cfg->hw_ocd_current) &&
			cfg->hw_ocd_current >= 4.0f && cfg->hw_ocd_current <= 200.0f &&
			cfg->psw_scd_tres >= 0 && cfg->psw_scd_tres <= 15;
}

static float ntc_nominal_resistance(NTC_RES selection) {
	static const float resistance[] = {
		4700, 5000, 10000, 20000, 22000, 47000, 50000, 100000, 200000
	};
	return (unsigned)selection < sizeof(resistance) / sizeof(resistance[0])
			? resistance[selection] : 0.0f;
}

static float ntc_pullup_resistance(NTC_RES selection) {
	return selection == NTC_RES_100K || selection == NTC_RES_200K ? 180000 : 18000;
}

static bool bq_init(uint8_t dev_addr, bool current_protection_en) {
	if (!command_subcommands(dev_addr, EXIT_DEEPSLEEP)) return false;
	if (!command_subcommands(dev_addr, EXIT_DEEPSLEEP)) return false;
	vTaskDelay(10);

	if (!command_subcommands(dev_addr, SET_CFGUPDATE)) return false;
	if (!command_subcommands(dev_addr, SET_CFGUPDATE)) return false;

	bool ok = true;

	// DPSLP_OT: 1
	// SHUT_TS2: 0
	// DPSLP_PD: 0
	// DPSLP_LDO: 1
	// DPSLP_LFO: 1
	// SLEEP: 0
	// OTSD: 1
	// FASTADC: 0
	// CB_LOOP_SLOW: 0
	// LOOP_SLOW: 0
	// WK_SPD: 0
	ok &= bq_set_verified(dev_addr, PowerConfig, 0b0010011010000000, 2);
	// Sometimes the first write has no effect. Do a few extra writes just in case...
	ok &= bq_set_verified(dev_addr, PowerConfig, 0b0010011010000000, 2);

	// REG0_EN: 1
	ok &= bq_set_verified(dev_addr, REG0Config, 0x01, 1);

	// REG1V: 6 (3.3v)
	// REG1_EN: 1
	ok &= bq_set_verified(dev_addr, REG12Config, 0b00001101, 1);

	// Disabled
	ok &= bq_set_verified(dev_addr, CFETOFFPinConfig, 0x00, 1);
	ok &= bq_set_verified(dev_addr, DFETOFFPinConfig, 0x00, 1);

	// ADC inputs with 18k pull-up
	main_config_t *cfg = (main_config_t *)&backup.config;
	if (ntc_nominal_resistance(cfg->temp_res) == 0.0f ||
			(cfg->temp_num > 0 && cfg->temp_beta == 0) ||
			!isfinite(cfg->max_charge_current) || cfg->max_charge_current < 0.0f ||
			!current_protection_config_valid(cfg)) {
		(void)command_subcommands(dev_addr, EXIT_CFGUPDATE);
		return false;
	}
	// Raw thermistor ADC: 18k pull-up, or 180k for 100k/200k sensors.
	uint32_t ntcPinConfig = ntc_pullup_resistance(cfg->temp_res) == 180000 ? 0x7b : 0x3b;

	ok &= bq_set_verified(dev_addr, TS1Config, ntcPinConfig, 1);
	ok &= bq_set_verified(dev_addr, TS3Config, ntcPinConfig, 1);
	ok &= bq_set_verified(dev_addr, ALERTPinConfig, ntcPinConfig, 1);
	ok &= bq_set_verified(dev_addr, DCHGPinConfig, ntcPinConfig, 1);
	ok &= bq_set_verified(dev_addr, HDQPinConfig, 0b00111011, 1);

	if (current_protection_en) {
		// The gate named "charge" controls the shared charge/discharge FET pair.
		// DDSG is routed to its disable circuit. Configure it as an
		// active-low hardware fault output. BQ voltage protections are intentionally
		// left disabled here because only the lower BQ DDSG is wired; software
		// charge control uses both BQ ICs for cell-voltage cutoff.
		ok &= bq_set_verified(dev_addr, DDSGPinConfig, 0x82, 1);
	} else {
		ok &= bq_set_verified(dev_addr, DDSGPinConfig, 0x00, 1);
	}

	// Use all cells
	ok &= bq_set_verified(dev_addr, VCellMode, 0x0000, 2);

	// The main charge/discharge MOSFETs are driven by external hardware, not
	// the BQ high-side FET drivers. Keep the BQ charge pump off.
	ok &= bq_set_verified(dev_addr, ChgPumpControl, 0x00, 1);

	ok &= bq_set_verified(dev_addr, MfgStatusInit, 0x10, 1); // FET_EN
	if (current_protection_en) {
		// Configure BQ1 current protections. Software still owns normal charge
		// control and all voltage limits. BQ1 DDSG inhibits the shared FET gate,
		// stopping both charge and discharge. BQ internal CHG/DSG fault routing
		// is separate from that external wiring: OCC cannot be added to the DSG
		// mask (bit 4 is reserved). The script disables the shared gate on OCC.
		ok &= bq_set_verified(dev_addr, OCCThreshold, overcurrent_threshold_from_a(cfg->hw_occ_current, 62), 1);
		ok &= bq_set_verified(dev_addr, OCCDelay, 1, 1);
		ok &= bq_set_verified(dev_addr, OCD1Threshold, overcurrent_threshold_from_a(cfg->hw_ocd_current, 100), 1);
		ok &= bq_set_verified(dev_addr, OCD1Delay, 1, 1);
		ok &= bq_set_verified(dev_addr, SCDThreshold, cfg->psw_scd_tres, 1);
		ok &= bq_set_verified(dev_addr, SCDDelay, 1, 1); // no extra delay
		ok &= bq_set_verified(dev_addr, SCDRecoveryTime, 5, 1);
		ok &= bq_set_verified(dev_addr, SCDLLatchLimit, 1, 1);
		ok &= bq_set_verified(dev_addr, SCDLCounterDecDelay, 1, 1);
		// Allow native OCC/OCD recovery at idle; software keeps the fault
		// latched until a fresh Chg En request. BQ current is positive charging.
		ok &= bq_set_verified(dev_addr, OCCRecoveryThreshold, 200, 2);
		ok &= bq_set_verified(dev_addr, OCDRecoveryThreshold, (uint16_t)-200, 2);
		ok &= bq_set_verified(dev_addr, ProtectionsRecoveryTime, 3, 1);
		ok &= bq_set_verified(dev_addr, ProtectionConfiguration, 0x0002, 2);
		// Keep TI's fast-turnoff CHG mask value. Voltage faults are disabled
		// globally below, so only the enabled current protections can act.
		ok &= bq_set_verified(dev_addr, CHGFETProtectionsA, 0x98, 1); // TI fast CHG mask
		ok &= bq_set_verified(dev_addr, DSGFETProtectionsA, 0xE4, 1); // TI fast DSG mask; OCC is not a DSG bit
		ok &= bq_set_verified(dev_addr, EnabledProtectionsA, 0xB0, 1); // SCD + OCD1 + OCC
		ok &= bq_set_verified(dev_addr, HWDDelay, BMS_BQ_HWD_SECONDS, 2);
		ok &= bq_set_verified(dev_addr, HWDRegulatorOptions, 0x00, 1); // Keep MCU power on
		ok &= bq_set_verified(dev_addr, EnabledProtectionsC, 0x42, 1); // SCDL + HWDF
		ok &= bq_set_verified(dev_addr, CHGFETProtectionsC, 0x42, 1);
		ok &= bq_set_verified(dev_addr, DSGFETProtectionsC, 0x42, 1);
	} else {
		// BQ2 has no current shunt and no DDSG charge-gate path on this board.
		ok &= bq_set_verified(dev_addr, CHGFETProtectionsA, 0x00, 1);
		ok &= bq_set_verified(dev_addr, DSGFETProtectionsA, 0x00, 1);
		ok &= bq_set_verified(dev_addr, EnabledProtectionsA, 0x00, 1);
		ok &= bq_set_verified(dev_addr, EnabledProtectionsC, 0x00, 1);
		ok &= bq_set_verified(dev_addr, CHGFETProtectionsC, 0x00, 1);
		ok &= bq_set_verified(dev_addr, DSGFETProtectionsC, 0x00, 1);
	}
	ok &= bq_set_verified(dev_addr, EnabledProtectionsB, 0x00, 1);

	// Host-controlled balancing
	ok &= bq_set_verified(dev_addr, BalancingConfiguration, 0x00, 1);

	// Current gain
	float cc_gain = 7.4768 / (HW_R_SHUNT * 1000.0);
	ok &= bq_set_verified(dev_addr, CCGain, float_to_u(cc_gain), 4);
	ok &= bq_set_verified(dev_addr, CapacityGain, float_to_u(cc_gain * 298261.6178), 4);

	// Voltage and current reporting, 1 mV and 10 mA (range +- 320A)
	ok &= bq_set_verified(dev_addr, DAConfiguration, 0b00011110, 1);

	ok &= command_subcommands(dev_addr, EXIT_CFGUPDATE);

	vTaskDelay(10);

	return ok && command_subcommands(dev_addr,
			current_protection_en && bms_protection_fault_mask() ? ALL_FETS_OFF : ALL_FETS_ON) &&
			command_subcommands(dev_addr, SLEEP_DISABLE);
}

// Extensions

// ext_bms_init - bring up the dual-BQ76952 stack on a single I2C bus.
//
// At cold boot both chips power up at the BQ76952 default address 0x08.
// To run them on the same bus we move BQ1 to 0x10, leaving BQ2 at 0x08.
// PIN_COM_EN is wired so that pulling it HIGH silences BQ2's I2C interface,
// giving us a window in which only BQ1 is reachable on the bus.
//
// Naming caveat - read carefully:
//   BQ_ADDR_1 = 0x10 - BQ1's address AFTER it has been re-addressed.
//   BQ_ADDR_2 = 0x08 - BQ2's address (always), AND BQ1's default address.
// During the address-change window below we talk to BQ1 using BQ_ADDR_2
// (= 0x08), because BQ1 hasn't moved yet and BQ2 is silenced via COM_EN.
// So `bq_init(BQ_ADDR_2)` followed by `bq_set_reg(BQ_ADDR_2, I2CAddress, ...)`
// is initialising BQ1, not BQ2. This is confusing but correct.
//
// Phases:
//   1. PIN_COM_EN = HIGH - silence BQ2.
//   2. Reinstall the I2C driver to recover from any wedged bus state.
//   3. RESET BQ1 at BQ_ADDR_1 (works on warm boot when BQ1 is at 0x10;
//      NAKs harmlessly on cold boot when BQ1 is already at 0x08). After
//      RESET, BQ1 is guaranteed to be at default 0x08 either way.
//   4. Configure BQ1 at BQ_ADDR_2 (0x08), write I2CAddress = 0x20 so that
//      EXIT_CFGUPDATE moves BQ1 to 0x10.
//   5. PIN_COM_EN = LOW - bring BQ2 back on the bus, now at 0x08 with no
//      address conflict (BQ1 is at 0x10).
//   6. Configure BQ2 at BQ_ADDR_2 (0x08).
static lbm_value ext_bms_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	unsigned int cells_ic1 = 16;
	if (argn >= 1) {
		cells_ic1 = lbm_dec_as_u32(args[0]);
	}

	unsigned int cells_ic2 = 16;
	if (argn >= 2) {
		cells_ic2 = lbm_dec_as_u32(args[1]);
	}

	if (!cell_counts_valid(cells_ic1, cells_ic2)) {
		lbm_set_error_reason("Invalid cell combination");
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);
	bms_clear_balance_state();

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}
	portENTER_CRITICAL(&m_control_lock);
	m_protection_ready = false;
	m_protection_io_ok = false;
	portEXIT_CRITICAL(&m_control_lock);
	m_protection_locked = false;

	gpio_set_level(PIN_COM_EN, 0);
	(void)bms_disable_balancing_hw(m_cells_ic2 != 0 || cells_ic2 != 0);

	// i2c_tx_rx() serializes through i2c_mutex, but i2c_driver_delete /
	// _install bypass it. If any other caller (Lisp i2c-tx-rx, i2c-detect,
	// future hardware) is mid-transfer on port 0 while we tear it down, the
	// driver state goes invalid. Hold both mutexes across the reinstall.
	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("i2c_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}

	// Disable COMM until the i2c-address of the first BQ
	// is changed.
	gpio_set_level(PIN_COM_EN, 1);

	// Fast path: only reinstall the ESP I2C driver. Bus-clear clocking is
	// reserved for retry recovery after a real BQ communication failure.
	i2c_reinstall_driver(false);

	xSemaphoreGive(i2c_mutex);

	vTaskDelay(50);

	// Wake BQ1 if it was put into BQ-level DEEPSLEEP by a previous
	// bms-sleep. The BQ I2C interface stays alive in DEEPSLEEP listening
	// for EXIT_DEEPSLEEP at the chip's address (which persists through
	// sleep - BQ1 stays at 0x10 once moved). Cold-boot fresh chips are
	// still at 0x08, so this NAKs harmlessly.
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	vTaskDelay(pdMS_TO_TICKS(10));

	// Probe BQ1 at its post-init address (0x10) first. If it ACKs, the
	// address change has already been done - either from a previous boot
	// in this session or because the new I2CAddress is committed to data
	// flash and survives BQ769x2_RESET. In that case the old code path
	// (RESET -> write I2CAddress at 0x08) NAKs forever because nothing is
	// at 0x08 (BQ2 silenced via COM_EN, BQ1 still at 0x10) - which is the
	// "Could not update I2C address" loop seen on warm boots.
	//
	// COM_EN=1 here means BQ2 is silenced, so an ACK at 0x10 can only be
	// from BQ1. Use a single direct command read; no state changes if it
	// fails.
	bool bq1_at_target = false;
	command_read(BQ_ADDR_1, Cell1Voltage, &bq1_at_target);

	if (!bq1_at_target) {
		bool addr_update_ok = false;

		for (int attempt = 0; attempt < BMS_INIT_ADDR_RETRIES; attempt++) {
			if (attempt > 0) {
				if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
					i2c_reinstall_driver(true);
					xSemaphoreGive(i2c_mutex);
				}

				gpio_set_level(PIN_COM_EN, 1);
				vTaskDelay(pdMS_TO_TICKS(50));
			}

			// Cold-boot / DF-default path: BQ1 expected at default 0x08.
			// BQ76952 datasheet allows up to ~260 ms for RESET to complete; a
			// shorter delay can leave the chip half-reset and wedge the bus.
			// Do not send SWAP_COMM_MODE at 0x10 here: after RESET the chip is
			// expected at its default address, so that old call only NAKed.
			command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_2, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_2, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_1, BQ769x2_RESET);
			vTaskDelay(pdMS_TO_TICKS(300));

			if (!bq_init(BQ_ADDR_2, true) ||
					!command_subcommands(BQ_ADDR_2, SET_CFGUPDATE)) continue;
			if (!bq_set_reg(BQ_ADDR_2, I2CAddress, 0x20, 1)) {
				continue;
			}

			command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE);
			// I2CAddress is applied on reset or SWAP_COMM_MODE. We are already
			// in I2C mode; this makes the new address take effect immediately.
			command_subcommands(BQ_ADDR_2, SWAP_COMM_MODE);
			vTaskDelay(pdMS_TO_TICKS(20));

			command_read(BQ_ADDR_1, Cell1Voltage, &addr_update_ok);
			if (addr_update_ok) {
				break;
			}
		}

		if (!addr_update_ok) {
			lbm_set_error_reason("BQ1 I2C address recovery failed");
			xSemaphoreGive(bq_mutex);
			return ENC_SYM_NIL;
		}
	}

	// Enable the other i2c now that BQ1's address is settled at 0x10
	gpio_set_level(PIN_COM_EN, 0);
	vTaskDelay(50);

	bool configured = cells_ic2 == 0 || bq_init(BQ_ADDR_2, false);

	// Always init BQ1 at its final address so its config is fresh on
	// every bms-init, regardless of which path we took above.
	configured &= bq_init(BQ_ADDR_1, true);

	m_cells_ic1 = cells_ic1;
	m_cells_ic2 = cells_ic2;

	if (!configured || !bms_disable_balancing_hw(cells_ic2 != 0)) {
		lbm_set_error_reason("BQ configuration or balance shutdown failed");
		xSemaphoreGive(bq_mutex);
		return ENC_SYM_NIL;
	}

	bool res = false;
	command_read(BQ_ADDR_1, Cell2Voltage, &res);
	if (m_cells_ic2 > 0) {
		bool res2 = false;
		command_read(BQ_ADDR_2, Cell2Voltage, &res2);
		res = res && res2;
	}

	portENTER_CRITICAL(&m_control_lock);
	m_protection_ready = res;
	portEXIT_CRITICAL(&m_control_lock);
	if (res) res = bms_protection_poll_hw();
	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_hw_sleep(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	// Sleep entry is rare and not latency-critical. Use a generous timeout
	// (5 s vs the 500 ms default) so transient I2C contention doesn't cause
	// us to skip BQ DEEPSLEEP - that would leave each BQ in active mode
	// (~mA) drawing only from its top cell, causing progressive top-cell
	// imbalance over weeks of 2 h-cycle wakes. Return EERROR (not NIL) so
	// Lisp `with-com` retries via its trap loop instead of silently letting
	// the caller proceed to sleep-deep with the BQs still active.
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-sleep");
		return ENC_SYM_EERROR;
	}

	// Pre-flight: confirm both BQs respond at their expected addresses
	// BEFORE we disable switches and start writing sleep config. If BQ1
	// somehow ended up at the default 0x08 (e.g. a previous bms-init
	// partially failed), DEEPSLEEP at 0x10 would NAK while DEEPSLEEP at
	// 0x08 might put the wrong chip to sleep - and we'd discover this
	// only after switches are off and TS pins reconfigured, leaving the
	// device in a half-prepared-for-sleep state. Bail early and let
	// Lisp's recovery path re-run init-hw to re-establish addresses.
	{
		bool probe_ok = false;
		command_read(BQ_ADDR_1, Cell1Voltage, &probe_ok);
		if (!probe_ok) {
			xSemaphoreGive(bq_mutex);
			lbm_set_error_reason("BQ1 not responsive at 0x10 before sleep");
			return ENC_SYM_EERROR;
		}
		if (m_cells_ic2 != 0) {
			probe_ok = false;
			command_read(BQ_ADDR_2, Cell1Voltage, &probe_ok);
			if (!probe_ok) {
				xSemaphoreGive(bq_mutex);
				lbm_set_error_reason("BQ2 not responsive at 0x08 before sleep");
				return ENC_SYM_EERROR;
			}
		}
	}

	// Disable all switches
	gpio_set_level(PIN_OUT_EN, 0);
	bms_set_chg_hw(false);
	gpio_set_level(PIN_PSW_EN, 0);

	// Stop balancing
	if (!bms_disable_balancing_hw(m_cells_ic2 != 0)) {
		goto exit_error1;
	}

	// Disable temperature measurement pull-ups and ensure that regulator is kept on in DEEP SLEEP

	if (!command_subcommands(BQ_ADDR_1, SET_CFGUPDATE) ||
		!bq_set_reg(BQ_ADDR_1, PowerConfig, 0b0010011010000000, 2) ||
		!bq_set_reg(BQ_ADDR_1, TS1Config, 0x00, 1) ||
		!bq_set_reg(BQ_ADDR_1, TS3Config, 0x00, 1) ||
		!command_subcommands(BQ_ADDR_1, EXIT_CFGUPDATE)) {
		goto exit_error1;
	}

	if (m_cells_ic2 != 0) {
		if (!command_subcommands(BQ_ADDR_2, SET_CFGUPDATE) ||
			!bq_set_reg(BQ_ADDR_2, PowerConfig, 0b0010011010000000, 2) ||
			!bq_set_reg(BQ_ADDR_2, TS1Config, 0x00, 1) ||
			!bq_set_reg(BQ_ADDR_2, TS3Config, 0x00, 1) ||
			!command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE)) {
				goto exit_error2;
			}
	}

	// DEEPSLEEP requires the subcommand to be sent twice in succession
	// (TI safety pattern: first call arms, second commits). Error-check
	// both: a NAK here means the chip stayed in ACTIVE mode (~mA draw
	// from the top cell) while ESP goes to deep sleep - silent battery
	// damage. Surface failures so Lisp doesn't proceed to sleep-deep.
	if (!command_subcommands(BQ_ADDR_1, DEEPSLEEP) ||
		!command_subcommands(BQ_ADDR_1, DEEPSLEEP)) {
		goto exit_error1;
	}

	if (m_cells_ic2 != 0) {
		if (!command_subcommands(BQ_ADDR_2, DEEPSLEEP) ||
			!command_subcommands(BQ_ADDR_2, DEEPSLEEP)) {
			goto exit_error2;
		}
	}

	// Disable CAN-bus and other COMM
	gpio_set_level(PIN_COM_EN, 1);
	// Successful intentional sleep must not make the watchdog wake the BQs.
	control_watchdog_disarm();

	xSemaphoreGive(bq_mutex);
	return ENC_SYM_TRUE;

exit_error1:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static bool bms_shutdown_bq(uint8_t addr) {
	// SHUTDOWN requires the subcommand to be sent twice in succession.
	return command_subcommands(addr, SHUTDOWN) &&
			command_subcommands(addr, SHUTDOWN);
}

#ifndef SHUTDOWN_SUPPORT
static void bms_config_shutdown_wakeup(void) {
	gpio_set_direction(PIN_ENABLE, GPIO_MODE_INPUT);

#if CONFIG_IDF_TARGET_ESP32S3
	esp_sleep_enable_ext0_wakeup(PIN_ENABLE, 1);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
	esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
			1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
#error "Unsupported target"
#endif
}

#endif

static lbm_value ext_bms_hw_shutdown(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);
	bms_clear_balance_state();

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-hw-shutdown");
		return ENC_SYM_EERROR;
	}

	gpio_set_level(PIN_COM_EN, 0);
	(void)bms_disable_balancing_hw(m_cells_ic2 != 0);

	if (m_cells_ic2 != 0 && !bms_shutdown_bq(BQ_ADDR_2)) {
		(void)bms_fail_close_outputs_hw(m_cells_ic2 != 0);
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("BQ2 shutdown command failed");
		return ENC_SYM_EERROR;
	}

	if (!bms_shutdown_bq(BQ_ADDR_1)) {
		(void)bms_fail_close_outputs_hw(m_cells_ic2 != 0);
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("BQ1 shutdown command failed");
		return ENC_SYM_EERROR;
	}
	control_watchdog_disarm();

	xSemaphoreGive(bq_mutex);

	vTaskDelay(pdMS_TO_TICKS(1000));

#ifdef SHUTDOWN_SUPPORT
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
	gpio_deep_sleep_hold_dis();
#endif
	// Disable ESP GPIO hold so the open-drain shutdown output can change
	// from idle/released high to asserted low.
	gpio_hold_dis(PIN_SHUTDOWN);
	gpio_set_level(PIN_SHUTDOWN, 0);

	vTaskDelay(pdMS_TO_TICKS(10000));
	bms_set_chg_hw(false);
	bms_clear_balance_state();
	lbm_set_error_reason("BMS shutdown pin did not power off");
	return ENC_SYM_EERROR;
#else
	bms_config_shutdown_wakeup();
	gpio_set_level(PIN_COM_EN, 1);
	esp_deep_sleep_start();

	return ENC_SYM_TRUE;
#endif
}

static lbm_value ext_get_vcells_unlocked(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value vc_list = ENC_SYM_NIL;

	for (int i = 0; i < m_cells_ic1; i++) {
		bool ok = false;
		int res = command_read(BQ_ADDR_1, Cell1Voltage + i * 2, &ok);
		if (ok) {
			vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
		} else {
			lbm_set_error_reason(error_comm_bq1);
			return ENC_SYM_EERROR;
		}
	}

	for (int i = 0; i < m_cells_ic2; i++) {
		bool ok = false;
		int res = command_read(BQ_ADDR_2, Cell1Voltage + i * 2, &ok);
		if (ok) {
			vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
		} else {
			lbm_set_error_reason(error_comm_bq2);
			return ENC_SYM_EERROR;
		}
	}

	return lbm_list_destructive_reverse(vc_list);
}

static lbm_value ext_cell0_report_offset(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	float offset = 0.0;
#ifndef SHUTDOWN_SUPPORT
	float current = lbm_dec_as_float(args[0]);
	if (current < 0.0) {
		offset = 0.003 * fabsf(current);
		utils_truncate_number(&offset, 0.0, 0.015);
	}
#endif

	return lbm_enc_float(offset);
}

// Keep invalid sensors distinguishable from a real subzero temperature.
// The control script validates every required sensor before using extrema.
static float ntc_temperature(float volts, float pullup, float nominal, float beta) {
	float temperature = bms_ntc_temperature(volts, pullup, nominal, beta);
	return bms_temperature_valid(temperature) ? temperature : -273.0f;
}

static lbm_value ext_get_temps_unlocked(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	// BQ1 IC, four external sensors, BQ1 MOS, BQ2 IC, BQ2 MOS.
	static const uint8_t sensors[] = {
		TS1Temperature, TS3Temperature, ALERTTemperature, DCHGTemperature, HDQTemperature
	};
	const main_config_t *cfg = (const main_config_t *)&backup.config;
	const float nominal = ntc_nominal_resistance(cfg->temp_res);
	const float pullup = ntc_pullup_resistance(cfg->temp_res);
	const float counts_to_volts = 0.358e-6 * 256.0; // 16 of 24 ADC bits are used
	lbm_value temperatures = ENC_SYM_NIL;

	for (unsigned ic = 0; ic < 2; ic++) {
		if (ic == 1 && m_cells_ic2 == 0) {
			temperatures = lbm_cons(lbm_enc_float(-273.0), temperatures);
			temperatures = lbm_cons(lbm_enc_float(-273.0), temperatures);
			break;
		}
		uint8_t addr = ic == 0 ? BQ_ADDR_1 : BQ_ADDR_2;
		bool ok = false;
		float internal = (float)command_read(addr, IntTemperature, &ok) * 0.1 - 273.15;
		if (!ok) {
			lbm_set_error_reason(ic == 0 ? error_comm_bq1 : error_comm_bq2);
			return ENC_SYM_EERROR;
		}
		temperatures = lbm_cons(lbm_enc_float(internal), temperatures);
		for (unsigned sensor = ic == 0 ? 0 : 4; sensor < 5; sensor++) {
			float volts = (float)command_read(addr, sensors[sensor], &ok) * counts_to_volts;
			if (!ok) {
				lbm_set_error_reason(ic == 0 ? error_comm_bq1 : error_comm_bq2);
				return ENC_SYM_EERROR;
			}
			// HDQ thermistors on both ICs measure MOS temperature, using fixed
			// 10k/3434 sensors and an 18k pull-up independently of external NTCs.
			float temperature = sensor == 4
					? ntc_temperature(volts, 18000, 10000, 3434)
					: ntc_temperature(volts, pullup, nominal, cfg->temp_beta);
			temperatures = lbm_cons(lbm_enc_float(temperature), temperatures);
		}
	}
	return lbm_list_destructive_reverse(temperatures);
}

static lbm_value ext_get_current_unlocked(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	bool ok       = false;
	float current = ((float)command_read(BQ_ADDR_1, CC2Current, &ok) / 100.0);

	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float(current);
}

static lbm_value ext_get_vout(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(HW_GET_VOUT());
}

static lbm_value ext_get_vchg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(HW_GET_VCHG());
}

static lbm_value ext_bms_supports_shutdown(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	// Both JFBMS32 variants support the Lisp shutdown command. v2 asserts
	// PIN_SHUTDOWN; v1 shuts down the BQs and then enters ESP deep sleep.
	return ENC_SYM_TRUE;
}

static lbm_value ext_get_time_of_day_s(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	struct timeval now;
	gettimeofday(&now, NULL);

	return lbm_enc_i32(now.tv_sec);
}

// Returns 1 for wakeup source GPIO IO or RTC IO, 2 for timer and 0 for any other source
static lbm_value ext_bms_wakeup_source(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();

	if (wakeup_causes & (BIT(ESP_SLEEP_WAKEUP_EXT0) | BIT(ESP_SLEEP_WAKEUP_GPIO))) {
		return lbm_enc_i(1);
	}

	if (wakeup_causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
		return lbm_enc_i(2);
	}

	return lbm_enc_i(0);
}

static lbm_value ext_get_btn(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(gpio_get_level(PIN_ENABLE) == 0 ? 0 : 1);
}

static lbm_value ext_set_btn_wakeup_state(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	switch (lbm_dec_as_i32(args[0])) {
		case 0:
			esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_LOW
			);
			break;

		case 1:
			esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH
			);
			break;

		default:
			gpio_wakeup_disable_on_hp_periph_powerdown_sleep(PIN_ENABLE);
			break;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_out(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_OUT_EN, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_chg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	bms_set_chg_hw(lbm_dec_as_i32(args[0]) != 0);
	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_disable_balancing(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_clear_balance_state();
		lbm_set_error_reason("bq_mutex timeout in bms-disable-balancing");
		return ENC_SYM_EERROR;
	}

	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool res = bms_disable_balancing_hw(m_cells_ic2 != 0);
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);

	if (!res) {
		lbm_set_error_reason("BMS balancing disable failed");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_fail_close_outputs(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_clear_balance_state();
		lbm_set_error_reason("bq_mutex timeout in bms-fail-close-outputs");
		return ENC_SYM_EERROR;
	}

	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool res = bms_fail_close_outputs_hw(m_cells_ic2 != 0);
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);

	if (!res) {
		lbm_set_error_reason("BMS fail-close balance disable failed");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-set-bal");
		return ENC_SYM_EERROR;
	}

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int state       = lbm_dec_as_i32(args[1]);
	bool res        = false;

	if (ch < m_cells_ic1) {
		if (state) {
			m_bal_state_ic1 |= (1 << ch);
		} else {
			m_bal_state_ic1 &= ~(1 << ch);
		}

		res = subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1);
		if (!res) {
			lbm_set_error_reason(error_comm_bq1);
		}
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		if (state) {
			m_bal_state_ic2 |= (1 << (ch - m_cells_ic1));
		} else {
			m_bal_state_ic2 &= ~(1 << (ch - m_cells_ic1));
		}

		res = subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2);
		if (!res) {
			lbm_set_error_reason(error_comm_bq2);
		}
	}

	xSemaphoreGive(bq_mutex);
	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_get_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int res         = -1;

	if (ch < m_cells_ic1) {
		res = (m_bal_state_ic1 >> ch) & 0x01;
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		res = (m_bal_state_ic2 >> (ch - m_cells_ic1)) & 0x01;
	}

	return lbm_enc_i(res);
}

static lbm_value ext_direct_cmd_unlocked(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	bool ok = false;
	int res = command_read(addr, lbm_dec_as_u32(args[1]), &ok);
	if (ok) {
		return lbm_enc_i(res);
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_subcmd_cmdonly_unlocked(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	return lbm_enc_i(command_subcommands(addr, lbm_dec_as_u32(args[1])));
}

static lbm_value ext_read_reg_unlocked(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	int reg = lbm_dec_as_i32(args[1]);
	int len = lbm_dec_as_i32(args[2]);
	if (reg < 0 || reg > UINT16_MAX || len < 1 || len > 4) return ENC_SYM_EERROR;

	uint32_t reg_data = 0;
	bool ok           = bq_read_reg(addr, reg, &reg_data, len);

	if (ok) {
		return lbm_enc_u32(reg_data);
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_write_reg_unlocked(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	int reg       = lbm_dec_as_i32(args[1]);
	uint32_t data = lbm_dec_as_u32(args[2]);
	int len       = lbm_dec_as_i32(args[3]);
	if (reg < 0 || reg > UINT16_MAX || (len != 1 && len != 2 && len != 4)) return ENC_SYM_EERROR;

	bool ok = bq_set_reg(addr, reg, data, len);

	if (ok) {
		return ENC_SYM_TRUE;
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

// Serialize complete BQ operations, not just individual I2C transfers.
// Register selection/readback must not interleave with init or the watchdog.
static lbm_value bq_extension_call(
	lbm_value (*function)(lbm_value *, lbm_uint), lbm_value *args, lbm_uint argn
) {
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("BQ transaction mutex timeout");
		return ENC_SYM_EERROR;
	}
	lbm_value result = function(args, argn);
	xSemaphoreGive(bq_mutex);
	return result;
}

static lbm_value ext_get_vcells(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_get_vcells_unlocked, args, argn);
}

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_get_temps_unlocked, args, argn);
}

static lbm_value ext_get_current(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_get_current_unlocked, args, argn);
}

static lbm_value ext_direct_cmd(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_direct_cmd_unlocked, args, argn);
}

static lbm_value ext_subcmd_cmdonly(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_subcmd_cmdonly_unlocked, args, argn);
}

static lbm_value ext_read_reg(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_read_reg_unlocked, args, argn);
}

static lbm_value ext_write_reg(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_write_reg_unlocked, args, argn);
}

typedef struct {
	lbm_uint cells_ic1;
	lbm_uint cells_ic2;
	lbm_uint temp_num;
	lbm_uint batt_ah;
	lbm_uint max_bal_ch;
	lbm_uint soc_use_ah;
	lbm_uint block_sleep;
	lbm_uint vc_empty;
	lbm_uint vc_full;
	lbm_uint vc_balance_start;
	lbm_uint vc_balance_end;
	lbm_uint vc_charge_start;
	lbm_uint vc_charge_end;
	lbm_uint vc_charge_min;
	lbm_uint vc_balance_min;
	lbm_uint balance_max_current;
	lbm_uint min_current_ah_wh_cnt;
	lbm_uint min_current_sleep;
	lbm_uint t_charge_max;
	lbm_uint t_charge_max_mos;
	lbm_uint sleep;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint hw_occ_current;
	lbm_uint hw_ocd_current;
	lbm_uint psw_scd_tres;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint temp_beta;
	lbm_uint temp_res;
	lbm_uint shutdown;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *cached, const char *name) {
	if (*cached == 0 && !lbm_add_symbol_const(name, cached)) return false;
	return *cached == sym;
}

static lbm_value get_or_set_float(bool set, float *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_float(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_float(*val);
	}
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_u16(bool set, uint16_t *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_bool(bool set, bool *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value bms_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg) || !isfinite(lbm_dec_as_float(set_arg))) {
			lbm_set_error_reason("Expected a finite numeric configuration value");
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name      = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t *)&backup.config;

	if (compare_symbol(name, &syms_vesc.cells_ic1, "cells_ic1")) {
		if (set && !cell_counts_valid(lbm_dec_as_u32(set_arg), cfg->cells_ic2)) {
			lbm_set_error_reason("Invalid cell combination");
			return ENC_SYM_EERROR;
		}
		res = get_or_set_i(set, &cfg->cells_ic1, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cells_ic2, "cells_ic2")) {
		if (set && !cell_counts_valid(cfg->cells_ic1, lbm_dec_as_u32(set_arg))) {
			lbm_set_error_reason("Invalid cell combination");
			return ENC_SYM_EERROR;
		}
		res = get_or_set_i(set, &cfg->cells_ic2, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_num, "temp_num")) {
		res = get_or_set_i(set, &cfg->temp_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.batt_ah, "batt_ah")) {
		res = get_or_set_float(set, &cfg->batt_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_bal_ch, "max_bal_ch")) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_use_ah, "soc_use_ah")) {
		res = get_or_set_bool(set, &cfg->soc_use_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.block_sleep, "block_sleep")) {
		res = get_or_set_bool(set, &cfg->block_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_empty, "vc_empty")) {
		res = get_or_set_float(set, &cfg->vc_empty, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_full, "vc_full")) {
		res = get_or_set_float(set, &cfg->vc_full, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_start, "vc_balance_start")) {
		res = get_or_set_float(set, &cfg->vc_balance_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_end, "vc_balance_end")) {
		res = get_or_set_float(set, &cfg->vc_balance_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_start, "vc_charge_start")) {
		res = get_or_set_float(set, &cfg->vc_charge_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_end, "vc_charge_end")) {
		res = get_or_set_float(set, &cfg->vc_charge_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_min, "vc_charge_min")) {
		res = get_or_set_float(set, &cfg->vc_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_min, "vc_balance_min")) {
		res = get_or_set_float(set, &cfg->vc_balance_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.balance_max_current, "balance_max_current")) {
		res = get_or_set_float(set, &cfg->balance_max_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_ah_wh_cnt, "min_current_ah_wh_cnt")) {
		res = get_or_set_float(set, &cfg->min_current_ah_wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_sleep, "min_current_sleep")) {
		res = get_or_set_float(set, &cfg->min_current_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max, "t_charge_max")) {
		res = get_or_set_float(set, &cfg->t_charge_max, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max_mos, "t_charge_max_mos")) {
		res = get_or_set_float(set, &cfg->t_charge_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep, "sleep")) {
		res = get_or_set_float(set, &cfg->sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.shutdown, "shutdown")) {
		res = get_or_set_u16(set, &cfg->shutdown, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_charge_current, "min_charge_current")) {
		res = get_or_set_float(set, &cfg->min_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_charge_current, "max_charge_current")) {
		res = get_or_set_float(set, &cfg->max_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.hw_occ_current, "hw_occ_current")) {
		if (set && (lbm_dec_as_float(set_arg) < 4.0f || lbm_dec_as_float(set_arg) > 124.0f)) return ENC_SYM_EERROR;
		res = get_or_set_float(set, &cfg->hw_occ_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.hw_ocd_current, "hw_ocd_current")) {
		if (set && (lbm_dec_as_float(set_arg) < 4.0f || lbm_dec_as_float(set_arg) > 200.0f)) return ENC_SYM_EERROR;
		res = get_or_set_float(set, &cfg->hw_ocd_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_tres, "psw_scd_tres")) {
		if (set && (lbm_dec_as_float(set_arg) < 0.0f || lbm_dec_as_float(set_arg) > 15.0f ||
				lbm_dec_as_float(set_arg) != lbm_dec_as_i32(set_arg))) return ENC_SYM_EERROR;
		res = get_or_set_i(set, &cfg->psw_scd_tres, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_filter_const, "soc_filter_const")) {
		res = get_or_set_float(set, &cfg->soc_filter_const, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_cell, "t_bal_max_cell")) {
		res = get_or_set_float(set, &cfg->t_bal_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_ic, "t_bal_max_ic")) {
		res = get_or_set_float(set, &cfg->t_bal_max_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_min, "t_charge_min")) {
		res = get_or_set_float(set, &cfg->t_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_mon_en, "t_charge_mon_en")) {
		res = get_or_set_bool(set, &cfg->t_charge_mon_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_res, "temp_res")) {
		res = get_or_set_i(set, (int *)(&cfg->temp_res), &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_beta, "temp_beta")) {
		res = get_or_set_u16(set, &cfg->temp_beta, &set_arg);
	}

	return res;
}

static lbm_value ext_bms_get_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(false, args, argn);
}

static lbm_value ext_bms_set_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(true, args, argn);
}

static lbm_value ext_bms_store_cfg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

// I2C Overrides

static lbm_value ext_i2c_start(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return ENC_SYM_TRUE;
}

static lbm_value ext_i2c_tx_rx_unlocked(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		return ENC_SYM_EERROR;
	}

	uint16_t addr  = 0;
	size_t txlen   = 0;
	size_t rxlen   = 0;
	uint8_t *txbuf = 0;
	uint8_t *rxbuf = 0;

	const unsigned int max_len = 20;
	uint8_t to_send[max_len];

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}
	addr = lbm_dec_as_u32(args[0]);

	if (lbm_is_array_r(args[1])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		txbuf                     = (uint8_t *)array->data;
		txlen                     = array->size;
	} else {
		lbm_value curr = args[1];
		while (lbm_is_cons(curr)) {
			lbm_value arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[txlen++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (txlen == max_len) {
				break;
			}

			curr = lbm_cdr(curr);
		}

		if (txlen > 0) {
			txbuf = to_send;
		}
	}

	if (argn >= 3 && lbm_is_array_rw(args[2])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);
		rxbuf                     = (uint8_t *)array->data;
		rxlen                     = array->size;
	}

	return lbm_enc_i(i2c_tx_rx(addr, txbuf, txlen, rxbuf, rxlen));
}

static lbm_value ext_i2c_tx_rx(lbm_value *args, lbm_uint argn) {
	return bq_extension_call(ext_i2c_tx_rx_unlocked, args, argn);
}

static lbm_value ext_i2c_detect_addr(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t address = lbm_dec_as_u32(args[0]);
	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		return ENC_SYM_NIL;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(0, cmd, 50 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mutex);

	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_bms_fw_version(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(9);
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Wake up and initialize hardware
	lbm_add_extension("bms-init", ext_bms_init);

	// Put BMS hardware in sleep mode
	lbm_add_extension("bms-sleep", ext_hw_sleep);

	// Get list of cell voltages
	lbm_add_extension("bms-get-vcells", ext_get_vcells);

	// Apply HW-specific Cell 0 voltage compensation before publishing BMS data
	lbm_add_extension("bms-cell0-report-offset", ext_cell0_report_offset);

	// Get list of temperature readings
	lbm_add_extension("bms-get-temps", ext_get_temps);

	// Get current in/out. Negative numbers mean charging
	lbm_add_extension("bms-get-current", ext_get_current);

	// Get output voltage after power switch
	lbm_add_extension("bms-get-vout", ext_get_vout);

	// Get charge input voltage
	lbm_add_extension("bms-get-vchg", ext_get_vchg);

	// Get user button state
	lbm_add_extension("bms-get-btn", ext_get_btn);

	// Enable user button wakeup. 1: wakeup on ON, 0: wakeup on OFF, otherwise disable wakeup
	lbm_add_extension("bms-set-btn-wakeup-state", ext_set_btn_wakeup_state);

	//Returns 1 for wakeup source GPIO IO, 2 for timer and 0 for any other source
	lbm_add_extension("bms-wakeup-source", ext_bms_wakeup_source);

	lbm_add_extension("get-time-of-day-s", ext_get_time_of_day_s);

	lbm_add_extension("bms-supports-shutdown", ext_bms_supports_shutdown);

	lbm_add_extension("bms-hw-shutdown", ext_bms_hw_shutdown);

	lbm_add_extension("bms-disable-balancing", ext_bms_disable_balancing);
	lbm_add_extension("bms-fail-close-outputs", ext_bms_fail_close_outputs);
	lbm_add_extension("bms-protection-status", ext_protection_status);
	lbm_add_extension("bms-protection-lock", ext_protection_lock);
	lbm_add_extension("bms-protection-reset", ext_protection_reset);
	lbm_add_extension("bms-control-start", ext_control_start);
	lbm_add_extension("bms-control-feed", ext_control_feed);
	lbm_add_extension("bms-control-ok", ext_control_ok);
	lbm_add_extension("bms-control-stop", ext_control_stop);

	// Enable/disable output switch
	lbm_add_extension("bms-set-out", ext_set_out);

	// Enable/disable charge switch
	lbm_add_extension("bms-set-chg", ext_set_chg);

	// Set and get balancing state for cell
	lbm_add_extension("bms-set-bal", ext_set_bal);
	lbm_add_extension("bms-get-bal", ext_get_bal);

	// HW-specific commands
	lbm_add_extension("bms-direct-cmd", ext_direct_cmd);
	lbm_add_extension("bms-subcmd-cmdonly", ext_subcmd_cmdonly);
	lbm_add_extension("bms-read-reg", ext_read_reg);
	lbm_add_extension("bms-write-reg", ext_write_reg);

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);

	// Replace existing I2C-extensions
	lbm_add_extension("i2c-start", ext_i2c_start);
	lbm_add_extension("i2c-tx-rx", ext_i2c_tx_rx);
	lbm_add_extension("i2c-detect-addr", ext_i2c_detect_addr);

	lbm_add_extension("bms-fw-version", ext_bms_fw_version);
}

void hw_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();
	bq_mutex  = xSemaphoreCreateMutex();

	gpio_config_t gpconf = {0};

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_SHUTDOWN, 1);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_COM_EN, 1);

	gpconf.pin_bit_mask = BIT(PIN_OUT_EN) | BIT(PIN_CHG_EN) | BIT(PIN_COM_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// PIN_PSW_EN configured as plain output
	gpio_set_intr_type(PIN_PSW_EN, GPIO_INTR_DISABLE);
	gpio_set_direction(PIN_PSW_EN, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(PIN_PSW_EN, GPIO_FLOATING);

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);

	// PIN_SHUTDOWN as open-drain output, idle high
	gpio_set_intr_type(PIN_SHUTDOWN, GPIO_INTR_DISABLE);
	gpio_set_direction(PIN_SHUTDOWN, GPIO_MODE_OUTPUT_OD);
	gpio_set_pull_mode(PIN_SHUTDOWN, GPIO_FLOATING);
	gpio_set_level(PIN_SHUTDOWN, 1);

#ifdef SHUTDOWN_SUPPORT
	// Keep DCDC enabled while the ESP32 is in deep sleep.
	gpio_hold_en(PIN_SHUTDOWN);
	gpio_deep_sleep_hold_en();
#endif

	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_COM_EN, 1);

	gpconf.pin_bit_mask = BIT(PIN_ENABLE);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);
	if (xTaskCreate(control_watchdog_task, "bms-control-wdt", 2048, NULL, 8,
			&m_control_watchdog_task) != pdPASS) {
		m_control_watchdog_task = NULL;
	}

	lispif_add_ext_load_callback(load_extensions);
}

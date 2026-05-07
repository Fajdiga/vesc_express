/*
    ESP32-C6 BMS builds do not include the LCD touch stack. Keep the Lisp
    extension hooks linkable without pulling esp_lcd/new I2C into the image.
 */

#include "lispif_touch_extensions.h"

void lispif_touch_irq_from_isr(void) {
}

void lispif_touch_shutdown(void) {
}

void lispif_load_touch_extensions(void) {
}

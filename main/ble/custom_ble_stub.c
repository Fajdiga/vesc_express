#include "custom_ble.h"

// Builds without a BLE host only need the two entry points used outside the
// BLE module (main.c and terminal.c); the scripting API is not compiled.

void custom_ble_init(void) {
}

bool custom_ble_started(void) {
	return false;
}

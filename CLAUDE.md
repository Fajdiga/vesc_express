# JFBMS Project Status

## Claude Code Instructions

**WORKFLOW - ALWAYS FOLLOW THIS:**
1. **PROPOSE FIRST** - Before making ANY code changes, describe what you plan to change and why
2. **WAIT FOR APPROVAL** - Do not implement until the user explicitly approves the approach
3. **THEN IMPLEMENT** - Only after approval, make the changes
4. **TEST & CONFIRM** - Wait until user confirms changes are working
5. **UPDATE HISTORY** - Only after confirmed working, update Development History section

**Development History:** Update the "Development History" section at the bottom of this file ONLY after changes are tested and confirmed working:
1. Today's date as a new ### heading (if not already present for today)
2. Brief description of what was changed and why
3. Files modified
4. Any important technical details or gotchas discovered

Keep entries concise but informative for future reference. This ensures project continuity and documents all changes.

---

## Current Stage: Master-Slave Communication - WORKING

**Date:** 2026-01-13

### JFBMS Architecture

The BMS system consists of:
- **JFBMS Master**: Receives data from slaves, aggregates pack information, displays in VESC Tool
- **JFBMS Slave**: Reads cell voltages/temperatures from BQ76952 chips, broadcasts via CAN

**Status:** Master successfully receives all 32 cell voltages, 4 temperatures, and status from slave.

### 11-bit CAN Protocol

**CAN ID Format:** `(msg_type << 7) | slave_id`

| ID Formula | Message Type | DLC | Content |
|------------|--------------|-----|---------|
| `(0-7 << 7) \| slave_id` | Cell Voltages | 8 | 4 cells per msg, 8 msgs total (32 cells) |
| `0x400 \| slave_id` | Temperatures | 8 | 4 temps (BQ1-IC, BQ1-TS1/cell, BQ2-TS1/cell, BQ2-IC) |
| `0x480 \| slave_id` | Status | 7 | Balance mask (4B) + faults (1B) + cells_ic1 (1B) + cells_ic2 (1B) |
| `0x500 \| slave_id` | Balance Cmd | 5 | Master -> Slave balance command + buzzer beep code |

**Data Format:**
- Cell voltages: uint16 mV, little-endian (0x0000 = not populated, 0xFFFF = error)
- Temperatures: int16 0.1°C, little-endian (0x7FFF = invalid)

#### Example CAN IDs for Slave ID 1
```
001 - Cells 0-3
081 - Cells 4-7
101 - Cells 8-11
181 - Cells 12-15
201 - Cells 16-19
281 - Cells 20-23
301 - Cells 24-27
381 - Cells 28-31
401 - Temperatures
481 - Status
501 - Balance command (from master)
```

---

## Building

### Build Scripts
- `build_master.bat` - Build master firmware only
- `build_slave.bat` - Build slave firmware only
- `build_both.bat` - Build both master and slave firmware

### Manual Build
```batch
REM Build Master
set HW_SRC=hwconf/jetfleet/jfbms_master/hw_jfbms_master.c
set HW_HEADER=hwconf/jetfleet/jfbms_master/hw_jfbms_master.h
idf.py -B build_master build

REM Build Slave
set HW_SRC=hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c
set HW_HEADER=hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h
idf.py -B build_slave build
```

---

## CAN Bus Requirements

For proper Master-Slave communication:
1. **Wiring**: CANH to CANH, CANL to CANL
2. **Baud Rate**: Both devices at 500 kbps (default)
3. **ACK Mode**: Both set to `HW_CAN_NO_ACK_MODE 0` (normal ACK mode)
4. **Termination**: 120Ω resistors at each end of CAN bus recommended

---

## Key Files

### Master
- `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.c` - CAN RX logic, Lisp extensions, VESC BMS integration
- `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.h` - Hardware configuration
- `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp` - Main script (direct CAN buffer)

### Slave
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c` - CAN TX logic, BQ76952 communication
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h` - Hardware configuration
- `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_main.lisp` - Main script (BQ76952 communication, CAN broadcast)

### Common
- `main/comm_can.c` - CAN driver with `hw_can_rx_hook` for hardware-specific handling

---

## Lisp Extensions

### Master Extensions
| Extension | Description |
|-----------|-------------|
| `(master-get-cell-voltage slave-id cell-idx)` | Get cell voltage in V |
| `(master-get-temp slave-id temp-idx)` | Get temperature in °C |
| `(master-get-status slave-id)` | Get (balance-mask faults) list |
| `(master-get-all-cells slave-id)` | Get list of 32 cell voltages |
| `(master-get-all-temps slave-id)` | Get list of 4 temperatures |
| `(master-get-cell-count slave-id)` | Get total cell count from slave (ic1+ic2) |
| `(master-get-cells-ic1 slave-id)` | Get number of cells on BQ1 from slave |
| `(master-get-cells-ic2 slave-id)` | Get number of cells on BQ2 from slave |
| `(master-send-balance slave-id ic1-mask ic2-mask beep-code)` | Send balance command (IC masks combined in C to avoid 28-bit overflow) |
| `(master-slave-active? slave-id)` | Check if slave is responding |
| `(master-get-active-slaves)` | Get list of active slave IDs |
| `(master-check-timeouts timeout-ms)` | Mark timed-out slaves as inactive |
| `(master-reset-data)` | Clear all stored data |
| `(master-can-read-all)` | Read and parse all buffered CAN messages |
| `(master-can-available)` | Get count of buffered messages |
| `(master-can-overflow)` | Get overflow count |
| `(master-update-vesc-bms)` | Update VESC BMS values for VESC Tool display |

### Slave Extensions
| Extension | Description |
|-----------|-------------|
| `(bms-get-slave-id)` | Get configured slave ID |
| `(bms-broadcast-all slave-id cells temps bal-mask faults)` | Broadcast all data via CAN |
| `(slave-update-vesc-bms cells temps)` | Update VESC BMS values for VESC Tool display |

---

## Configuration

### Master Configuration (VESC Tool)
- **Number of Slaves**: Expected number of slave devices (1-8)
- **Slave Timeout**: Time before marking slave as inactive (ms)

### Slave Configuration (VESC Tool)
- **Slave ID**: Unique ID on CAN bus (1-8)
- **Cells IC1**: Number of cells on BQ76952 #1 (3-16)
- **Cells IC2**: Number of cells on BQ76952 #2 (0-16, 0=single chip)

---

## Development History

### 2026-05-05: C6 BMS Data Tab Buttons Wired Up + `event-bms-zero-ofs` Pattern Bug Fix

**Problem:** In VESC Tool's BMS Data tab on `JFBMS_MASTER_C6`, only "Bal On/Off" worked. "Chg En/Dis", "Reset Ah/Wh", and "Cal Zero Current" did nothing.

**Root cause for Cal Zero Current (was already coded but silently broken):** `event-bms-zero-ofs` is sent from C as a **bare symbol** (`f_sym(&v, sym_bms_zero_ofs)` in `lispif_vesc_extensions.c:1956-1967`) — no surrounding `f_cons` like every other BMS event. The Lisp pattern `((event-bms-zero-ofs) ...)` matched a 1-element list, not a bare symbol, so the recv clause never fired. Fix: pattern is now `(event-bms-zero-ofs ...)` (bare-symbol literal). bms32 has the same dead `event-enable` registration but never had a recv clause for zero-ofs, so this trap is brand new.

**New handlers added in `jfbms_master_c6_main.lisp`:**
- `event-bms-chg-allow` — toggles `PIN_CHG_EN` (GPIO 5, confirmed wired to charge FET on the C6 PCB) via existing `bms-set-chg`. Mirrors state to VESC Tool via `(set-bms-val 'bms-chg-allowed ...)` so the button label updates correctly. **Default at boot: charge OFF** — user must explicitly press "Chg En" to enable charging (matches user's safety preference for this hardware).
- `event-bms-reset-cnt` — zeroes `ah-cnt` and/or `wh-cnt` based on the (ah, wh) flags from VESC Tool, pushes new value to BMS struct immediately so the UI reflects reset.
- 10 Hz Ah/Wh integrator in main loop: accumulates `(master-get-current) * 0.1 s` into Ah and `i * v_tot * 0.1 / 3600` into Wh, gated by `min_current_ah_wh_cnt` so quiescent-noise current doesn't drift the counters. Counters do NOT persist across reboots (no NVS save — bms32 does this via `store-setting`, deferred for C6).

**`set-bms-val` quirk:** `bms-chg-allowed` is stored as `int` in the bms struct (`get_or_set_i` at `lispif_vesc_extensions.c:660-661`), not `bool` — must pass `(if chg-allowed 1 0)`, not the raw bool, otherwise the value doesn't propagate.

**Files modified:**
- `main/hwconf/jetfleet/jfbms_master_c6/jfbms_master_c6_main.lisp` — handlers, integrator, default-OFF charge state

**Status:** Tested on hardware — all four button groups (Bal On/Off, Chg En/Dis, Reset Ah/Wh, Cal Zero Current) now respond correctly. Charge FET defaults to OFF at boot.

---

### 2026-05-05: ESP32-C6 NimBLE Port — Stage 3 (custom_ble.c full port) — IMPLEMENTED, UNTESTED

**Goal:** Replace the Stage 2 stub `custom_ble_nimble.c` with a working implementation so the LispBM `ble-*` extensions in `lispif_ble_extensions.c` work on C6. Stage 2 had every entry point return `CUSTOM_BLE_NOT_STARTED` because JFBMS production never uses `BLE_MODE_SCRIPTING` — this stage closes that gap for any future C6 firmware that wants to expose custom GATT services from Lisp.

**Architecture:**
- Internal state mirrors the Bluedroid version: `service_instance_t[ble_service_capacity]` + flat `attr_instance_t[ble_chr_descr_capacity]` table, with each attr owning its heap-allocated value buffer.
- NimBLE's GATT registration is declarative (`ble_gatt_svc_def[]`), so dynamic add/remove is implemented as a full **rebuild**: `ble_gap_adv_stop` → `ble_gatts_reset` → `ble_svc_gap_init` / `ble_svc_gatt_init` → `ble_gatts_count_cfg` / `ble_gatts_add_svcs` → `ble_gatts_start` → restart adv. Triggered on every `custom_ble_add_service` / `custom_ble_remove_service`.
- Handle capture is via `ble_hs_cfg.gatts_register_cb` — NimBLE fires it once per service / chr / dsc as `ble_gatts_start()` walks the table in declaration order. We track three cursors (`reg_cur_service`, `reg_cur_chr_attr_idx`, `reg_cur_dsc_attr_idx`) into `custom_attr[]` and write the assigned handles back. Auto-CCCDs (UUID 0x2902) inserted by NimBLE for chars with NOTIFY/INDICATE flags are detected by UUID and skipped — they aren't user-defined attrs.
- Single `gatt_access_cb` for everything; `arg` field on each `ble_gatt_chr_def` / `ble_gatt_dsc_def` points back to the `attr_instance_t`. Reads append `value`/`value_len` to the response mbuf; writes flatten the inbound mbuf into the attr's value buffer (clamped to `value_max_len`) and fire the global `attr_write_cb`.
- `custom_ble_set_attr_value` calls `ble_gattc_notify_custom` and/or `ble_gattc_indicate_custom` based on the char's prop flags when a peer is connected.
- **Disconnect-before-rebuild** (approved at proposal time): `ble_gatts_reset()` requires no active conns, so any rebuild while a peer is connected calls `ble_gap_terminate` and waits up to 500 ms on a binary semaphore signaled from the disconnect event. Scripts that mutate GATT only at startup never hit this; mutations during an active session will kick the peer (expected).

**UUID / flag conversions** (all in `custom_ble_nimble.c`):
- `bd_uuid_to_nimble`: maps `esp_bt_uuid_t` (Bluedroid-shaped) to `ble_uuid_any_t`. 128-bit UUIDs are LE in both representations → straight memcpy.
- `chr_flags_from(prop, perm)`: ORs property bits (`READ`/`WRITE`/`WRITE_NR`/`NOTIFY`/`INDICATE`/`BROADCAST`) and permission bits (`READ_ENCRYPTED`/`READ_ENC_MITM`/`WRITE_ENCRYPTED`/`WRITE_ENC_MITM`/`WRITE_SIGNED`) into the right `BLE_GATT_CHR_F_*` flags.
- `dsc_flags_from(perm)`: same idea for descriptor `att_flags`, mapping into `BLE_ATT_F_*`.

**Memory model:** All built-table allocations (`built_svcs`, `built_chrs`, `built_dscs`, `built_uuids`, `built_val_handles`) are freed at the start of every rebuild via `free_built_tables()`. NimBLE's contract is that `ble_gatt_svc_def[]` and everything it points to must outlive registration but doesn't need to live past it — the host copies what it needs internally.

**Files modified:**
- `main/ble/custom_ble_nimble.c` — was a 105-line Stage 2 stub, now ~580 lines of real implementation. No other file touched (`custom_ble.h`, `ble_compat.h`, `lispif_ble_extensions.c`, `comm_ble_nimble.c`, `custom_ble_bluedroid.c`, `main/CMakeLists.txt` all unchanged).

**Image-size delta (JFBMS_MASTER_C6):**
- Stage 2 stub: 1,585,964 bytes
- Stage 3 full: 1,592,332 bytes
- Growth: ~6 KB (the implementation is mostly bookkeeping and short helper fns; NimBLE GATT host code was already linked in for Stage 2). OTA slot usage 84% of 1856 KB, ~302 KB headroom.

**Test status — IMPLEMENTED, UNTESTED.**
- Build: clean (no new warnings, all NimBLE symbols resolve against IDF v5.5 headers).
- Runtime: not exercised. JFBMS production firmware never sets `ble_mode = BLE_MODE_SCRIPTING`, so the Stage 3 path is dormant in shipping C6 builds. Same was true on C3/Bluedroid — the upstream `custom_ble.c` has been in the tree since 2023 but JFBMS has never used it. Whoever first wires a C6 Lisp script to `ble-add-service` / `ble-attr-set-value` will be the first to exercise this code; expect to iterate when that happens.
- To test on hardware when the time comes: set `ble_mode = BLE_MODE_SCRIPTING`, set `ble_service_capacity > 0` and `ble_chr_descr_capacity > 0` in BMS config, then run a Lisp script that calls `(ble-start-app)` followed by `(ble-add-service …)` / `(ble-attr-set-value …)` and connect with nRF Connect or LightBlue to verify reads/writes/notifications. Removing then re-adding a service while a peer is connected will disconnect the peer (this is the approved rebuild semantic).

**Sharp edges to watch when first exercising:**
- The disconnect/rebuild path is timing-sensitive — the 500 ms semaphore timeout is a guess. If a real peer takes longer to ack `ble_gap_terminate`, raise it.
- `ble_gatts_reset()` returns `BLE_HS_EBUSY` on an active connection; the disconnect-first dance is required, not optional. If for some reason the disconnect signal is missed, the rebuild will fail with `CUSTOM_BLE_ESP_ERROR`.
- Auto-CCCD detection skips by UUID 0x2902. If NimBLE's auto-inserted CCCD ever uses a different UUID type (it shouldn't), the cursor advance will desynchronize and chr/dsc handles will be wrong.

---

### 2026-05-04: ESP32-C6 NimBLE Port — Stage 1+2 (VESC Tool BLE Channel)

**Goal:** Cut ~150 KB of flash on `JFBMS_MASTER_C6` by replacing the Bluedroid host stack with NimBLE. C3 hardware (`JFBMS_MASTER`, slave, all other VESC HW) stays on Bluedroid — no functional change there.

**Architecture — stack split via CMake selector:**
- `main/comm_ble.c` → renamed `main/comm_ble_bluedroid.c`
- `main/ble/custom_ble.c` → renamed `main/ble/custom_ble_bluedroid.c`
- New `main/comm_ble_nimble.c` — full VESC Tool GATT channel (RX write, TX notify, MTU, encrypted-mode passkey pairing) on NimBLE. Same Nordic-UART-style 128-bit UUIDs as Bluedroid → VESC Tool sees an identical profile.
- New `main/ble/custom_ble_nimble.c` — Stage 3 stub. All Lisp `ble-*` extensions return `CUSTOM_BLE_NOT_STARTED`. JFBMS doesn't use the LispBM custom-GATT API so this is fine in production; full NimBLE port deferred.
- New `main/ble/ble_compat.h` — type shim. On Bluedroid builds it's a transparent pass-through (`#include "esp_bt_defs.h"` etc.). On NimBLE builds it re-defines `esp_bt_uuid_t`, `esp_gatt_perm_t`, `esp_gatt_char_prop_t`, `esp_ble_adv_params_t`, `ESP_UUID_LEN_*`, `ESP_GATT_PERM_*`, `ESP_GATT_CHAR_PROP_BIT_*`, `ADV_CHNL_*` so `custom_ble.h` and `lispif_ble_extensions.c` compile against either stack with no source changes.

**CMake selector (`main/CMakeLists.txt`):** picks `_nimble.c` when `CONFIG_BT_NIMBLE_ENABLED`, `_bluedroid.c` otherwise. Gated only when not a slave build.

**Upstream VESC files patched:**
- `main/commands.c` — `#include "esp_bt_main.h"` gated behind `#ifdef CONFIG_BT_BLUEDROID_ENABLED` (was unused but the include broke compilation when Bluedroid headers vanished from the include path).
- `main/lispif_vesc_extensions.c` — `sleep_disable_radios` / `sleep_deinit_radios` provide a NimBLE branch using `nimble_port_stop()` / `nimble_port_deinit()` instead of `esp_bluedroid_disable/deinit`. The controller calls (`esp_bt_controller_*`) are stack-agnostic and stayed.

**sdkconfig (`sdkconfig.defaults.esp32c6`):**
- `# CONFIG_BT_BLUEDROID_ENABLED is not set`
- `CONFIG_BT_NIMBLE_ENABLED=y`
- `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1`, `BT_NIMBLE_ROLE_PERIPHERAL=y`, `BT_NIMBLE_ROLE_BROADCASTER=y`, central/observer disabled
- `CONFIG_BT_NIMBLE_NVS_PERSIST=y`, `BT_NIMBLE_SM_LEGACY=y`, `BT_NIMBLE_SM_SC=y` for encrypted-mode passkey pairing parity

**Image-size delta (C6, JFBMS_MASTER_C6):**
- Bluedroid: 1,743,538 bytes
- NimBLE: 1,585,964 bytes
- Saved: ~154 KB (~9%). OTA slot usage 91% → 83%, ~280 KB headroom in the 1856 KB slot.

**Sharp edges hit during the port (lessons for next time):**
- **Cached `sdkconfig` trap (again).** First "successful" C6 build still ran on Bluedroid because the project-root `sdkconfig` file was generated from the previous defaults and hasn't been regenerated when only `sdkconfig.defaults.esp32c6` changes. ESP-IDF only reads `sdkconfig.defaults*` when there is no `sdkconfig`. Workflow: `del sdkconfig`, `rmdir /S /Q build`, then reconfigure. Same trap as the C6 partition table swap a few weeks ago.
- **`BLE_GAP_EVENT_REPEAT_PAIRING` member confusion.** `event->repeat_pairing` only carries `conn_handle` — to delete the bonded peer you have to look up the conn descriptor first via `ble_gap_conn_find` and pass `desc.peer_id_addr` to `ble_store_util_delete_peer`.
- **`bool` not in `custom_ble.h`.** The header used `bool` in function signatures but only got `<stdbool.h>` transitively through Bluedroid headers. Without those on NimBLE builds the header doesn't parse — added explicit `#include <stdbool.h>` to the header.

**Stage 3 (deferred):** Full NimBLE port of `custom_ble.c` (~1100 lines, dynamic GATT add/remove) and the LispBM `ble-*` extensions. Not blocking — JFBMS_MASTER_C6 uses BLE_MODE_OPEN/ENCRYPTED via `comm_ble_nimble.c` and never enters BLE_MODE_SCRIPTING.

**Files modified:**
- `main/CMakeLists.txt`
- `main/commands.c`, `main/lispif_vesc_extensions.c`
- `main/ble/custom_ble.h`, `main/ble/lispif_ble_extensions.c`
- `sdkconfig.defaults.esp32c6`
- `.vscode/settings.json` (default IDF_TARGET → esp32c6)

**Files renamed:**
- `main/comm_ble.c` → `main/comm_ble_bluedroid.c`
- `main/ble/custom_ble.c` → `main/ble/custom_ble_bluedroid.c`

**Files added:**
- `main/comm_ble_nimble.c`
- `main/ble/custom_ble_nimble.c` (Stage 3 stub)
- `main/ble/ble_compat.h`

**Status:** Built clean, flashed to C6 PCB, VESC Tool pairs and exchanges packets, BLE-triggered reboot command round-trips correctly. C3 master build path untouched (verified by reading the resulting include chain: Bluedroid headers stay in the include path on C3 builds because `CONFIG_BT_BLUEDROID_ENABLED=y` there).

---

### 2026-05-04: ESP32-C6 Master Variant — `JFBMS_MASTER_C6` Builds Cleanly with BLE+WiFi

**Goal:** Add a second master variant targeting the ESP32-C6-MINI-N4 module (future product with current sensing, MOSFET switching, and full BLE/WiFi). First milestone is "compiles and links cleanly with BLE+WiFi enabled" — pin map and peripheral wiring stay as placeholders until the C6 PCB is available.

**New hardware variant:** `main/hwconf/jetfleet/jfbms_master_c6/` cloned from `jfbms_master/` with:
- `HW_NAME "JFBMS_MASTER_C6"`, `HW_TARGET "esp32c6"`, `VAR_INIT_CODE` bumped (so existing C3 master units never load this struct)
- Conf-parser symbols namespaced to `jfbms_master_c6_confparser_*` and `jfbms_master_c6_confxml_*`
- All `OVR_CONF_*` and `#include` paths point at the renamed conf files
- `#warning` in `hw_init()` flags placeholder pin map; `// TODO C6 pin map` in the header

**Build-system / CMake:** Added `hwconf/jetfleet/jfbms_master_c6` to `main/CMakeLists.txt` `COMPONENT_ADD_INCLUDEDIRS`. Simplified the default-HW_NAME logic in root `CMakeLists.txt` to `JFBMS_MASTER_C6`.

**ESP32-C6 chip-API drift fixes** (each gated by `CONFIG_IDF_TARGET_ESP32C6`, no behaviour change on C3/S3):
- `main/comm_wifi.c` — added C6 case for `UDP_MULTICAST_TASK_STACK_SIZE`
- `main/lispif.c` — added C6 case for LBM heap/mem/bitmap sizes
- `main/lispif_vesc_extensions.c` — added C6 case for `LBM_EVENTS_TASK_STACK_SIZE`, `esp_deep_sleep_enable_gpio_wakeup` path. Stubbed `gpio_deep_sleep_hold_en/dis` on C6 (functions don't exist there)
- `main/drivers/hwspi.c`, `main/drivers/spi_bb.c` — added C6 case for GPIO register macros. C6 uses `GPIO.in.val` (not `.data`)
- `main/display/disp_{ili9341,ili9488,sh8601,ssd1351,st7735,st7789}.c` — added C6 case for `DISP_REG_SET/CLR`
- `main/adc.c` — gated the legacy `esp_adc_cal_*` calibration API for C6 (not available there in IDF v5.5+); `adc_get_voltage` returns -1.0 on C6 same as the uncalibrated fallback. New ADC API migration deferred until C6 master needs it.
- `main/comm_can.c` — C6 has two TWAI controllers, so signal indexes are namespaced (`TWAI0_TX_IDX`/`TWAI0_RX_IDX` instead of `TWAI_TX_IDX`/`TWAI_RX_IDX`)

**SDKconfig & partition table:**
- New `sdkconfig.defaults.esp32c6` overrides only what differs from `sdkconfig.defaults`: USB-Serial-JTAG console, 160 MHz CPU, BT_BTU stack size, and the partition file
- New `partition_ota_c6.csv` because the C6 image (~1.75 MB with Bluedroid + WiFi + BLE) does not fit C3's 1600 KB OTA slots. Layout: ota_0/ota_1 = 1856 KB each, lisp = 192 KB, qml = 64 KB. Total = 4 MB exactly.

**Bluetooth/WiFi stack:** Bluedroid (BLE-only mode) works on ESP32-C6 in IDF v5.5.x — no host-stack swap to NimBLE was needed. The previous attempt to port to C6 stalled because the cached `sdkconfig` at the project root pinned `partition_ota.csv`; deleting it forced regen against the new defaults stack and the build flowed through.

**Files modified:**
- `CMakeLists.txt`, `main/CMakeLists.txt`
- `main/adc.c`, `main/comm_can.c`, `main/comm_wifi.c`, `main/lispif.c`, `main/lispif_vesc_extensions.c`
- `main/drivers/hwspi.c`, `main/drivers/spi_bb.c`
- `main/display/disp_ili9341.c`, `disp_ili9488.c`, `disp_sh8601.c`, `disp_ssd1351.c`, `disp_st7735.c`, `disp_st7789.c`

**Files added:**
- `main/hwconf/jetfleet/jfbms_master_c6/` (9 files: hw + conf + lisp + bq769x2_defs)
- `partition_ota_c6.csv`
- `sdkconfig.defaults.esp32c6`

**Build command:**
```
idf.py -B build -DIDF_TARGET=esp32c6 -DHW_NAME=JFBMS_MASTER_C6 reconfigure
idf.py -B build build
```

**Status:** Build succeeds end-to-end on ESP32-C6 target. Bootloader + app binaries produced; partition check passes. Hardware bring-up (pin remap, BQ I2C, FET drivers, BLE pairing, WiFi STA join) is the next step once the C6 PCB is in hand.

---

### 2026-02-16: Master GPIO Pin Remap & Hardware Init Cleanup

**Changes:**
- Corrected master pin definitions to match actual JFBMS PCB layout:
  - `PIN_BQ1_EN` = GPIO 3 (new), `PIN_SHUTDOWN` = GPIO 8 (was `PIN_PCHG_EN`), `PIN_PCHG_EN` = GPIO 10 (was `PIN_PSW_EN`)
- Removed `PIN_PSW_EN` entirely — gate driver master enable no longer used on this board
- Removed all `gpio_set_level(PIN_PSW_EN, ...)` from C code (was set before every FET enable)
- Configured `PIN_SHUTDOWN` and `PIN_BQ1_EN` as open-drain outputs (default hi-Z/off)
- Removed GPIO 10 from C init — now owned by LEDC PWM for precharge buzzer
- Set `PIN_COM_EN` default to low (active) in C init
- Added `(pwm-start 4000 0.5 0 10 8)` in Lisp for 4kHz precharge buzzer on GPIO 10
- Added `(gpio-write 9 0)` in Lisp to hold COM_EN active
- Changed CAN pins from GPIO 0/1 to GPIO 6/7 (`JFBMS_USE_CAN_IO_0_1` = 0)
- Changed default build target from slave to master

**Files modified:**
- `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.h` — pin definitions, CAN IO select
- `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.c` — GPIO init, removed PSW_EN references
- `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp` — PWM buzzer, COM_EN
- `main/CMakeLists.txt` — default build target

---

### 2026-02-13: Fix IC2 Balance State Display & Add Balance Debug Logging

**Problem:** VESC Tool BMS data tab only showed IC1 cells as orange (balancing). IC2 cells never showed balance state even when actively balancing.

**Root cause:** Balance mask is packed as `ic1[bits 0-15] | ic2[bits 16-31]`, but `bal_state[]` was extracted by shifting by flat cell index `c`. For IC2 cells (e.g. c=5 in a 5+5 config), it read bit 5 instead of bit 16 — always 0.

**Fix:** Map cell index to correct bit position: `bit = (c < ic1_cnt) ? c : (16 + c - ic1_cnt)`. Applied to both master (`ext_master_update_vesc_bms`) and slave (`ext_slave_update_vesc_bms`).

**Also added:** Binary balance mask logging in master balance thread. Prints per-cell balance state as `BAL S1 IC1:0101 IC2:1000 min=3.284` every compute cycle when balancing is active.

**Also cleaned up:** Removed CAN queue diagnostics from master main loop (queue bar, throughput counters, loss delta prints) to reduce log noise.

**Files modified:**
- `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.c` — IC2 bal_state bit mapping fix
- `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp` — mask-to-bin helper, balance debug prints, removed CAN diag clutter
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c` — IC2 bal_state bit mapping fix

**Status:** Tested and working — both IC1 and IC2 cells show orange in VESC Tool when balancing.

---

### 2026-02-13: Dual BQ76952 I2C Address Change (Remove Chip Selection)

**Problem:** Both BQ76952 chips shared the same default I2C address (0x08). The slave had to use enable pins (`PIN_BQ1_EN`, `PIN_BQ2_EN`) to toggle which chip was active before every I2C operation. Only one chip could communicate at a time, adding latency and complexity.

**Solution:** Adopted the bms32/master approach — change BQ1's I2C address during init so both chips can be on the bus simultaneously:

1. During `bms-init`: disable BQ2, reset BQ1 to default address (0x08)
2. Initialize BQ1 at 0x08, then change its I2C address to 0x10 (via `I2CAddress = 0x20` register)
3. Enable both BQ chips — BQ1 at 0x10, BQ2 at 0x08 (different addresses, no conflicts)
4. Initialize BQ2 at 0x08 if `cells_ic2 > 0`

**Changes:**
- `BQ_ADDR_1` changed from `0x08` to `0x10` (address after init change)
- `BQ_ADDR_2` stays at `0x08` (default)
- Removed `select_bq_chip()` function entirely
- Removed all `select_bq_chip()` calls from: `ext_get_vcells`, `ext_get_temps`, `ext_set_bal`, `ext_hw_sleep`, `apply_bal_bitmap`
- Updated `ext_direct_cmd`, `ext_subcmd_cmdonly`, `ext_read_reg`, `ext_write_reg` to select BQ via first arg (1=BQ1@0x10, 2=BQ2@0x08) matching bms32 API
- Updated pin comments in header

**Files modified:**
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c`
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h`

**Lisp scripts:** No changes needed — `bms-init` handles everything in C, transparent to Lisp.

**Status:** Tested and working — both BQ chips communicate correctly with unique I2C addresses.

---

### 2026-02-11: Fix LBM Extension Slot Overflow & Dynamic Sim Cell Config
- Slave `bms-get-param` and other late-registered extensions silently failed because LBM extension table (350 slots) was full — upstream `lispif_vesc_extensions.c` grew from 189 to 197 extensions, pushing total past limit.
- Added `USER_EXTENSION_STORAGE_SIZE 50` to slave header (matching master), giving 400 total slots.
- Updated `jfbms_slave_sim.lisp` to read `cells_ic1`/`cells_ic2` from VESC Tool package config instead of hardcoded 16+16.
- Simulator now respects VESC Tool cell count settings (e.g. 5+5 = 10 cells).
- Files modified:
  - `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h`
  - `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_sim.lisp`
  - `main/CMakeLists.txt` (default build target changed to slave)
- Status: Tested and working on hardware.

### 2026-02-10: Slave TWAI Hardware Filter & CAN Filter Hook Refactor
- Added a board-specific CAN filter hook path so `comm_can.c` can use hardware-defined TWAI filters when provided, while defaulting to `ACCEPT_ALL` for other hardware.
- Implemented `hw_can_get_filter_config(...)` in JFBMS slave to accept only master balance command ID `0x500 | slave_id` in TWAI hardware.
- Removed redundant slave software ID check in `hw_can_rx_hook` and rely on hardware filter for standard-ID selection.
- Filter reconfiguration now runs both on CAN start and baud-rate reinit.
- Files modified:
  - `main/comm_can.c`
  - `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c`
- Note: Changes are implemented and committed; runtime verification on hardware is still required.

### 2026-02-11: CAN RX/TX Loss Monitoring Counters
- Added core CAN diagnostics counters for dropped RX ring frames, TWAI RX queue misses/overruns, and TX failures/timeouts.
- RX ring write path now detects full condition and increments overflow counter instead of blindly overwriting unread entries.
- Exposed new counters to Lisp via:
  - `(can-rx-ring-overflow)`
  - `(can-rx-queue-missed)`
  - `(can-rx-queue-overrun)`
  - `(can-tx-fail)`
  - `(can-tx-timeout)`
- Extended JFBMS master `can-debug` output with these core CAN metrics.
- Files modified:
  - `main/comm_can.h`
  - `main/comm_can.c`
  - `main/lispif_vesc_extensions.c`
  - `main/hwconf/jetfleet/jfbms_master/hw_jfbms_master.c`
- Note: Changes are implemented; hardware/runtime validation still pending.

### 2026-02-11: Queue Monitoring, Counter Reset, and Master Log Cleanup
- Added TWAI RX queue level and peak counters in `comm_can` and exposed them to Lisp:
  - `(can-rx-queue-level)`
  - `(can-rx-queue-peak)`
  - `(can-reset-counters)`
- Added 1-second CAN diagnostics in master, slave, and slave simulator scripts:
  - Queue bar output (`CAN QRX` / `SIM CAN QRX`)
  - Throughput + pending max snapshot (`pend_max_1s`)
  - Loss and TX error deltas (`CAN STAT` / `SIM CAN STAT`)
- Changed queue missed/overrun reporting to be baseline-relative after reset/start, so startup history does not pollute runtime monitoring.
- Reduced master runtime log load by removing verbose balancing debug prints (per-cell and per-send logs) while keeping balancing behavior unchanged.
- Switched default build target to master in both root and main CMake defaults.
- Fixed TWAI filter initialization compatibility by avoiding assignment of `TWAI_FILTER_CONFIG_ACCEPT_ALL()` where toolchain requires explicit field assignment/initialization.
- Files modified:
  - `CMakeLists.txt`
  - `main/CMakeLists.txt`
  - `main/comm_can.h`
  - `main/comm_can.c`
  - `main/lispif_vesc_extensions.c`
  - `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp`
  - `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c`
  - `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_main.lisp`
  - `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_sim.lisp`
- Status: Build tested by user (successful), runtime logs verified on master + simulator with zero ongoing loss counters.

### 2026-02-11: Master Loop Retune, 1 Hz Balance Keepalive, and Blink Fix
- Retuned master main-loop timing to match observed traffic profile:
  - CAN RX drain now runs at fixed 20 Hz (`sleep 0.05`)
  - VESC update/timeout/connectivity tasks remain gated at 10 Hz
  - 1-second diagnostics now use 20-loop gating (instead of 100-loop gating)
- Reworked balance command flow to decouple mask computation from command keepalive:
  - Balance thread computes masks from latest cached local+slave voltages
  - Main loop transmits cached masks as keepalive while balancing is active
- Set slave balance keepalive cadence to 1 Hz to reduce CAN load while still satisfying slave 10s watchdog.
- Added immediate first keepalive transmission on `BAL CMD: start` to avoid random 0-1 second phase delay before first command.
- Fixed balancing "blink" in VESC Tool by removing transient global mask clearing during recompute:
  - Previously, keepalive could catch an all-zero intermediate cache and momentarily command balance off
  - Now cached masks are updated in place, preventing spurious off pulses
- Confirmed protocol handling remains correct:
  - Master sends one target mask command per slave (no extra zero pre-send from master)
  - Slave C implementation performs required BQ toggle internally (`0 -> new_mask`) when applying bitmap
- Files modified:
  - `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp`
- Status: User runtime logs show clean CAN counters (`m_ovf/core_ovf/q_missed/q_overrun/tx_fail/tx_to = 0`) and stable balancing behavior after blink fix.

### 2026-01-09: Slave ID Runtime Configuration
- Slave ID now configurable via VESC Tool without restart

### 2026-01-09: CAN Protocol Implementation
- Complete 11-bit CAN master-slave protocol
- 8 cell voltage messages (32 cells total)
- 1 temperature message (4 sensors)
- 1 status message (balance mask + faults)

### 2026-01-12: Master Implementation
- Created JFBMS Master device based on slave architecture
- Implemented CAN RX via LispBM event system (`event-can-sid`) - later found to be broken
- Added all master Lisp extensions for data access

### 2026-01-13: Master CAN Reception Fix & VESC BMS Integration
- Fixed CAN ACK mode (`HW_CAN_NO_ACK_MODE 0`) on both master and slave
- Discovered LispBM event system (`event-can-sid`) has a bug - events not delivered to Lisp
- Implemented direct CAN buffer solution:
  - Added `hw_can_rx_hook` weak symbol in `comm_can.c` for hardware-specific CAN handling
  - Master implements circular buffer (64 messages) to capture all CAN messages
  - New Lisp extensions: `master-can-read-all`, `master-can-available`, `master-can-overflow`
- Added VESC BMS data integration (`master-update-vesc-bms`) for VESC Tool display
- Master now successfully receives all 32 cells, 4 temps, and status at 20Hz
- Removed unnecessary GPIO configuration in `comm_can.c` that was causing signal issues

### 2026-01-13: SOC/SOH Fix, Temperature Filtering & Slave ID from Config
- Fixed SOC and SOH calculation - VESC expects 0.0-1.0 (fraction), not 0-100 (percent)
- Invalid temperatures now correctly use `0x7FFF` marker instead of `-1.0°C`
- Slave ID now reads from VESC Tool config via `(bms-get-slave-id)` instead of hardcoded value

### 2026-01-13: Balancing Fixes, LispBM Corrections & Build Improvements
- BQ76952 balancing timeout fix: use TOGGLE approach (write 0, then value) to reset ~18s internal timer
- LispBM fixes: `bitwise-or` → `+`, counter-based timing instead of `systime`, mutable list for state
- Build system: Added `build_both.bat`, fixed VS Code integration, increased `BMS_MAX_CELLS` to 64

### 2026-01-21: Dynamic Cell Count in VESC Tool
- Slave sends actual cell count in status message (byte 5)
- Master only displays configured cells, no more empty 0.0V slots
- New extension: `(master-get-cell-count slave-id)`

### 2026-01-22: CAN Bus Optimization
- Slave only sends required cell messages (12 cells = 3 msgs instead of 8)
- Reduced CAN bus load by 40-50% for typical configurations

### 2026-01-22: Slave Balancing Refresh Optimization
- Removed redundant 100ms balancing refresh from main loop
- I2C traffic reduced from ~10 writes/sec to 1 write/sec

### 2026-01-22: VESC Tool BMS Display on Slave
- Added `slave-update-vesc-bms` extension for local VESC BMS display
- VESC Tool now shows cell voltages, temperatures when connected to slave

---

## Troubleshooting

### No CAN communication
1. Check physical wiring (CANH/CANL, termination)
2. Verify both devices have `HW_CAN_NO_ACK_MODE 0`
3. Check baud rate matches (500K default)
4. Rebuild both firmware after configuration changes

### Master shows "No slaves detected"
1. Ensure slave is powered and running demo script
2. Verify CAN bus connection
3. Check `master-can-available` returns > 0 (messages being received)
4. Check `master-can-overflow` for buffer overflows

### Known Issues
- LispBM `event-can-sid` system does not work - use direct buffer approach instead
- `can-recv-sid` only catches ~10% of messages when slave sends bursts
- VESC Tool shows temperatures as T1, T2, T3... (hardcoded in app, cannot rename)
- LispBM `bitwise-or` only accepts 2 arguments - use `+` when bits don't overlap
- LispBM `(systime)` resets each loop iteration in `loopwhile` - use counter-based timing instead
- LispBM global variables (`def`, `setq`, `set`) don't persist in loops - use mutable list with `(setix list idx value)`
- BQ76952 CB_ACTIVE_CELLS requires TOGGLE (write 0, then value) to reset ~18s internal timeout
- BQ76952 cannot balance adjacent cells simultaneously - must alternate odd/even cell groups
- LispBM uses 28-bit integers (4 tag bits in 32-bit word) - max value 0x7FFFFFF. Never construct 32-bit values in Lisp; pass 16-bit halves to C extensions instead

### VS Code Build Errors
If you get `${ProjectId}.map not found` error:
1. Delete `.vscode` folder
2. Reopen project in VS Code
3. Run `ESP-IDF: Configure ESP-IDF Extension`
4. Set target: `ESP-IDF: Set Espressif Device Target` → `esp32c3`

Or build from terminal instead: `idf.py build`

### Temperature Sensor Mapping

| Index | VESC Tool | Actual Sensor | Description |
|-------|-----------|---------------|-------------|
| 0 | T1 | BQ1 IC | BQ76952 #1 internal die temperature |
| 1 | T2 | BQ1 TS1 | External NTC on BQ1 TS1 pin (cell temp) |
| 2 | T3 | BQ2 IC | BQ76952 #2 internal die temperature |
| 3 | T4 | BQ2 TS1 | External NTC on BQ2 TS1 pin (cell temp) |

**Invalid temperature marker:** `0x7FFF` (3276.7°C) - filtered out, not displayed

### 2026-02-02: Buzzer Beep Codes via CAN
- Moved buzzer beep codes into balance command byte 4 (DLC 5) instead of separate CAN message
- Master sends beep code along with balance mask in single command
- Slave decodes beep code and plays pattern via `handle-beep` function
- Backwards compatible: slave accepts both 4-byte (old) and 5-byte (new) balance commands
- Beep patterns defined in Lisp for easy modification without reflashing

### 2026-02-03: Master BQ76952 Integration (Standalone + Slave Aggregation)
- Rewrote master to include full BQ76952 I2C support (based on VBMS32 architecture)
- Master can now read its own local cells via BQ76952 AND receive slave cells via CAN
- `master-update-vesc-bms` combines local BQ76952 cells + slave cells into single VESC BMS display
- Added all BQ76952 Lisp extensions to master (bms-init, bms-get-vcells, bms-get-temps, etc.)
- Added BQ769x2 register definitions header (`bq769x2_defs.h`)
- Master can work standalone (32 local cells) or with slaves (local + remote cells)
- Files modified: `hw_jfbms_master.c/h`, `jfbms_master_conf_default.h`, `jfbms_master_confparser.c/h`, `jfbms_master_confxml.c/h`, `bq769x2_defs.h` (new)

### 2026-02-03: Slave CAN Simulator
- Created `jfbms_slave_sim.lisp` for testing master CAN protocol without real battery hardware
- Simulates 32 cells (~3.65-3.75V with per-cell imbalance and random drift) and 4 temperatures
- Broadcasts at 10 Hz (100ms) matching protocol spec
- Uses `bms-init 16 16` to set C-level M_CELLS for correct CAN message count
- Changed default CMakeLists.txt build from master to slave
- Files: `jfbms_slave_sim.lisp` (new), `CMakeLists.txt`

### 2026-02-03: BMS_MAX_CELLS Increase & Buffer Overflow Fix
- Increased `BMS_MAX_CELLS` from 64 to 255 to support up to 8 slaves (8×32=256 cells)
- 255 chosen because VESC protocol sends cell_num as single byte (0-255)
- Fixed buffer overflow in `bms_process_cmd`: `send_buffer` was 256 bytes but 255 cells need ~964 bytes
- Changed `send_buffer` to `static uint8_t send_buffer[1024]` to avoid stack overflow
- Changed default CMakeLists.txt build target from slave to master
- Files modified: `datatypes.h`, `bms.c`, `main/CMakeLists.txt`

### 2026-02-04: Faster Slave Disconnect Detection & Buzzer Alert
- Slave connect/disconnect check now runs every loop iteration (100ms) instead of every 5 seconds
- Reduced slave timeout from 2000ms to 500ms for faster disconnect detection
- `master-send-balance` now always sends 5 bytes (balance mask + beep code)
- On slave disconnect, master sends buzzer alert (0x04 SHUTDOWN: 4 fast beeps) to all remaining active slaves
- Balance mask sent as 0 during alert; regular balancing loop restores correct mask on next cycle
- Added CAN RX processing to slave simulator (`jfbms_slave_sim.lisp`) for balance command and buzzer testing
- Files modified: `hw_jfbms_master.c`, `jfbms_master_main.lisp`, `jfbms_slave_sim.lisp`, `CLAUDE.md`

### 2026-02-04: Even/Odd Balancing Algorithm & 28-bit Integer Overflow Fix
- Replaced greedy no-adjacent-cell algorithm with even/odd group approach (matches Harmony32)
  - Cells split into even-indexed (0,2,4,...) and odd-indexed (1,3,5,...) groups
  - Pick group with more cells above threshold, select up to max_bal_ch highest voltage first
  - With N cells per IC, max balanced = floor(N/2) (16 cells → 8, 10 → 5)
- Fixed LispBM 28-bit integer overflow: `(shl mask 16)` silently truncated upper bits
  - `master-send-balance` now takes 4 args: `(slave-id ic1-mask ic2-mask beep-code)`, C combines them
  - `bms-set-bal-bitmap` now takes 2 args: `(ic1-mask ic2-mask)`, C combines them
  - `bms-set-bal-bitmap-demo` same change for simulator
  - No 32-bit integers constructed in Lisp anywhere; all combining done in C
- Balance cycle reduced from 8s to 1s for responsive VESC Tool updates
- Files modified: `hw_jfbms_master.c`, `hw_jfbms_slave.c`, `jfbms_master_main.lisp`, `jfbms_slave_main.lisp`, `jfbms_slave_sim.lisp`, `CLAUDE.md`

### 2026-02-09: Manual Balance Start/Stop Wiring in Master Script
- Added VESC Tool BMS command handling in master Lisp script:
  - Registered `event-bms-force-bal` and added `event-handler` thread
  - `BAL START/STOP` now controls a manual balance request gate (`bal-request`)
- Balancing thresholds remain package-config driven (`vc_balance_start`, `vc_balance_end`, `vc_balance_min`)
- Added `stop-all-balancing` helper to force local + slave balance outputs off when stopped
- Removed `event-bms-bal-ovr` control path so override payload does not change start/stop state
- Balancing now auto-stops when delta falls below end threshold and logs `BAL: target reached`
- Files modified: `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp`, `CLAUDE.md`
- Status: NOT TESTED on hardware yet

### 2026-02-09: CAN Buffer Overflow Prevention & Protocol Hardening
- Increased CAN RX buffer from 64 to 128 messages (`CAN_BUF_SIZE`)
- Master main loop now runs at 100 Hz (10ms) for CAN drain, with 10 Hz gating for VESC BMS updates/timeouts/disconnect checks
- Added DLC guard on slave balance command: checks `(>= (buflen data) 4)` before reading mask bytes
- Updated protocol doc: all buzzer beep codes are one-shot (master re-sends to sustain alerts)
- Restored truncated `jfbms_slave_main.lisp` diagnostic loop (was cut off at EOF since commit `7830da4`)
- Files modified: `hw_jfbms_master.c`, `jfbms_master_main.lisp`, `jfbms_slave_main.lisp`, `BMS_MASTER_SLAVE_PROTOCOL.md`
- Status: NOT TESTED on hardware yet

### 2026-02-10: Simultaneous Slave Balancing Fix & CAN RX Optimization
- Fixed slaves responding to balance commands with up to 2s stagger
- Root cause: slave `hw_can_rx_hook` buffered ALL CAN bus traffic (other slaves' broadcasts filled 16-msg buffer, dropping master's balance command)
- Fix: CAN RX hook now filters by slave ID — only buffers messages matching `0x500 | slave_id`, discards all other traffic
- Increased slave CAN RX buffer from 16 to 64 messages for safety margin
- Master balance thread split into 3 phases: compute all masks → send all commands in tight loop → debug print after
- Added `bal-request` check before send phase and post-cycle immediate stop for responsive BAL OFF
- Slave sim: added `bal-rx-flag` for immediate status broadcast after receiving balance command, added multiple `process-can-messages` calls per loop
- Slave main: same improvements (multiple CAN drains per loop, immediate broadcast on balance RX)
- Added balance mask change tracking in master main loop with timestamps for debugging
- Confirmed via timestamps: master sends to all slaves within 1-2ms, response within ~100ms (1 slave loop cycle)
- Files modified: `hw_jfbms_slave.c`, `jfbms_master_main.lisp`, `jfbms_slave_main.lisp`, `jfbms_slave_sim.lisp`
- Status: TESTED on simulator, working correctly

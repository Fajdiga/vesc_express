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
| `0x400 \| slave_id` | Temperatures | 8 | 4 temps (BQ1-int, Ext1/TS1, Ext2/TS3, BQ2-int) |
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
| 0 | T1 | BQ1 | BQ76952 #1 internal die temperature |
| 1 | T2 | Ext1 | External NTC on TS1 pin |
| 2 | T3 | Ext2 | External NTC on TS3 pin |
| 3 | T4 | BQ2 | BQ76952 #2 internal die temperature |

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
- Fixed cell dropout breaking balance mask alignment: invalid cells now occupy slot as `-1.0f` instead of being skipped
- Updated protocol doc: all buzzer beep codes are one-shot (master re-sends to sustain alerts)
- Restored truncated `jfbms_slave_main.lisp` diagnostic loop (was cut off at EOF since commit `7830da4`)
- Files modified: `hw_jfbms_master.c`, `jfbms_master_main.lisp`, `jfbms_slave_main.lisp`, `BMS_MASTER_SLAVE_PROTOCOL.md`
- Status: NOT TESTED on hardware yet

# JFBMS Project Status

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
| `0x480 \| slave_id` | Status | 5 | Balance mask (4B) + faults (1B) |
| `0x500 \| slave_id` | Balance Cmd | 4 | Master -> Slave balance command |

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
- `main/hwconf/jetfleet/jfbms_slave/can_demo.lisp` - Demo script (simulated data)

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
| `(master-send-balance slave-id mask)` | Send balance command to slave |
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

### 2026-01-13: Balancing Fixes, LispBM Corrections & Build Improvements

**BQ76952 Balancing Timeout Fix:**
- BQ76952 has internal ~18 second timeout for CB_ACTIVE_CELLS command
- Writing the SAME value does NOT reset the timeout - must use TOGGLE approach
- Fix: Write 0 first, then write actual mask to reset internal timer
- Adjacent cells cannot balance simultaneously (BQ76952 limitation) - use odd/even groups
- Master sends balance command every 1 second to slave
- Slave refreshes BQ76952 every 100ms with toggle approach

**LispBM Fixes:**
- `bitwise-or` only supports 2 arguments in LispBM - replaced with `+` for CAN ID calculation and byte extraction (works because bits don't overlap)
- `(systime)` resets each `loopwhile` iteration - use counter-based timing instead
- Global variables (`def`, `setq`, `set`) don't persist across loop iterations - use mutable list: `(def state (list 0))` with `(setix state 0 value)` to update
- `(to-str x)` includes type suffix like `"0u32"` - use `(str-from-n x "%d")` for clean output
- Fixed in both master and slave Lisp scripts

**Build System Improvements:**
- Added `build_both.bat` - builds both master and slave firmware
- Added `.vscode/settings.json` for proper ESP-IDF VS Code integration
- Fixed `${ProjectId}` error for other developers
- Increased `BMS_MAX_CELLS` from 50 to 64 (for dual BQ76952 config)
- Default CMakeLists.txt builds master (change paths for slave)

**Files Changed:**
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c` - Always refresh CB_ACTIVE_CELLS
- `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_main.lisp` - bitwise-or → +
- `main/hwconf/jetfleet/jfbms_slave/can_demo.lisp` - bitwise-or → +
- `main/hwconf/jetfleet/jfbms_master/jfbms_master_main.lisp` - Counter-based timing, mutable state list, str-from-n formatting
- `main/datatypes.h` - BMS_MAX_CELLS 50 → 64

---

### 2026-01-13: SOC/SOH Fix, Temperature Filtering & Slave ID from Config

**SOC/SOH Fix:**
- Fixed SOC and SOH calculation - VESC expects 0.0-1.0 (fraction), not 0-100 (percent)
- Was showing 1724% SOH due to incorrect scaling

**Temperature Improvements:**
- Invalid temperatures now correctly use `0x7FFF` marker instead of `-1.0°C`
- Changed `NAN_TO_M1` macro to `NAN_TO_INVALID` returning 999.0°C (outside valid range)
- Master filters out invalid temps and only displays valid ones
- Temperature naming in Lisp output:
  - `BQ1` = BQ76952 #1 internal die temperature
  - `Ext1` = External NTC on TS1 pin
  - `Ext2` = External NTC on TS3 pin
  - `BQ2` = BQ76952 #2 internal die temperature
- Note: VESC Tool still shows T1, T2, T3... (hardcoded in app)

**Slave Configuration:**
- Slave ID now reads from VESC Tool config via `(bms-get-slave-id)` instead of hardcoded value
- Fixed Lisp syntax errors (`var` → `def` for top-level, `spawn-trap` → `spawn`)

**Debug Features:**
- Added debug mode to master script showing raw CAN messages with parsed values
- Shows first 20 raw messages then switches to normal operation

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

### 2026-01-12: Master Implementation
- Created JFBMS Master device based on slave architecture
- Implemented CAN RX via LispBM event system (`event-can-sid`) - later found to be broken
- Added all master Lisp extensions for data access

### 2026-01-09: CAN Protocol Implementation
- Complete 11-bit CAN master-slave protocol
- 8 cell voltage messages (32 cells total)
- 1 temperature message (4 sensors)
- 1 status message (balance mask + faults)

### 2026-01-09: Slave ID Runtime Configuration
- Slave ID now configurable via VESC Tool without restart

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

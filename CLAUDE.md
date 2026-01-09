# JFBMS Project Status

## Current Stage: CAN Protocol Working

**Date:** 2026-01-09

### JFBMS Slave - 11-bit CAN Protocol

The BMS Master-Slave CAN protocol using 11-bit standard IDs is fully operational.

#### Protocol Specification

**CAN ID Format:** `(msg_type << 7) | slave_id`

| ID Formula | Message Type | DLC | Content |
|------------|--------------|-----|---------|
| `(0-7 << 7) \| slave_id` | Cell Voltages | 8 | 4 cells per msg, 8 msgs total (32 cells) |
| `0x400 \| slave_id` | Temperatures | 8 | 4 temps (T_BQ1, T_TS1, T_TS3, T_BQ2) |
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
```

#### Verified Working
- All 10 messages transmitted within ~2ms
- 32 cell voltages properly encoded
- 4 temperature readings properly encoded
- Status message with balance mask and fault flags
- No-ACK mode available for testing without second CAN device

#### Key Files
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c` - CAN protocol implementation
- `main/hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h` - Hardware configuration
- `main/hwconf/jetfleet/jfbms_slave/can_demo.lisp` - Demo script for protocol testing

#### Configuration Flags
- `HW_CAN_PING_SCAN_ENABLED 0` - Disables VESC CAN ping scan (slave uses 11-bit protocol)
- `HW_CAN_NO_ACK_MODE 1` - Enables no-ACK mode for standalone testing

> **IMPORTANT:** When testing is complete and a real master device is connected,
> set `HW_CAN_NO_ACK_MODE` back to `0` in `hw_jfbms_slave.h` and rebuild.
> No-ACK mode is for development/testing only. Production requires normal ACK mode
> for reliable CAN communication.

#### Slave ID Configuration

The slave ID is configurable via VESC Tool:
- **Location:** VESC Tool -> JFBMS Slave tab -> Slave ID (1-8)
- **Function:** `(bms-get-slave-id)` returns the configured value
- **Runtime changes:** The demo script reads slave ID on every broadcast, so changing ID in VESC Tool and clicking "Write" takes effect immediately without restart

**How it works:**
1. VESC Tool "Slave ID" field maps to `backup.config.slave_id`
2. `bms-get-slave-id` Lisp extension reads this value
3. `can_demo.lisp` calls `(bms-get-slave-id)` on each broadcast loop iteration

---

## Development History

### 2026-01-09: Slave ID Runtime Configuration
**Problem:** Changing slave ID in VESC Tool didn't affect CAN message IDs
**Root cause:** Demo script cached slave ID at startup with `(def demo-slave-id 1)`
**Solution:**
1. Updated `can_demo.lisp` to call `(bms-get-slave-id)` on every broadcast instead of caching
2. Confirmed `bms-get-slave-id` reads from `backup.config.slave_id` (VESC Tool field)

**Files changed:**
- `hw_jfbms_slave.c` - Verified `ext_get_slave_id` reads `cfg->slave_id`
- `can_demo.lisp` - Changed to read slave ID dynamically: `(bms-broadcast-all (bms-get-slave-id) cells temps 1 1)`

### 2026-01-09: CAN Protocol Implementation
**Added:** Complete 11-bit CAN master-slave protocol
- 8 cell voltage messages (32 cells total)
- 1 temperature message (4 sensors)
- 1 status message (balance mask + faults)
- No-ACK mode for standalone testing

---

## Next Steps
- [ ] Implement master-side CAN receiver
- [ ] Add balance command handling (0x500 | slave_id)
- [ ] Integration testing with real BMS hardware
- [ ] Disable no-ACK mode for production (set `HW_CAN_NO_ACK_MODE 0`)

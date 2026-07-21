# BMS Master-Slave CAN Protocol Specification

## Overview

11-bit standard CAN protocol for JFBMS32 master to communicate with slave BMS boards, enabling cell count expansion beyond 32 cells.

**CAN Bus:** 500 kbps

**Key Design Decisions:**

- Uses 11-bit standard CAN IDs (not 29-bit extended like VESC protocol)
- Slaves broadcast data periodically (no polling)
- Master sends only balance commands and buzzer notification codes
- Only sends cell voltage messages needed for configured cells (optimizes CAN bus load)
- Status message includes `cells_ic1`, `cells_ic2`, and external NTC enable bits
- Temperature frame keeps fixed BQ1/BQ2 positions; disabled NTCs use `0x7FFF`
- Data format matches BQ76952 native format for easy integration
- The configured topology uses contiguous slave IDs `1..N`. The master applies
  a TWAI acceptance mask where possible and always applies an exact software
  check; frames from IDs above `N` are ignored.

---

## System Architecture

```
┌─────────────────┐     CAN Bus (11-bit IDs)     ┌─────────────────┐
│   MASTER        │◄──────────────────────────────│   SLAVE 1       │
│   (JFBMS32)     │                               │   (Dual BQ76952)│
│                 │                               │   Cells 1-32    │
│ - FETs          │     ┌─────────────────┐       │   4 Temps       │
│ - Current sense │◄────│   SLAVE 2       │       └─────────────────┘
│ - Balancing     │     │   Cells 33-64   │
│   decisions     │     │   4 Temps       │
│ - SOC/SOH       │     └─────────────────┘
└─────────────────┘
```

### Master (JFBMS32)

- Has FETs (charge/discharge control)
- Has current sensor
- Makes all balancing decisions
- Aggregates cell data from all slaves
- Can work standalone (32 cells) or with slaves

### Slaves (Dual BQ76952 Boards)

- Dual BQ76952 chips (up to 32 cells, 16 per chip)
- Measure: cell voltages, BQ die temperatures, and configured external TS1 NTCs
- Execute: balance commands from master
- NO: FETs, current sensor, sleep/shutdown
- Address: Pre-configured (not auto-discovery)

**Note:** BQ76952 hardware protections are disabled. All protection logic is implemented in the master's LispBM script.

---

## CAN ID Structure (11-bit)

```
Bit:  10  9  8  7 │ 6  5  4 │ 3  2  1  0
     ─────────────┼─────────┼───────────
      Message     │ SubType │ Slave ID
      Type (4b)   │ (3b)    │ (4b)
```

### Slave ID (Bits 3-0)

| Value | Meaning |
|-------|---------|
| 0x0 | Reserved (not used) |
| 0x1-0x8 | Slave address 1-8 |
| 0x9-0xF | Reserved |

### Message Types (Bits 10-7)

| Type | Direction | Description |
|------|-----------|-------------|
| 0x0 | Slave→Master | Cell voltages 1-4 |
| 0x1 | Slave→Master | Cell voltages 5-8 |
| 0x2 | Slave→Master | Cell voltages 9-12 |
| 0x3 | Slave→Master | Cell voltages 13-16 |
| 0x4 | Slave→Master | Cell voltages 17-20 |
| 0x5 | Slave→Master | Cell voltages 21-24 |
| 0x6 | Slave→Master | Cell voltages 25-28 |
| 0x7 | Slave→Master | Cell voltages 29-32 |
| 0x8 | Slave→Master | Temperatures |
| 0x9 | Slave→Master | Status (balance state + faults + cells per IC + temp enable mask) |
| 0xA | Master→Slave | Balance command + buzzer beep code |
| 0xB-0xF | - | Reserved |

---

## Message Formats

### Cell Voltages (Types 0x0-0x7)

**CAN ID:** `(type << 7) | slave_id`

Each message carries 4 cell voltages:

```
Byte:   0      1      2      3      4      5      6      7
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │Cell 1│Cell 1│Cell 2│Cell 2│Cell 3│Cell 3│Cell 4│Cell 4│
     │[7:0] │[15:8]│[7:0] │[15:8]│[7:0] │[15:8]│[7:0] │[15:8]│
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 8 bytes

**Encoding:**

- 16-bit unsigned, little-endian
- Resolution: 1 mV per bit (matches BQ76952)
- Range: 0-65535 mV (0 V to 65.535 V)

**Special Values:**

| Value | Meaning |
|-------|---------|
| `0x0000` | Cell position not populated / unused slot in final frame |
| `0x0001` through `0xFFFE` | Cell voltage in millivolts |
| `0xFFFF` | Measurement or BQ communication failure |

**Cell Position Mapping:**

- Types 0x0-0x3 always belong to BQ1 cells 1-16
- Types 0x4-0x7 always belong to BQ2 cells 1-16
- Master uses `CellsIC1` and `CellsIC2` from the status frame as the authoritative cell counts
- Zero-filled CAN slots do not define cell count

Example with 5 cells on BQ1 and 5 cells on BQ2:

```
BQ1 wire slots:
[3500, 3501, 3499, 3500, 3502, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

BQ2 wire slots:
[3498, 3500, 3499, 3501, 3500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

Logical master display order:
BQ1 configured cells first, then BQ2 configured cells.
```

**Example:** Cell at 3.500 V

```c
// Sender (slave):
uint16_t voltage_mv = 3500;
data[0] = voltage_mv & 0xFF;        // 0xAC (low byte)
data[1] = (voltage_mv >> 8) & 0xFF; // 0x0D (high byte)

// Receiver (master):
uint16_t voltage_mv = data[0] | (data[1] << 8);  // = 3500 mV
float voltage_v = voltage_mv / 1000.0f;          // = 3.500 V
```

---

### Temperatures (Type 0x8)

**CAN ID:** `0x400 | slave_id`

```
Byte:   0      1      2      3      4      5      6      7
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │    T_BQ1    │  T_BQ1_TS1  │    T_BQ2    │  T_BQ2_TS1  │
     │    int16    │    int16    │    int16    │    int16    │
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 8 bytes

**Temperature Sensors (fixed positional order):**

| Bytes | Field | Requirement |
|-------|-------|-------------|
| 0-1 | T_BQ1 | BQ1 internal die temperature; mandatory |
| 2-3 | T_BQ1_TS1 | External NTC on BQ1 TS1 pin; enabled by status byte 7 bit 0 |
| 4-5 | T_BQ2 | BQ2 internal die temperature; mandatory when `CellsIC2 > 0` |
| 6-7 | T_BQ2_TS1 | External NTC on BQ2 TS1 pin; enabled by status byte 7 bit 1 |

**Encoding:**

- int16, little-endian, 0.1 °C resolution
- Value in units of 0.1 °C (e.g. 253 = 25.3 °C, -105 = -10.5 °C)
- `0x7FFF` (32767) = not present / intentionally disabled / invalid reading

The slave always keeps all four positions. It must not remove or shift an unused channel. Disabled external NTCs transmit `0x7FFF` in their fixed slot and clear their enable bit in the status frame. When BQ2 is not fitted, both BQ2 temperature positions are `0x7FFF`, and status byte 7 bit 1 is clear.

The master interprets `0x7FFF` according to the status frame:

| Channel | Status bit | Raw value | Master interpretation |
|---------|------------|-----------|-----------------------|
| External NTC | enable bit set | valid temperature | Valid sensor reading |
| External NTC | enable bit set | `0x7FFF` or out-of-range | Sensor fault; block charge/balance |
| External NTC | enable bit clear | `0x7FFF` | Intentionally unused; ignore only this channel |
| BQ die temperature | N/A | `0x7FFF` or out-of-range | BQ temperature-data fault |

**Example:** Temperature 25.3 °C

```c
int16_t temp = 253;  // 25.3°C × 10
data[0] = temp & 0xFF;         // 0xFD
data[1] = (temp >> 8) & 0xFF;  // 0x00
```

---

### Status (Type 0x9)

**CAN ID:** `0x480 | slave_id`

```
Byte:   0      1      2      3      4      5      6      7
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │BalMsk│BalMsk│BalMsk│BalMsk│Flags │IC1Cnt│IC2Cnt│TmpMsk│
     │ [7:0]│[15:8]│[23:16]│[31:24]│      │      │      │      │
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**DLC:** exactly 8 bytes

**Fields:**

| Bytes | Field | Description |
|-------|-------|-------------|
| 0-3 | BalanceMask | 32-bit bitmap, bit N = cell/channel N is currently balancing |
| 4 | Runtime flags | Fault and voltage-settled flags |
| 5 | CellsIC1 | Number of cells on BQ1 (0-16) |
| 6 | CellsIC2 | Number of cells on BQ2 (0-16, 0 = single chip) |
| 7 | TempSensorMask | External NTC enable flags |

**Runtime Flags Byte (Byte 4):**

| Bit | Meaning |
|-----|---------|
| 0 | BQ1 initialization/communication fault |
| 1 | BQ2 initialization/communication fault |
| 2 | Voltage-settled flag (1 = balance FETs off >= 2 s, voltages are accurate) |
| 3 | Balance-control write/watchdog fault; master must stop charge and balance |
| 4-7 | Reserved (set to 0) |

For a single-BQ slave, bit 1 remains clear and `CellsIC2` is zero.

**External Temperature Sensor Mask (Byte 7):**

| Bit | Meaning |
|-----|---------|
| 0 | BQ1 external TS1 NTC enabled |
| 1 | BQ2 external TS1 NTC enabled |
| 2-7 | Reserved (set to 0) |

Examples:

| TempSensorMask | Meaning |
|----------------|---------|
| `0x00` | No external NTCs enabled |
| `0x01` | BQ1 external NTC enabled |
| `0x02` | BQ2 external NTC enabled |
| `0x03` | Both external NTCs enabled |

When `CellsIC2` is zero, bit 1 is always transmitted clear even if the stored BQ2 sensor setting is enabled, because no BQ2 temperature channel exists.

**CellsIC1 / CellsIC2 (Bytes 5-6):**

Tells the master the exact cell configuration per BQ76952 chip. This is critical for:

- Displaying only actual cells in VESC Tool (no zeros for unpopulated positions)
- Building the correct balance mask (bits 0-15 = IC1, bits 16-31 = IC2)
- Determining which cell voltage CAN frames are required
- Determining whether BQ2 die/external temperature slots are expected
- Applying per-IC balance channel limits and adjacent-cell rules

The master accepts the status message only when DLC is exactly 8. Malformed status DLCs are rejected and do not refresh slave status.

**Note:** Single-chip slaves (no BQ2) set `CellsIC2 = 0`, send BQ2 temperatures as `0x7FFF`, clear TempSensorMask bit 1, and keep runtime fault bit 1 clear.

---

### Balance Command (Type 0xA)

**CAN ID:** `0x500 | slave_id` (slave_id = 1-8)

```
Byte:   0      1      2      3      4
     ┌──────┬──────┬──────┬──────┬──────┐
     │BalMsk│BalMsk│BalMsk│BalMsk│Buzzer│
     │ [7:0]│[15:8]│[23:16]│[31:24]│Code │
     └──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 5 bytes

**Fields:**

| Bytes | Field | Description |
|-------|-------|-------------|
| 0-3 | BalanceMask | 32-bit bitmap, bit N = balance cell/channel N |
| 4 | BuzzerCode | One-shot buzzer beep code (see table below) |

Master is responsible for respecting BQ chip limits - only set bits for cells that should actually balance.

**Buzzer Beep Codes (Byte 4):**

| Code | Name | Beep Pattern |
|------|------|--------------|
| 0x00 | NONE | No beep |
| 0x01 | POWER_ON | 2 short beeps |
| 0x02 | POWER_OFF | 1 long beep |
| 0x03 | CHARGE_COMPLETE | 3 short beeps |
| 0x04 | SHUTDOWN | 4 fast beeps |
| 0x10 | ERR_OVER_TEMP | 1 beep, pause, repeat while resent |
| 0x11 | ERR_CELL_HIGH | 2 beeps, pause, repeat while resent |
| 0x12 | ERR_CELL_LOW | 3 beeps, pause, repeat while resent |
| 0x13 | ERR_OVERCURRENT | 4 beeps, pause, repeat while resent |
| 0x14 | ERR_BQ_COMM | 5 beeps, pause, repeat while resent |

All nonzero beep codes play once per received command. For sustained error alerts, the master re-sends the error beep code with each balance command.

**Balance Watchdog:** Slave stops all balancing if no balance command is received for **3 seconds**. A C-side watchdog is independent of the Lisp control loop, and Lisp also checks elapsed monotonic time and local cell/temperature safety. Master must resend active balance commands every second.

---

## Timing

| Parameter | Value | Description |
|-----------|-------|-------------|
| Broadcast interval | 100 ms | Slaves send all data every 100 ms |
| Bus access | Automatic | CAN hardware handles arbitration - sends when bus is free |
| Slave freshness timeout | 1000 ms | A required frame older than this is considered stale |
| Slave offline debounce | 3 stale checks | Master marks slave offline only after consecutive stale checks |
| Balance update | As needed | Master sends when balance mask changes |
| Balance watchdog | 3 s | Slave C and Lisp watchdogs stop balancing if no command is received |

### Transmission Sequence (per slave, every 100 ms)

Each slave sends its messages back-to-back as fast as the CAN bus allows:

```
Cell voltages 1-4   (if BQ1 cells >= 1)
Cell voltages 5-8   (if BQ1 cells >= 5)
Cell voltages 9-12  (if BQ1 cells >= 9)
Cell voltages 13-16 (if BQ1 cells >= 13)
Cell voltages 17-20 (if BQ2 cells >= 1)
Cell voltages 21-24 (if BQ2 cells >= 5)
Cell voltages 25-28 (if BQ2 cells >= 9)
Cell voltages 29-32 (if BQ2 cells >= 13)
Temperatures
Status
```

**Message count depends on cell configuration:**

| Cells | Cell Messages | Total Messages | Messages/sec |
|-------|---------------|----------------|--------------|
| 12 | 3 | 5 | 50 |
| 16 | 4 | 6 | 60 |
| 20 | 5 | 7 | 70 |
| 32 | 8 | 10 | 100 |

**No artificial delays needed.** CAN arbitration automatically handles bus access:

- If bus is idle → transmit immediately
- If bus is busy → wait until idle, then transmit
- If collision → lower CAN ID wins, other retries immediately when bus is free

Normal CAN ACK behavior requires at least one other active CAN node to acknowledge transmitted frames.

---

## Balance Settle Synchronization

### Problem

BQ76952 balancing FETs cause voltage measurement errors while active. The internal balancing resistor draws current through the cell, causing an IR drop that makes the cell appear at a different voltage than its true open-circuit voltage. Using these "dirty" readings for balance decisions causes oscillation or over-balancing.

### Solution: Master-Driven Settle Cycle

The master coordinates a settle period before reading voltages for balance decisions. This mirrors the approach used by the VBMS32 (Harmony32) firmware.

**Balance cycle (repeated while balancing is active):**

```
1. Master sends zero balance mask to ALL slaves      (CAN TX, ~1 ms)
2. Master stops local BQ76952 balancing              (I2C, ~1 ms)
3. All devices settle for 2 seconds                  (FET-induced voltage error dissipates)
4. Slaves read settled voltages and broadcast them    (automatic, 10 Hz continuous)
5. Master drains CAN buffer to get latest settled slave voltages
6. Master reads local BQ76952 voltages (also settled)
7. Master computes new balance masks from settled data
8. Master sends new balance commands to slaves        (CAN TX)
9. Balancing runs until next cycle (~0.2 s cadence)
```

### Voltage-Settled Flag (Status Byte Bit 2)

Slaves track how long balance FETs have been off. After 20 consecutive loops (2 seconds at 10 Hz) with zero balance bitmap, the slave sets bit 2 in the status message flags byte. This flag indicates to the master that the voltages being broadcast are accurate (not affected by balancing).

**Flag behavior:**

| Condition | Settled Flag |
|-----------|--------------|
| No balance command ever received (boot) | 1 (settled) |
| Balance bitmap is zero for >= 2 s | 1 (settled) |
| Balance bitmap is non-zero | 0 (not settled) |
| Balance just stopped (< 2 s ago) | 0 (not settled) |

The master can check this flag via `(master-get-slave-settled? slave-id)` to verify data quality before computing balance masks. The helper returns true only when the slave is active, strictly fresh, settled, and not reporting BQ communication faults.

---

## CAN ID Quick Reference

### Slave→Master (Periodic Broadcasts, every 100 ms)

```
Cell Voltages 1-4:   0x001-0x008 (slave 1-8)
Cell Voltages 5-8:   0x081-0x088
Cell Voltages 9-12:  0x101-0x108
Cell Voltages 13-16: 0x181-0x188
Cell Voltages 17-20: 0x201-0x208
Cell Voltages 21-24: 0x281-0x288
Cell Voltages 25-28: 0x301-0x308
Cell Voltages 29-32: 0x381-0x388
Temperatures:        0x401-0x408
Status:              0x481-0x488
```

### Master→Slave (Commands)

```
Balance (slave 1):   0x501
Balance (slave 2):   0x502
...
Balance (slave 8):   0x508
```

---

## Freshness and Activation

At a 100 ms broadcast interval, the master stages each slave burst privately. Cell frame type 0 starts a candidate and the status frame ends it. The candidate is committed atomically only when all cell frames required by the status topology and the temperature frame arrived on the same CAN bus within 75 ms. Incomplete candidates never modify live safety data.

The master applies a fixed 300 ms safety freshness timeout from the last complete atomic commit and a separate 3-check offline debounce for display/presence state.

The master tracks two related states:

- **Fresh:** all required frames for the slave are inside the freshness timeout.
- **Active:** the slave is considered present. Once active, the master keeps the slave active through short stale gaps and only marks it offline after 3 consecutive stale checks.

A slave is fresh only when all of the following are fresh and valid:

1. Status frame with DLC exactly 8
2. Temperature frame with DLC 8
3. Every cell frame required by `CellsIC1` and `CellsIC2`
4. Cell counts within 0-16 per IC and a nonzero combined count
5. Temperature frame values consistent with `TempSensorMask`

Receiving unrelated frames does not make a slave fresh if any required frame has timed out.

The master keeps the last valid values visible while a previously active slave is inside the offline debounce window. Safety-critical balance decisions still require fresh, settled slave data.

Each configured slave must have a unique slave ID. Duplicate IDs cannot be reliably separated by CAN arbitration because frames with the same CAN ID are treated as the same message source.

The master exposes CAN RX overflow, complete, incomplete, timeout, restart, orphan, duplicate, malformed, commit-rate, age, bus-off recovery and TX-failure diagnostics. Any nonzero or increasing overflow counter indicates dropped frames and must be treated as a bus/load/firmware scheduling issue rather than normal arbitration.

The current wire format has no broadcast sequence number. Therefore the master cannot count every lost CAN frame exactly: a completely missing burst is indistinguishable from a slave that did not broadcast. Incomplete-burst counts and gaps in the complete-commit rate are loss estimates. Exact per-burst loss accounting would require a future slave protocol revision carrying a sequence number.

---

## Implementation Notes

### Master Integration

- Receives slave data via standard CAN RX
- Stages each slave broadcast and publishes cells, temperatures, topology and status as one atomic snapshot
- Uses `cells_ic1` and `cells_ic2` from status message to know exact IC configuration
- Uses `TempSensorMask` from status message to decide which external NTC slots are required
- Treats invalid enabled NTC readings as faults
- Ignores disabled external NTC slots only when they are `0x7FFF`
- Separates strict freshness from debounced presence so short gaps do not clear last-good display data
- Global min/max voltage across all slaves for balancing decisions
- Resends active balance commands every second to maintain balancing
- Retries balance command if status doesn't match expected

### Slave Implementation

- Initializes BQ76952 chips at startup, sets fault bits if init fails
- Reads BQ76952 at ~50 ms internally
- Broadcasts only required cell messages every 100 ms (optimizes CAN bus load)
- Sends `cells_ic1`, `cells_ic2`, and `TempSensorMask` in status message
- Sends disabled external NTC slots as `0x7FFF`
- Listens for balance commands matching its ID (`0x501`-`0x508`)
- Applies balance mask to BQ76952 `CB_ACTIVE_CELLS` register
- Stops balancing if no command is received for 3 seconds (independent C and Lisp watchdogs)

### Error Handling

| Condition | Slave Action | Master Action |
|-----------|--------------|---------------|
| BQ1 init failed | Set fault bit 0, send `0xFFFF` for BQ1 cells | Detect via flags byte, mark slave faulty |
| BQ2 init failed | Set fault bit 1, send `0xFFFF` for BQ2 cells | Detect via flags byte, mark slave faulty |
| Enabled external NTC open/short/invalid | Send `0x7FFF`, keep enable bit set | Set temperature-data fault; block charge/balance |
| External NTC intentionally absent | Send `0x7FFF`, clear enable bit | Ignore only that disabled external channel |
| Fitted BQ die temperature invalid | Send `0x7FFF` | Set temperature-data fault; block charge/balance |
| Malformed status DLC | N/A | Reject frame; do not refresh slave status |
| No active balance cmd (3 s) | Stop all balancing and report bit 3 if the hardware clear fails | N/A (master resends every second) |
| Slave data older than 300 ms | N/A | Immediately block charge/balance; retain last complete values for display only |
| Slave offline | N/A | Mark display/presence offline after 3 stale checks |
| Single-chip slave | BQ2 temps = `0x7FFF`, TempSensorMask bit 1 = 0, fault bit 1 = 0 | Normal operation (not an error) |

---

; JFBMS Slave Simulator - Sends realistic fake cell/temp data via CAN
; No BQ76952 hardware required - for testing master CAN protocol
; Hardcoded to 32 cells (16+16) to match dual BQ76952 configuration

; ============================================================================
; Configuration
; ============================================================================
(def slave-id (bms-get-slave-id))
(def total-cells 32)

; Initialize BMS with 16+16 cells so C-level M_CELLS is set correctly
; This makes can_send_all_cells send all 8 messages and status sends cell_count=32
(bms-init 16 16)

; ============================================================================
; Simulated Cell Base Voltages (mV, slightly different per cell)
; ============================================================================
; Range: ~3650mV to ~3750mV to mimic real pack imbalance
(def cell-bases (list
    3700.0 3749.0 3698.0 3747.0 3696.0 3745.0 3694.0 3743.0
    3692.0 3741.0 3690.0 3739.0 3688.0 3737.0 3686.0 3735.0
    3710.0 3659.0 3708.0 3657.0 3706.0 3655.0 3704.0 3653.0
    3702.0 3651.0 3720.0 3669.0 3718.0 3667.0 3716.0 3665.0
))

; ============================================================================
; Simulated Temperature Base Values (deg C)
; ============================================================================
; T0: BQ1 die ~32, T1: Ext1 NTC ~26, T2: Ext2 NTC ~27, T3: BQ2 die ~33
(def temp-bases (list 32.0 26.0 27.0 33.0))

; ============================================================================
; Simple pseudo-random number generator (state in mutable list)
; ============================================================================
(def rng-state (list 12345))

(defun next-rng () {
    (var s (ix rng-state 0))
    (setix rng-state 0 (mod (+ (* s 1103) 12345) 65536))
    (ix rng-state 0)
})

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 slave-id))
    (loopwhile (> (slave-can-available) 0) {
        (var msg (slave-can-read))
        (if msg {
            (var can-id (car msg))
            (var data (cdr msg))
            (if (= can-id expected-bal-id) {
                ; Extract 32-bit balance mask (little-endian)
                (var bal-mask (+ (bufget-u8 data 0)
                    (shl (bufget-u8 data 1) 8)
                    (shl (bufget-u8 data 2) 16)
                    (shl (bufget-u8 data 3) 24)))
                (print (str-merge "SIM BAL: 0x" (str-from-n bal-mask "%08X")))

                ; Extract buzzer beep code from byte 4 if present (DLC >= 5)
                (if (>= (buflen data) 5) {
                    (var beep-code (bufget-u8 data 4))
                    (if (not (= beep-code 0)) {
                        (print (str-merge "SIM BEEP: 0x" (str-from-n beep-code "%02X")))
                        (buzzer-beep 4 60)
                    })
                })
            })
        })
    })
})

; ============================================================================
; Main Simulation Loop - Broadcast every 100ms
; ============================================================================

(defun main-thd () {
    (var loop-count 0)

    (loopwhile t {
        ; Process incoming CAN messages (balance + buzzer commands from master)
        (process-can-messages)

        ; Build cell voltage list with random drift
        (var cells '())
        (looprange i 0 total-cells {
            (var base (ix cell-bases i))
            ; Random drift: +/-10mV
            (var r (next-rng))
            (var drift (- (* (/ (to-float r) 32768.0) 10.0) 10.0))
            (var v (+ base drift))
            ; Clamp 3000-4200 mV
            (if (< v 3000.0) (setq v 3000.0))
            (if (> v 4200.0) (setq v 4200.0))
            ; Convert mV to V
            (setq cells (append cells (list (/ v 1000.0))))
        })

        ; Build temperature list with random drift +/-0.5 deg C
        (var temps '())
        (looprange i 0 4 {
            (var tb (ix temp-bases i))
            (var r (next-rng))
            (var drift (- (* (/ (to-float r) 32768.0) 0.5) 0.5))
            (setq temps (append temps (list (+ tb drift))))
        })

        ; Debug print every 100 loops (10 seconds)
        (setq loop-count (+ loop-count 1))
        (if (= (mod loop-count 100) 0) {
            (print (str-merge "SIM loop " (str-from-n loop-count "%d")
                              " cells:" (str-from-n total-cells "%d")
                              " V0=" (str-from-n (ix cells 0) "%.3f")
                              "V V1=" (str-from-n (ix cells 1) "%.3f") "V"))
        })

        ; Broadcast via CAN
        (bms-broadcast-all slave-id cells temps true true)

        ; Update local VESC BMS display
        (slave-update-vesc-bms cells temps)

        ; 100ms interval (10 Hz, matches protocol spec)
        (sleep 0.1)
    })
})

; ============================================================================
; Startup
; ============================================================================

(print "=== JFBMS Slave SIMULATOR ===")
(print (str-merge "Slave ID: " (str-from-n slave-id "%d")))
(print (str-merge "Simulated cells: " (str-from-n total-cells "%d")))
(print "Broadcasting fake data at 10 Hz (100ms)...")

(spawn 150 main-thd)

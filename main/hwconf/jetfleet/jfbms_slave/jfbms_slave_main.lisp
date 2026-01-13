; JFBMS Slave - CAN Protocol Implementation
; Broadcasts cell voltages and temperatures to master BMS via 11-bit CAN protocol
; Receives balance commands from master

; ============================================================================
; SLAVE CONFIGURATION - Read from VESC Tool configuration
; ============================================================================
(def slave-id (bms-get-slave-id))  ; Configured in VESC Tool -> JFBMS Slave -> Slave ID

; ============================================================================
; Cell Configuration (read from stored config)
; ============================================================================
(def cells-ic1 (bms-get-param 'cells_ic1))
(def cells-ic2 (bms-get-param 'cells_ic2))
(def total-cells (+ cells-ic1 cells-ic2))

; ============================================================================
; Balance Watchdog
; ============================================================================
; Master must send balance command at least every 10 seconds
; If no command received, stop all balancing for safety
(def last-bal-cmd-time 0)
(def bal-watchdog-timeout 10.0)

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
; Balance command CAN ID: 0x500 | slave_id
; Data: 4 bytes balance bitmap (little-endian uint32)
; Uses direct CAN buffer (bypasses broken event-can-sid system)

(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 slave-id))
    ; Process all available CAN messages
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
                (bms-set-bal-bitmap bal-mask)
                ; Reset watchdog timer
                (setq last-bal-cmd-time (systime))
                (print (str-merge "BAL CMD: mask=0x" (str-from-n bal-mask "%08X")))
            })
        })
    })
})

; ============================================================================
; Balance Watchdog Check
; ============================================================================
(defun check-bal-watchdog () {
    ; If we received a balance command and timeout has elapsed, stop balancing
    (if (and (> last-bal-cmd-time 0)
             (> (- (systime) last-bal-cmd-time) bal-watchdog-timeout))
        {
            (bms-stop-balancing)
            (setq last-bal-cmd-time 0)
            (print "Balance watchdog triggered - stopped balancing")
        })
})

; ============================================================================
; Main Thread - Broadcast data every 100ms
; ============================================================================

(defun main-thd () {
    ; Track BQ communication status
    (var bq1-ok true)
    (var bq2-ok (if (> cells-ic2 0) true false))
    (var loop-count 0)

    (loopwhile t {
        ; Process incoming CAN messages (balance commands from master)
        (process-can-messages)

        ; Read cell voltages from both BQ chips
        (var cells (bms-get-vcells))
        (if (eq cells nil) {
            (setq bq1-ok false)
            (setq cells '())
        } (setq bq1-ok true))

        ; Read temperatures (BQ1-Int, TS1, TS3, BQ2-Int)
        (var temps (bms-get-temps))
        (if (eq temps nil)
            (setq temps '(-273.0 -273.0 -273.0 -273.0)))

        ; Check BQ2 status from temperature (invalid if < -200)
        (if (> cells-ic2 0) {
            (if (and (>= (length temps) 4) (> (ix temps 3) -200.0))
                (setq bq2-ok true)
                (setq bq2-ok false))
        })

        ; Debug: print every 10 loops (1 second)
        (setq loop-count (+ loop-count 1))
        (if (= (mod loop-count 10) 0) {
            (print (str-merge "Loop " (to-str loop-count) " - Cells: " (to-str (length cells))
                              ", BalMask: 0x" (str-from-n (bms-get-bal-bitmap) "%08X")))
        })

        ; Broadcast all data via CAN (8 cell msgs + 1 temp + 1 status)
        (bms-broadcast-all slave-id cells temps bq1-ok bq2-ok)

        ; Check balance watchdog
        (check-bal-watchdog)

        ; 100ms loop (10 Hz broadcast rate per protocol spec)
        (sleep 0.1)
    })
})

; ============================================================================
; Startup
; ============================================================================

(print "JFBMS Slave starting...")
(print (str-merge "Slave ID: " (to-str slave-id)))
(print (str-merge "Cells IC1: " (to-str cells-ic1) ", IC2: " (to-str cells-ic2)))

; Check if I2C device is present at 0x08
(print (str-merge "I2C detect 0x08: " (to-str (i2c-detect-addr 0x08))))

; Initialize BMS hardware
(def init-ok false)
(def bq1-init-ok false)
(def bq2-init-ok false)

(looprange i 0 10 {
    (if (bms-init cells-ic1 cells-ic2) {
        (setq init-ok true)
        (setq bq1-init-ok true)
        (setq bq2-init-ok (if (> cells-ic2 0) true false))
        (break)
    } {
        (print (str-merge "BMS init failed, attempt " (to-str (+ i 1)) ", retrying..."))
        (sleep 1.0)
    })
})

; Set fault flags based on init status
(def fault-flags 0)
(if (not bq1-init-ok) (setq fault-flags (+ fault-flags 0x01)))
(if (and (> cells-ic2 0) (not bq2-init-ok)) (setq fault-flags (+ fault-flags 0x02)))
(bms-set-fault-flags fault-flags)

(if init-ok
    (print "BMS initialized successfully")
    (print "BMS init failed after 10 attempts - check I2C wiring and BQ76952 power"))

; Only continue if init was successful
(if init-ok {
    ; Start main broadcast loop (CAN RX is polled in main loop)
    (print "Starting CAN broadcast loop...")
    (spawn 150 main-thd)
} {
    ; Init failed - enter diagnostic loop, but still broadcast status
    (print "Entering diagnostic mode due to init failure")
    (loopwhile t {
        (print (str-merge "I2C detect 0x08: " (to-str (i2c-detect-addr 0x08))))
        ; Broadcast empty data with fault flags
        (bms-broadcast-all slave-id '() '() false false)
        (sleep 1.0)
    })
})

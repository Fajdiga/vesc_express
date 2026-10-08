; JFBMS Slave - CAN Protocol Implementation
; Broadcasts cell voltages and temperatures to master BMS via 11-bit CAN protocol
; Receives balance commands from master

; ============================================================================
; SLAVE CONFIGURATION - Read from VESC Tool configuration
; ============================================================================
(def slave-id (bms-get-slave-id))  ; Configured in VESC Tool -> JFBMS Slave -> Slave ID
(def broadcast-period-s 0.1)       ; Total loop period: 100 ms = 10 Hz.

; ============================================================================
; Cell Configuration (read from stored config)
; ============================================================================
(def cells-ic1 (bms-get-param 'cells_ic1))
(def cells-ic2 (bms-get-param 'cells_ic2))
(def temp-bq1-en (bms-get-param 'temp_bq1_en))
(def temp-bq2-en (bms-get-param 'temp_bq2_en))
(def total-cells (+ cells-ic1 cells-ic2))

; ============================================================================
; Balance watchdog and local measurement gate
; ============================================================================
; Master must send balance command at least every 3 seconds
; If no command received, stop all balancing for safety
(def bal-state (list 0))  ; state[0] = 1 while a nonzero mask is active
(def bal-last-rx-ts (list (systime)))
; Flag: set to 1 when a balance command was received in process-can-messages
(def bal-rx-flag (list 0))
; Previous balance masks for change detection (only print when mask changes)
(def prev-ic1-mask (list 0))
(def prev-ic2-mask (list 0))
; Previous status flags printed to terminal
(def prev-status-flags (list -1))
; Settle tracking: both ICs on one slave are synchronized.
; After 20 loops (2s at 10Hz) voltages are considered settled for master
; balance decisions. Each slave reports this flag independently.
(def settle-counter (list 20))  ; Start settled (no balancing at boot)
(def last-cells '())
(def last-temps '())
(def last-data-ts (systime))
(def active-hw-config nil)
(def debug-balancing false) ; Enable periodic mask diagnostics only when needed.

; As with JFBMS32, save definitions once and run main again on image boot.
@const-start

str-merge
str-from-n
defun
loopwhile
looprange
loopforeach
map

(defun bms-hw-config () (map (fn (param) (bms-get-param param))
    '(cells_ic1 cells_ic2 temp_bq1_en temp_bq2_en temp_res temp_beta slave_id)))

(defun load-hw-config (cfg) {
    (setq cells-ic1 (ix cfg 0))
    (setq cells-ic2 (ix cfg 1))
    (setq temp-bq1-en (ix cfg 2))
    (setq temp-bq2-en (ix cfg 3))
    (setq slave-id (ix cfg 6))
    (setq total-cells (+ cells-ic1 cells-ic2))
})

(defun config-current () (and (bms-hw-ready)
    (eq active-hw-config (bms-hw-config))))

(defun clear-local-state () {
    (setq last-cells nil)
    (setq last-temps nil)
    (setq last-data-ts (systime))
    (setix bal-state 0 0)
    (setix prev-ic1-mask 0 0)
    (setix prev-ic2-mask 0 0)
    (setix prev-status-flags 0 -1)
    (setix settle-counter 0 0)
    (bms-set-settled-flag 0)
    ; Old queued balance commands must not be replayed after reinitialization.
    (loopwhile (> (slave-can-available) 0) (slave-can-read))
})

(defun init-hw () {
    (trap-value '(bms-stop-balancing) false)
    (clear-local-state)
    (var attempts 0)
    (var ready false)
    (loopwhile (not ready) {
        (var cfg (bms-hw-config))
        (load-hw-config cfg)
        (bms-set-fault-flags (if (> cells-ic2 0) 0x03 0x01))
        (setq ready (and
            (trap-value `(bms-init ,cells-ic1 ,cells-ic2) false)
            (eq cfg (bms-hw-config))
            (bms-hw-ready)))
        (if ready {
            (setq active-hw-config cfg)
            (clear-local-state)
            (bms-set-fault-flags 0)
            (print "BMS initialized; settings applied automatically")
        } {
            (setq attempts (+ attempts 1))
            ; Keep reporting faults during recovery, and keep trying even if
            ; startup failed. Read live settings again on every attempt.
            (trap-value `(bms-broadcast-all ,slave-id () () 0 ,(if (> cells-ic2 0) 0 1)) false)
            (if (or (= attempts 1) (= (mod attempts 5) 0))
                (print "BQ initialization pending, retry" attempts))
            (sleep (if (< attempts 5) 1.0 3.0))
        })
    })
})

(defun trap-value (expr fallback) {
    (match (trap (eval expr))
        ((exit-ok (? value)) value)
        (_ fallback)
    )
})

; Pace the complete broadcast loop to a 100 ms deadline. A fixed 100 ms
; sleep after the I2C/CAN work makes the actual rate slower than 10 Hz. If the
; work overruns the deadline, do not add delay; the loop remains fail-safe and
; runs at the maximum rate the hardware can sustain.
(defun sleep-to-broadcast-deadline (loop-start) {
    (var remaining (- broadcast-period-s (secs-since loop-start)))
    (if (> remaining 0.0)
        (sleep remaining)
    )
})

(defun local-balance-safe () {
    (var safe (and
        (config-current)
        (= (length last-cells) total-cells)
        (>= (length last-temps) (if (> cells-ic2 0) 4 2))
        (< (secs-since last-data-ts) 0.5)
    ))
    (if safe {
        (loopforeach v last-cells {
            (if (not (and (number? v) (>= v 2.5) (<= v 5.0))) (setq safe false))
        })
        ; IC die temperatures (indices 0 and 2) are mandatory. External
        ; NTCs (indices 1 and 3) are checked only when configured; disabled
        ; sensors intentionally carry the invalid -273 C marker.
        (var die1 (ix last-temps 0))
        (if (not (and (number? die1) (>= die1 -40.0) (<= die1 100.0))) (setq safe false))
        ; bms-get-param returns numeric 0/1. In LispBM, numeric 0 is still
        ; truthy, so compare explicitly before validating an external NTC.
        (if (= temp-bq1-en 1) {
            (var ext1 (ix last-temps 1))
            (if (not (and (number? ext1) (>= ext1 -40.0) (<= ext1 60.0))) (setq safe false))
        })
        (if (> cells-ic2 0) {
            (var die2 (ix last-temps 2))
            (if (not (and (number? die2) (>= die2 -40.0) (<= die2 100.0))) (setq safe false))
            (if (= temp-bq2-en 1) {
                (var ext2 (ix last-temps 3))
                (if (not (and (number? ext2) (>= ext2 -40.0) (<= ext2 60.0))) (setq safe false))
            })
        })
    })
    safe
})

; Balance visualization: show which cells are balancing per IC
; mask = 16-bit balance mask, ncells = number of configured cells
; Returns e.g. "_.X.X._" for cells 2,4 balancing out of 7
(defun bal-cells-str (mask ncells) {
    (var s "")
    (var active 0)
    (looprange i 0 ncells {
        (if (= (bitwise-and (shr mask i) 1) 1)
            { (setq s (str-merge s "X")) (setq active (+ active 1)) }
            (setq s (str-merge s "_"))
        )
    })
    (str-merge s " " (str-from-n active "%d") "/" (str-from-n ncells "%d"))
})

(defun tx-status-flags (bq1-ok bq2-ok) {
    (var flags 0)

    (if (not bq1-ok) (setq flags (+ flags 0x01)))
    ; CellsIC2 = 0 means there is no BQ2 fitted, not a BQ2 fault.
    (if (and (> cells-ic2 0) (not bq2-ok)) (setq flags (+ flags 0x02)))

    (if (>= (ix settle-counter 0) 20) (setq flags (+ flags 0x04)))

    flags
})

(defun print-tx-status (flags bq1-ok bq2-ok reason) {
    (print (str-merge "STATUS TX " reason
        " flags=0x" (str-from-n flags "%02X")
        " bq1=" (if bq1-ok "ok" "ERR")
        " bq2=" (if (> cells-ic2 0) (if bq2-ok "ok" "ERR") "none")
        " settled=" (if (> (bitwise-and flags 0x04) 0) "1" "0")
        " ic1=" (str-from-n cells-ic1 "%d")
        " ic2=" (str-from-n cells-ic2 "%d")))
})

(defun maybe-print-tx-status (bq1-ok bq2-ok force) {
    (var flags (tx-status-flags bq1-ok bq2-ok))

    (if (or force (not-eq flags (ix prev-status-flags 0))) {
        (print-tx-status flags bq1-ok bq2-ok (if force "repeat" "change"))
        (setix prev-status-flags 0 flags)
    })

    flags
})

(defun bool-u8 (v) (if v 1 0))

; ============================================================================
; Buzzer Beep Code Handler
; ============================================================================
; Beep codes received in byte 4 of balance command (DLC 5)
; Patterns defined here so they can be modified without reflashing firmware

(defun handle-beep (code)
    (cond
        ((= code 0x01) (buzzer-beep 2 100))    ; POWER_ON: 2 short beeps
        ((= code 0x02) (buzzer-beep 1 500))    ; POWER_OFF: 1 long beep
        ((= code 0x03) (buzzer-beep 3 100))    ; CHARGE_COMPLETE: 3 short beeps
        ((= code 0x04) (buzzer-beep 4 60))     ; SHUTDOWN: 4 fast beeps
        ((= code 0x10) (buzzer-beep 1 200))    ; ERR_OVER_TEMP: 1 beep
        ((= code 0x11) (buzzer-beep 2 200))    ; ERR_CELL_HIGH: 2 beeps
        ((= code 0x12) (buzzer-beep 3 200))    ; ERR_CELL_LOW: 3 beeps
        ((= code 0x13) (buzzer-beep 4 200))    ; ERR_OVERCURRENT: 4 beeps
        ((= code 0x14) (buzzer-beep 5 200))    ; ERR_BQ_COMM: 5 beeps
    )
)

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
; Balance command CAN ID: 0x500 | slave_id
; Data: 4 bytes balance bitmap (little-endian uint32) + optional byte 4 buzzer code
; Uses direct CAN buffer (bypasses broken event-can-sid system)

(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 slave-id))
    (var rx-count 0)
    ; Process all available CAN messages
    (loopwhile (> (slave-can-available) 0) {
        (var msg (slave-can-read))
        (if msg {
            (setq rx-count (+ rx-count 1))
            (var can-id (car msg))
            (var data (cdr msg))
            (if (and (= can-id expected-bal-id)
                    (or (= (buflen data) 4) (= (buflen data) 5))) {
                ; Extract IC1 and IC2 masks separately (16-bit each, avoids 28-bit overflow)
                (var ic1-mask (+ (bufget-u8 data 0) (shl (bufget-u8 data 1) 8)))
                (var ic2-mask (+ (bufget-u8 data 2) (shl (bufget-u8 data 3) 8)))
                (var old-bal-active (or (> (ix prev-ic1-mask 0) 0)
                                         (> (ix prev-ic2-mask 0) 0)))
                (var new-bal-active (or (> ic1-mask 0) (> ic2-mask 0)))
                (var mask-changed (or (not-eq ic1-mask (ix prev-ic1-mask 0))
                                      (not-eq ic2-mask (ix prev-ic2-mask 0))))
                ; Nonzero commands require a recent local voltage/temperature
                ; sample. A zero command is always accepted as the safe action.
                (var result (if (or (not new-bal-active) (local-balance-safe))
                    (trap-value `(bms-set-bal-bitmap ,ic1-mask ,ic2-mask) false)
                    false
                ))
                (setix bal-rx-flag 0 1)
                (if result {
                    ; Only nonzero masks need keepalives. A zero-mask command
                    ; has already made the hardware safe and must not arm a
                    ; watchdog that prints a false timeout later.
                    (setix bal-state 0 (if new-bal-active 1 0))
                    (if new-bal-active (setix bal-last-rx-ts 0 (systime)))
                } {
                    (trap-value '(bms-stop-balancing) false)
                    (setix bal-state 0 0)
                })

                ; Any transition into or out of active balancing makes voltages
                ; unsettled immediately. Only the zero-mask settle timer can set
                ; this true again.
                (if (and result (or old-bal-active new-bal-active)) {
                    (setix settle-counter 0 0)
                    (bms-set-settled-flag 0)
                })

                ; Only print when mask actually changes
                (if mask-changed {
                    (if result {
                        (setix prev-ic1-mask 0 ic1-mask)
                        (setix prev-ic2-mask 0 ic2-mask)
                        (print (str-merge "BAL: IC1=[" (bal-cells-str ic1-mask cells-ic1)
                            "] IC2=[" (bal-cells-str ic2-mask cells-ic2) "]"))
                    }
                        (print (str-merge "BAL FAIL: IC1=0x" (str-from-n ic1-mask "%04X")
                            " IC2=0x" (str-from-n ic2-mask "%04X"))))
                })

                ; Extract buzzer beep code from byte 4 if present (DLC >= 5)
                (if (>= (buflen data) 5) {
                    (var beep-code (bufget-u8 data 4))
                    (if (not (= beep-code 0)) (handle-beep beep-code))
                })
            })
        })
    })
    rx-count
})

; ============================================================================
; Balance watchdog check
; ============================================================================
; Three-second elapsed-time timeout, independent of loop execution time.
(def bal-watchdog-timeout-s 3.0)
(defun check-bal-watchdog () {
    (if (> (ix bal-state 0) 0) {
        (if (or (> (secs-since (ix bal-last-rx-ts 0)) bal-watchdog-timeout-s)
                (not (local-balance-safe))) {
            (trap-value '(bms-stop-balancing) false)
            (setix bal-state 0 0)
            (setix prev-ic1-mask 0 0)
            (setix prev-ic2-mask 0 0)
            (setix settle-counter 0 0)
            (bms-set-settled-flag 0)
            (print "Balance watchdog/safety gate triggered - stopped balancing")
        })
    })
})

; ============================================================================
; Main Thread - Broadcast data every 100ms
; ============================================================================

(defun main-thd () {
    ; Track BQ communication status
    (var bq1-ok true)
    (var bq2-ok true)
    (var loop-count 0)
    (var bq-fail-count 0)

    (loopwhile t {
        (var loop-start (systime))
        (if (not (config-current)) {
            (print "BMS settings changed, reinitializing")
            (exit-error 2)
        })
        (setix bal-rx-flag 0 0)

        ; Process incoming CAN messages (balance commands from master)
        (process-can-messages)

        ; Read cell voltages from both BQ chips
        (var cells (trap-value '(bms-get-vcells) nil))
        (if (eq cells nil) {
            (setq bq1-ok false)
            (setq cells '())
        } (setq bq1-ok true))

        ; Check CAN again after slow I2C reads so balance commands aren't delayed
        (process-can-messages)

        ; Read temperatures (BQ1-Int, BQ1-TS1, BQ2-Int, BQ2-TS1)
        (var temps (trap-value '(bms-get-temps) nil))
        (if (eq temps nil)
            (setq temps '(-273.0 -273.0 -273.0 -273.0)))

        ; Cell reads alone do not prove BQ1 health. Its mandatory internal
        ; temperature must also be present so persistent temperature-bus
        ; failures enter the same supervised hardware reinitialization path.
        (setq bq1-ok (and bq1-ok (>= (length temps) 2)
            (> (ix temps 0) -200.0)))

        (if (and (= (length cells) total-cells)
                (>= (length temps) (if (> cells-ic2 0) 4 2))) {
            (setq last-cells cells)
            (setq last-temps temps)
            (setq last-data-ts (systime))
        })

        ; Check CAN again after temperature I2C reads
        (process-can-messages)

        ; Check BQ2 status from its mandatory IC die temperature. Index 3 is
        ; the optional external sensor and is intentionally invalid when off.
        (if (> cells-ic2 0) {
            (if (and (>= (length temps) 4) (> (ix temps 2) -200.0))
                (setq bq2-ok true)
                (setq bq2-ok false))
        } {
            ; Single-chip slaves report CellsIC2 = 0, not a BQ2 fault.
            (setq bq2-ok true)
        })

        (if (and bq1-ok bq2-ok)
            (setq bq-fail-count 0)
            (setq bq-fail-count (+ bq-fail-count 1))
        )
        (if (or (>= bq-fail-count 20) (not (config-current))) {
            (trap-value '(bms-stop-balancing) false)
            (print "Persistent BQ communication fault - reinitializing")
            (exit-error 1)
        })

        ; Track settle state for master balance synchronization. Both ICs on
        ; this slave are synchronized, while different slaves settle separately.
        (var bal-bmp-now (bms-get-bal-bitmap))
        (if (= bal-bmp-now 0) {
            ; Balance is off - increment settle counter toward threshold
            (if (< (ix settle-counter 0) 20)
                (setix settle-counter 0 (+ (ix settle-counter 0) 1)))
            (if (>= (ix settle-counter 0) 20)
                (bms-set-settled-flag 1))
        } {
            ; Balance is active - voltages are not settled
            (setix settle-counter 0 0)
            (bms-set-settled-flag 0)
        })

        (maybe-print-tx-status bq1-ok bq2-ok false)

        ; If a balance command was received, broadcast status immediately
        ; so master sees the updated balance mask without waiting for next cycle
        (if (= (ix bal-rx-flag 0) 1)
            (bms-broadcast-all slave-id cells temps (bool-u8 bq1-ok) (bool-u8 bq2-ok)))

        ; Debug: print every 10 loops (1 second)
        (setq loop-count (+ loop-count 1))
        (if (and debug-balancing (= (mod loop-count 10) 0)) {
            ; Get balance bitmap and split into IC1/IC2 masks
            (var bal-bmp (bms-get-bal-bitmap))
            (var ic1-mask (bitwise-and bal-bmp 0xFFFF))
            (var ic2-mask (bitwise-and (shr bal-bmp 16) 0xFFFF))
            (var tx-flags (tx-status-flags bq1-ok bq2-ok))

            (print (str-merge "BQ1 bal: [" (bal-cells-str ic1-mask cells-ic1) "]"
                " 0x" (str-from-n ic1-mask "%04X")))
            (if (> cells-ic2 0)
                (print (str-merge "BQ2 bal: [" (bal-cells-str ic2-mask cells-ic2) "]"
                    " 0x" (str-from-n ic2-mask "%04X"))))
            (if (> (bitwise-and tx-flags 0x03) 0)
                (maybe-print-tx-status bq1-ok bq2-ok true))
        })

        ; Broadcast all data via CAN (cell msgs + 1 temp + 1 status)
        (bms-broadcast-all slave-id cells temps (bool-u8 bq1-ok) (bool-u8 bq2-ok))

        ; Update local VESC BMS values (for VESC Tool display when connected to slave)
        (slave-update-vesc-bms cells temps)

        ; Check balance watchdog
        (check-bal-watchdog)

        ; Keep the complete loop at 100 ms (10 Hz nominal), including the
        ; I2C reads and CAN transmission time.
        (sleep-to-broadcast-deadline loop-start)
    })
})

(defun main-supervisor () {
    (loopwhile t {
        (match (trap {
            (init-hw)
            (main-thd)
        })
            ((exit-ok _) (print "Slave controller exited, restarting"))
            (_ (print "Slave controller recovering; balancing off"))
        )
        (trap-value '(bms-stop-balancing) false)
        (setq active-hw-config nil)
        (sleep 0.2)
    })
})

(defun main () {
    (print "JFBMS Slave starting...")
    (loopwhile (!= (bms-fw-version) 7) {
        (trap-value '(bms-stop-balancing) false)
        (print (if (< (bms-fw-version) 7)
            "Firmware too old; update slave firmware"
            "Package too old; update slave Lisp application"))
        (sleep 5.0)
    })
    ; Config values in a saved image belong to image creation, not this boot.
    (load-hw-config (bms-hw-config))
    (setq active-hw-config nil)
    (clear-local-state)
    (main-supervisor)
})

@const-end

(image-save)
(main)

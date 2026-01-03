; JFBMS Slave - Data streaming script for master-slave BMS topology
; This slave BMS reads cell voltages and temperatures and streams them
; to the master BMS via custom CAN protocol. Master handles all safety logic.
;
; CAN Protocol (11-bit Standard IDs):
;   Slave -> Master (0x600 range):
;     0x01 = Cell voltages (3 per message, uint16 mV)
;     0x02 = Temperatures (4 temps, int16 0.1C)
;     0x03 = Status (cell count, flags, bal bitmap)
;   Master -> Slave (0x700 range):
;     0x10 = Balancing command (bitmap)

; ============================================================================
; Configuration
; ============================================================================

(def cells-ic1 (bms-get-param 'cells_ic1))
(def cells-ic2 (bms-get-param 'cells_ic2))
(def slave-id (bms-get-slave-id))
(def total-cells (+ cells-ic1 cells-ic2))

; ============================================================================
; CAN RX Handler for Master Commands
; ============================================================================

; Handle incoming CAN messages from master
(defun handle-can-rx (can-id data) {
    ; Extract packet type and target slave from CAN ID
    ; Format: 0x700 + slave_id + (packet_type << 4)
    (var pkt-type (shr (bitwise-and can-id 0xF0) 4))
    (var target-slave (bitwise-and can-id 0x0F))

    ; Only process if addressed to us (or broadcast 0x0F)
    (if (or (= target-slave slave-id) (= target-slave 0x0F)) {
        (if (= pkt-type 0x10) {
            ; Balancing command from master
            (var cmd (bufget-u8 data 0))
            (if (= cmd 0x00)
                ; Stop all balancing
                (bms-stop-balancing)
                ; Set balancing bitmap
                (if (= cmd 0x01) {
                    (var new-bitmap (bufget-u32 data 1))
                    (bms-set-bal-bitmap new-bitmap)
                })
            )
        })
    })
})

; Event handler thread for CAN RX
(defun event-handler ()
    (event-enable 'event-can-sid)
    (loopwhile t
        (recv
            ((event-can-sid (? id) (? data))
                ; Check if it's in master command range (0x700-0x7FF)
                (if (and (>= id 0x700) (<= id 0x7FF))
                    (handle-can-rx id data)))
            ((? x) nil)  ; Ignore other events
        )))

; ============================================================================
; Main Thread
; ============================================================================

(defun main-thd () {
    ; Track BQ communication status
    (var bq1-ok true)
    (var bq2-ok (> cells-ic2 0))

    (loopwhile t {
        ; Read cell voltages from both BQ chips
        (var cells (bms-get-vcells))
        (if (eq cells nil) {
            (setq bq1-ok false)
            (setq cells '())
        } (setq bq1-ok true))

        ; Read temperatures (BQ1-Int, TS1, TS3, BQ2-Int)
        (var temps (bms-get-temps))
        (if (eq temps nil)
            (setq temps '(-1.0 -1.0 -1.0 -1.0)))

        ; Check BQ2 status from temperature (BQ2-Int = -1.0 if not present or error)
        (if (and (> cells-ic2 0) (>= (ix temps 3) 0.0))
            (setq bq2-ok true)
            (if (> cells-ic2 0) (setq bq2-ok false)))

        ; Update VESC BMS values for VESC Tool display (USB/WiFi direct connection)
        (bms-update-vesc cells temps)

        ; Optionally send to master via custom CAN protocol (uncomment if needed)
        ; (bms-can-send-cells slave-id cells)
        ; (bms-can-send-temps slave-id temps)
        ; (bms-can-send-status slave-id bq1-ok bq2-ok)

        ; 1 Hz loop
        (sleep 1.0)
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
(looprange i 0 10 {
    (if (bms-init cells-ic1 cells-ic2) {
        (setq init-ok true)
        (break)
    } {
        (print (str-merge "BMS init failed, attempt " (to-str (+ i 1)) ", retrying..."))
        (sleep 1.0)
    })
})

(if init-ok
    (print "BMS initialized successfully")
    (print "BMS init failed after 10 attempts - check I2C wiring and BQ76952 power"))

; Only continue if init was successful
(if init-ok {
    ; Start event handler thread for CAN RX
    (spawn 64 event-handler)

    ; Main loop with error recovery
    (loopwhile t {
        (spawn-trap "main-thd" main-thd)
        (recv
            ((exit-error (? tid) (? e)) {
                (print (str-merge "Main thread error: " (to-str e)))
                (bms-stop-balancing)  ; Safety: stop balancing on error
                (sleep 1.0)
                ; Re-initialize BMS
                (looprange j 0 5 {
                    (if (bms-init cells-ic1 cells-ic2)
                        (break)
                        (print "BMS re-init failed, retrying..."))
                    (sleep 1.0)
                })
            })
            ((exit-ok (? tid) (? v)) {
                (print "Main thread exited normally, restarting...")
                (sleep 1.0)
            })
        )
    })
} {
    ; Init failed - enter diagnostic loop
    (print "Entering diagnostic mode due to init failure")
    (loopwhile t {
        (print (str-merge "I2C detect 0x08: " (to-str (i2c-detect-addr 0x08))))
        (sleep 5.0)
    })
})

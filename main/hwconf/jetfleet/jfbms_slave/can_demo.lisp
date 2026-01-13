; JFBMS Slave CAN Protocol Demo Script
; Generates simulated cell voltages and temperatures, broadcasts via 11-bit CAN protocol
;
; Usage:
;   1. Load this script to the device
;   2. Demo will start automatically, broadcasting every 200ms
;   3. Use (demo-stop) to stop, (demo-start) to restart
;   4. Adjust parameters below as needed

; ============================================================================
; Configuration
; ============================================================================

(def demo-running true)           ; Set to false to stop demo
(def demo-interval 0.2)           ; Broadcast interval in seconds (200ms)
(def demo-slave-id (bms-get-slave-id))  ; Slave ID from VESC Tool (JFBMS Slave tab)

; Cell configuration
(def demo-num-cells 32)           ; Number of cells to simulate (max 32)
(def demo-cell-nominal 3.70)      ; Nominal cell voltage (V)
(def demo-cell-variation 0.05)    ; Random variation +/- (V)

; Temperature configuration (4 sensors: T_BQ1, T_TS1, T_TS3, T_BQ2)
(def demo-temp-nominal 25.0)      ; Nominal temperature (C)
(def demo-temp-variation 5.0)     ; Random variation +/- (C)

; Balance state (simulated, updated by master commands)
(def demo-bal-mask 0)

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
; Uses direct CAN buffer (bypasses broken event-can-sid system)

(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 demo-slave-id))
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
                ; Set balance mask directly (demo mode - no BQ chips)
                (bms-set-bal-bitmap-demo bal-mask)
                (print (str-merge "BAL CMD: mask=0x" (str-from-n bal-mask "%08X")))
            })
        })
    })
})

; ============================================================================
; Helper Functions
; ============================================================================

; Generate random variation using systime for pseudo-randomness
(defun rand-variation (idx)
    (/ (mod (+ (systime) (* idx 137)) 1000) 1000.0))

; Generate a single simulated cell voltage with index-based variation
(defun gen-cell-voltage (idx)
    (+ demo-cell-nominal
       (* (- (* (rand-variation idx) 2.0) 1.0) demo-cell-variation)))

; Generate list of 32 simulated cell voltages (iterative, no recursion)
; Cells beyond demo-num-cells are set to 0 (not populated)
(defun gen-cell-list-32 () {
    (var cells nil)
    (looprange i 0 32 {
        (if (< i demo-num-cells)
            (setq cells (cons (gen-cell-voltage i) cells))
            (setq cells (cons 0 cells))
        )
    })
    (reverse cells)
})

; Generate a single simulated temperature with index-based variation
(defun gen-temp (idx)
    (+ demo-temp-nominal
       (* (- (* (rand-variation (+ idx 100)) 2.0) 1.0) demo-temp-variation)))

; Generate list of 4 temperatures (T_BQ1, T_TS1, T_TS3, T_BQ2)
(defun gen-temp-list ()
    (list (gen-temp 0) (gen-temp 1) (gen-temp 2) (gen-temp 3)))

; ============================================================================
; Demo Control Functions
; ============================================================================

(defun demo-stop ()
    (def demo-running false)
    (print "Demo stopped"))

(defun demo-start ()
    (def demo-running true)
    (print "Demo started"))

(defun demo-set-interval (interval)
    (def demo-interval interval)
    (print (str-from-n interval "Interval set to %.3f s")))

(defun demo-set-cells (num)
    (def demo-num-cells num)
    (print (str-from-n num "Cell count set to %d")))

(defun demo-set-slave-id (id)
    (def demo-slave-id id)
    (print (str-from-n id "Slave ID set to %d")))

; ============================================================================
; Main Demo Loop
; ============================================================================

(print "=== JFBMS Slave CAN Demo ===")
(print (str-from-n demo-slave-id "Slave ID: %d"))
(print (str-from-n demo-num-cells "Cells: %d"))
(print (str-from-n demo-interval "Interval: %.2f s"))
(print "Commands: (demo-stop) (demo-start) (demo-set-interval x)")
(print "Starting broadcast...")

(loopwhile t {
    (if demo-running {
        ; Process incoming CAN messages (balance commands from master)
        (process-can-messages)

        ; Generate simulated data (32 cells, 4 temps)
        (var cells (gen-cell-list-32))
        (var temps (gen-temp-list))

        ; Broadcast via 11-bit CAN protocol
        ; bms-broadcast-all sends: 8 cell msgs + 1 temp msg + 1 status msg
        ; Balance mask is read from internal state (set by bms-set-bal-bitmap-demo)
        (bms-broadcast-all (bms-get-slave-id) cells temps 1 1)

        ; Debug output (comment out for silent operation)
        ; (print (str-from-n (ix cells 0) "Cell 1: %.3f V"))
    })

    (sleep demo-interval)
})

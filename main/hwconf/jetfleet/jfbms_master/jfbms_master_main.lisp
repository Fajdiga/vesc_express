; JFBMS Master
; Receives cell voltages from slaves via CAN, manages centralized balancing
; Displays in VESC Tool

(print "=== JFBMS Master ===")

; Config
(def slave-timeout 500)

; Reset slave data
(master-reset-slaves)

; Previous active state for each slave (1-8)
; Using mutable list for loop persistence
(def prev-active (list 0 0 0 0 0 0 0 0))

; Balancing state - mutable list for loop persistence
; state[0] = 1 if currently balancing, 0 if not
(def bal-state (list 0))
; True when manual balance was requested from VESC Tool
(def bal-request false)

; ============================================================================
; Balance Helper: compute balance mask for one BQ76952 IC group
; ============================================================================
; BQ76952 cannot balance adjacent cells simultaneously.
; Solution: split cells into even-indexed (0,2,4,...) and odd-indexed (1,3,5,...)
; groups. No cell within a group is adjacent, so the constraint is guaranteed.
; Pick the group with more cells above threshold, then select up to max-ch
; from that group (highest voltage first).
; With N cells per IC, max balanced = floor(N/2) (e.g. 16 cells -> 8, 10 -> 5)
;
; voltages: list of cell voltages (V) for this IC
; c-min: global minimum cell voltage
; threshold: voltage delta above c-min to trigger balancing
; max-ch: maximum simultaneous balance channels for this IC
; Returns: bitmask of cells to balance (bit N = cell N in this IC)

(defun balance-ic-group (voltages c-min threshold max-ch) {
    (var n (length voltages))
    (if (= n 0) 0 {
        ; Split cells above threshold into even and odd index groups
        (var even-grp '())
        (var odd-grp '())
        (looprange i 0 n {
            (var v (ix voltages i))
            (if (> (- v c-min) threshold) {
                (if (= (mod i 2) 0)
                    (setq even-grp (cons (cons i v) even-grp))
                    (setq odd-grp (cons (cons i v) odd-grp))
                )
            })
        })

        ; Sort each group by voltage descending (balance highest first)
        (var even-sorted (sort (fn (a b) (> (cdr a) (cdr b))) even-grp))
        (var odd-sorted (sort (fn (a b) (> (cdr a) (cdr b))) odd-grp))

        ; Pick group with more cells above threshold
        ; Tie-break: group with higher total delta from min
        (var use-even true)
        (if (> (length odd-sorted) (length even-sorted))
            (setq use-even false)
        )
        (if (= (length odd-sorted) (length even-sorted)) {
            (var even-sum 0.0)
            (loopforeach p even-sorted
                (setq even-sum (+ even-sum (- (cdr p) c-min))))
            (var odd-sum 0.0)
            (loopforeach p odd-sorted
                (setq odd-sum (+ odd-sum (- (cdr p) c-min))))
            (if (> odd-sum even-sum) (setq use-even false))
        })

        (var grp (if use-even even-sorted odd-sorted))

        ; Select up to max-ch from chosen group
        (var mask 0)
        (var cnt 0)
        (loopforeach c grp {
            (if (>= cnt max-ch) (break))
            (setq mask (+ mask (shl 1 (car c))))
            (setq cnt (+ cnt 1))
        })
        mask
    })
})

; ============================================================================
; Balance Debug: print which cells are selected on an IC
; ============================================================================
; label: string like "S1-BQ1" or "Local"
; voltages: list of cell voltages for this IC
; mask: bitmask of cells being balanced
; c-min: global minimum voltage

(defun print-bal-ic (label voltages mask c-min) {
    (var n (length voltages))
    (var cnt 0)
    (looprange i 0 n {
        (if (> (bitwise-and mask (shl 1 i)) 0) {
            (print (str-merge "  " label " C" (str-from-n i "%d")
                ": " (str-from-n (ix voltages i) "%.3fV")
                " (+" (str-from-n (* (- (ix voltages i) c-min) 1000.0) "%.0fmV") ")"))
            (setq cnt (+ cnt 1))
        })
    })
    cnt
})

; Stop local and slave balancing outputs
(defun stop-all-balancing () {
    (var local-ic1 (bms-get-param 'cells_ic1))
    (var local-ic2 (bms-get-param 'cells_ic2))
    (var local-total (+ local-ic1 local-ic2))

    (if (> local-total 0) {
        (trap (looprange i 0 local-total (bms-set-bal i 0)))
    })

    (var sid 1)
    (loopwhile (<= sid 8) {
        (if (master-slave-active? sid)
            (master-send-balance sid 0 0 0)
        )
        (setq sid (+ sid 1))
    })

    (setix bal-state 0 0)
})

; Handle balance start/stop commands from VESC Tool
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-force-bal (? v)) {
                    (setq bal-request (= v 1))
                    (if bal-request
                        (print "BAL CMD: start")
                        (print "BAL CMD: stop")
                    )
            })
            (_ nil)
)))

; ============================================================================
; Balance Thread
; ============================================================================
; Every second: read cached slave voltages, compute balance masks, send.
; No pause needed for slave cells (they send voltages continuously via CAN).
; Only local BQ needs pause for clean I2C readings (when wired).
; 1s cycle keeps us within the slave's 10s balance watchdog.

(defun balance-thd () (loopwhile t {
    ; Read package config each iteration so threshold changes take effect
    (var max-ch (bms-get-param 'max_bal_ch))
    (var bal-start (bms-get-param 'vc_balance_start))
    (var bal-end (bms-get-param 'vc_balance_end))
    (var bal-min (bms-get-param 'vc_balance_min))
    (var local-ic1 (bms-get-param 'cells_ic1))
    (var local-ic2 (bms-get-param 'cells_ic2))
    (var local-total (+ local-ic1 local-ic2))
    (var is-bal (= (ix bal-state 0) 1))
    (var threshold (if is-bal bal-end bal-start))

    (if (not bal-request) {
        ; Keep outputs off until user requests balancing from VESC Tool
        (if (= (ix bal-state 0) 1) {
            (print "BAL: stopped by command")
        })
        (stop-all-balancing)
    } {
        ; --- Step 1: Read all cell voltages and find global minimum ---
        (var global-min 9.0)
        (var global-max 0.0)
        (var any-cells false)

        ; Read local BQ cells (if configured)
        ; Uses trap to handle BQ not wired/initialized - errors won't crash thread
        (var local-cells nil)
        (if (> local-total 0) {
            (match (trap {
                (looprange i 0 local-total (bms-set-bal i 0))
                (sleep 2.0)
                (bms-get-vcells)
            })
                ((exit-ok (? cells)) {
                    (if (and cells (> (length cells) 0)) {
                        (setq local-cells cells)
                        (setq any-cells true)
                        (loopforeach v local-cells {
                            (if (< v global-min) (setq global-min v))
                            (if (> v global-max) (setq global-max v))
                        })
                    })
                })
                (_ nil) ; BQ not wired or comm error - skip local cells
            )
        })

        ; Read slave cells from CAN cache (no pause needed)
        ; Store as list of (slave-id cells ic1-count ic2-count)
        (var slave-data '())
        (var sid 1)
        (loopwhile (<= sid 8) {
            (if (master-slave-active? sid) {
                (var cells (master-get-slave-cells sid))
                (var s-ic1 (master-get-cells-ic1 sid))
                (var s-ic2 (master-get-cells-ic2 sid))
                (var cnt (+ s-ic1 s-ic2))
                (if (and cells (> (length cells) 0) (= (length cells) cnt)) {
                    (setq any-cells true)
                    (setq slave-data (cons (list sid cells s-ic1 s-ic2) slave-data))
                    (loopforeach v cells {
                        (if (< v global-min) (setq global-min v))
                        (if (> v global-max) (setq global-max v))
                    })
                })
            })
            (setq sid (+ sid 1))
        })

        ; --- Debug: print voltage summary ---
        (print "--- BAL cycle ---")
        (if any-cells {
            (print (str-merge "Vmin=" (str-from-n global-min "%.3fV")
                " Vmax=" (str-from-n global-max "%.3fV")
                " spread=" (str-from-n (* (- global-max global-min) 1000.0) "%.0fmV")
                " thr=" (str-from-n (* threshold 1000.0) "%.0fmV")
                " max_ch=" (str-from-n max-ch "%d")))
        } (print "No cells available"))

        ; --- Step 2: Check if balancing is allowed ---
        (var bal-allowed (and
                any-cells
                (> global-min bal-min)
        ))

        (if (and any-cells (not bal-allowed))
            (print (str-merge "BAL blocked: Vmin " (str-from-n global-min "%.3fV")
                " < bal_min " (str-from-n bal-min "%.3fV")))
        )

        (if bal-allowed {
            ; --- Step 3: Balance each IC group ---
            (var total-bal-cells 0)

            ; Local BQ (single IC on master) - trap in case BQ not wired
            (if (and local-cells (> (length local-cells) 0)) {
                (var local-mask (balance-ic-group local-cells global-min threshold max-ch))
                (trap (looprange i 0 (length local-cells) {
                    (bms-set-bal i (if (> (bitwise-and local-mask (shl 1 i)) 0) 1 0))
                }))
                (if (> local-mask 0) {
                    (print "Local BQ:")
                    (setq total-bal-cells (+ total-bal-cells
                        (print-bal-ic "Local" local-cells local-mask global-min)))
                })
            })

            ; Slave ICs
            (loopforeach sd slave-data {
                (var s-id (ix sd 0))
                (var cells (ix sd 1))
                (var ic1-cnt (ix sd 2))
                (var ic2-cnt (ix sd 3))

                ; Extract IC1 voltages (cells 0 to ic1-cnt-1)
                (var ic1-volts (map (fn (i) (ix cells i)) (range ic1-cnt)))

                ; Extract IC2 voltages (cells ic1-cnt to total-1)
                (var ic2-volts (if (> ic2-cnt 0)
                    (map (fn (i) (ix cells (+ ic1-cnt i))) (range ic2-cnt))
                    '()
                ))

                ; Compute balance masks per IC
                (var ic1-mask (balance-ic-group ic1-volts global-min threshold max-ch))
                (var ic2-mask (if (> ic2-cnt 0)
                    (balance-ic-group ic2-volts global-min threshold max-ch)
                    0
                ))

                ; Send IC1 and IC2 masks separately to C code (avoids 28-bit overflow)
                ; C code combines: mask = ic1_mask | (ic2_mask << 16)
                (master-send-balance s-id ic1-mask ic2-mask 0)

                ; Debug: print per-IC details
                (var s-label (str-merge "S" (str-from-n s-id "%d")))
                (if (> ic1-mask 0) {
                    (print (str-merge s-label "-BQ1 (" (str-from-n ic1-cnt "%d") " cells):"))
                    (setq total-bal-cells (+ total-bal-cells
                        (print-bal-ic (str-merge s-label "-BQ1") ic1-volts ic1-mask global-min)))
                })
                (if (> ic2-mask 0) {
                    (print (str-merge s-label "-BQ2 (" (str-from-n ic2-cnt "%d") " cells):"))
                    (setq total-bal-cells (+ total-bal-cells
                        (print-bal-ic (str-merge s-label "-BQ2") ic2-volts ic2-mask global-min)))
                })

                (print (str-merge s-label " IC1=0x" (str-from-n ic1-mask "%04X")
                    " IC2=0x" (str-from-n ic2-mask "%04X")))
            })

            ; Update balancing state
            (if (> total-bal-cells 0) {
                (setix bal-state 0 1)
                (print (str-merge "BAL TOTAL: " (str-from-n total-bal-cells "%d") " cells"))
            } {
                (setix bal-state 0 0)
                (setq bal-request false)
                (print "BAL: target reached")
            })
        } {
            ; Balancing blocked by voltage/cell availability. Stop and end request.
            (stop-all-balancing)
            (setq bal-request false)
            (if any-cells
                (print "BAL: stopped (conditions not met)")
                (print "BAL: stopped (no cells available)")
            )
        })
    })

    ; --- Step 4: Wait before next cycle ---
    ; 1 second for responsive updates, well within slave's 10s balance watchdog
    (sleep 1.0)
}))

; ============================================================================
; Main Loop
; ============================================================================

; Register BMS command events used by VESC Tool
(event-register-handler (spawn event-handler))
(event-enable 'event-bms-force-bal)

; Spawn balance thread
(spawn 200 balance-thd)

; Main loop - CAN drain at 100 Hz, everything else at 10 Hz
(var loop-cnt 0)
(loopwhile t {
    ; Drain CAN buffer every 10ms (100 Hz) - prevents overflow with multiple slaves
    (master-can-read-all)

    ; Run 10 Hz tasks every 10th iteration (100ms)
    (if (= (mod loop-cnt 10) 0) {
        ; Update VESC BMS display with slave cell voltages
        (master-update-vesc-bms)

        ; Check for timed-out slaves
        (master-check-timeouts slave-timeout)

        ; Check each slave 1-8 for connect/disconnect
        (var id 1)
        (loopwhile (<= id 8) {
            (var active (if (master-slave-active? id) 1 0))
            (var prev (ix prev-active (- id 1)))

            (if (and (= active 1) (not-eq prev 1))
                (print (str-merge "Slave " (str-from-n id "%d") " is connected"))
            )
            (if (and (= active 0) (= prev 1)) {
                (print (str-merge "Slave " (str-from-n id "%d") " disconnected"))
                ; Alert all remaining active slaves with buzzer (SHUTDOWN: 4 fast beeps)
                (var alert-id 1)
                (loopwhile (<= alert-id 8) {
                    (if (and (not-eq alert-id id) (master-slave-active? alert-id))
                        (master-send-balance alert-id 0 0 0x04)
                    )
                    (setq alert-id (+ alert-id 1))
                })
            })

            (setix prev-active (- id 1) active)
            (setq id (+ id 1))
        })
    })

    (setq loop-cnt (+ loop-cnt 1))
    ; 10ms loop (100 Hz CAN drain, 10 Hz everything else)
    (sleep 0.01)
})

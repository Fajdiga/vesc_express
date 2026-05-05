; JFBMS Master
; Receives cell voltages from slaves via CAN, manages centralized balancing
; Displays in VESC Tool

(print "=== JFBMS Master ===")

; COM enable low (active)
(gpio-write 6 0)

; Buzzer on GPIO8 — 4 kHz carrier, duty 0 (silent until beep command)
(pwm-start 4000 0.0 0 8)

; Config
(def slave-timeout 500)

; Reset slave data
(master-reset-slaves)

; Charge allow state — true means charge FET enabled (default OFF at boot;
; user must press "Chg En" in VESC Tool to enable charging)
(def chg-allowed false)

; Coulomb / energy counters (reset on power-cycle, no NVS persistence)
(def ah-cnt 0.0)
(def wh-cnt 0.0)

; Previous active state for each slave (1-8)
; Using mutable list for loop persistence
(def prev-active (list 0 0 0 0 0 0 0 0))

; Balancing state - mutable list for loop persistence
; state[0] = 1 if currently balancing, 0 if not
(def bal-state (list 0))
; True when manual balance was requested from VESC Tool
(def bal-request false)
; Cached slave masks, updated by balance thread and transmitted by main loop keepalive
(def slave-bal-mask-ic1 (list 0 0 0 0 0 0 0 0))
(def slave-bal-mask-ic2 (list 0 0 0 0 0 0 0 0))
; One-shot request for immediate keepalive TX (removes 0-1s phase delay on start)
(def bal-keepalive-kick (list 0))

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

; Convert balance mask to binary string (cell 0 on left)
(defun mask-to-bin (mask n) {
    (var s "")
    (looprange i 0 n {
        (setq s (str-merge s (if (> (bitwise-and mask (shl 1 i)) 0) "1" "0")))
    })
    s
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

    (looprange i 0 8 {
        (setix slave-bal-mask-ic1 i 0)
        (setix slave-bal-mask-ic2 i 0)
    })

    (setix bal-keepalive-kick 0 0)
    (setix bal-state 0 0)
})

; Handle BMS Data tab button events from VESC Tool
; Note: event-bms-zero-ofs is sent by C as a bare symbol (no cons wrapper),
; so its match pattern must be the bare symbol, not (event-bms-zero-ofs).
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-force-bal (? v)) {
                    (setq bal-request (= v 1))
                    (if bal-request {
                        (setix bal-keepalive-kick 0 1)
                        (print "BAL CMD: start")
                    } {
                        (print "BAL CMD: stop")
                    })
            })
            ((event-bms-chg-allow (? allow)) {
                    (setq chg-allowed (= allow 1))
                    (bms-set-chg (if chg-allowed 1 0))
                    (set-bms-val 'bms-chg-allowed (if chg-allowed 1 0))
                    (print (str-merge "CHG: " (if chg-allowed "enabled" "disabled")))
            })
            ((event-bms-reset-cnt (? ah) (? wh)) {
                    (if (= ah 1) {
                        (setq ah-cnt 0.0)
                        (set-bms-val 'bms-ah-cnt 0.0)
                    })
                    (if (= wh 1) {
                        (setq wh-cnt 0.0)
                        (set-bms-val 'bms-wh-cnt 0.0)
                    })
                    (print (str-merge "RESET CNT: ah=" (str-from-n ah "%d")
                        " wh=" (str-from-n wh "%d")))
            })
            (event-bms-zero-ofs {
                    (print "CAL: zero current...")
                    (master-calibrate-current)
            })
            (_ nil)
)))

; ============================================================================
; Balance Thread - 3 Phase: Settle -> Compute -> Hold
; ============================================================================
; Phase 1 (Settle): Stop all balancing, poll slave settled flags + 2s local min
; Phase 2 (Compute): Read settled voltages, compute balance masks, apply
; Phase 3 (Hold): Let balancing run ~30s, main loop sends keepalive at 1Hz
; Cell voltage monitoring at 20Hz continues in main loop regardless of phase.

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
        ; Stop once when transitioning from balancing to idle (not every loop)
        (if (= (ix bal-state 0) 1) {
            (print "BAL: stopped by command")
            (stop-all-balancing)
        })
        (sleep 0.2)
    } {
        ; ===== PHASE 1: SETTLE =====
        ; Send zero to slaves so they stop balancing and voltages settle
        (var settle-sid 1)
        (loopwhile (<= settle-sid 8) {
            (if (master-slave-active? settle-sid)
                (master-send-balance settle-sid 0 0 0))
            (setq settle-sid (+ settle-sid 1))
        })
        ; Clear cached masks so main loop keepalive sends zero during settle
        (looprange i 0 8 {
            (setix slave-bal-mask-ic1 i 0)
            (setix slave-bal-mask-ic2 i 0)
        })
        ; Stop local balance
        (if (> local-total 0) {
            (trap (looprange i 0 local-total (bms-set-bal i 0)))
        })

        ; Poll until all active slaves report settled (min 2s for local BQ, 5s timeout)
        (var settle-wait 0)
        (var max-settle-wait 50)  ; 50 x 100ms = 5s timeout
        (loopwhile (and bal-request (< settle-wait max-settle-wait)) {
            (sleep 0.1)
            (master-can-read-all)
            (setq settle-wait (+ settle-wait 1))
            ; Need at least 2s for local BQ settle
            (if (>= settle-wait 20) {
                ; Check if all active slaves report settled
                (var all-settled true)
                (var chk-sid 1)
                (loopwhile (<= chk-sid 8) {
                    (if (and (master-slave-active? chk-sid)
                             (not (master-get-slave-settled? chk-sid)))
                        (setq all-settled false))
                    (setq chk-sid (+ chk-sid 1))
                })
                (if all-settled (break))
            })
        })

        ; Final CAN drain to get latest settled voltages
        (master-can-read-all)

        ; ===== PHASE 2: COMPUTE =====
        (if bal-request {
            ; --- Read all cell voltages and find global minimum ---
            (var global-min 9.0)
            (var any-cells false)

            ; Read local BQ cells (settled after 2s)
            (var local-cells nil)
            (if (> local-total 0) {
                (match (trap (bms-get-vcells))
                    ((exit-ok (? cells)) {
                        (if (and cells (> (length cells) 0)) {
                            (setq local-cells cells)
                            (setq any-cells true)
                            (loopforeach v local-cells {
                                (if (< v global-min) (setq global-min v))
                            })
                        })
                    })
                    (_ (print "BAL: local BQ comm failed"))
                )
            })

            ; Read slave cells from CAN cache (settled, confirmed by flag)
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
                        })
                    })
                })
                (setq sid (+ sid 1))
            })

            ; --- Check if balancing is allowed ---
            (var bal-allowed (and any-cells (> global-min bal-min)))

            (if bal-allowed {
                ; --- Compute balance masks ---
                (var any-bal false)

                ; Local BQ
                (if (and local-cells (> (length local-cells) 0)) {
                    (var local-mask (balance-ic-group local-cells global-min threshold max-ch))
                    (trap (looprange i 0 (length local-cells) {
                        (bms-set-bal i (if (> (bitwise-and local-mask (shl 1 i)) 0) 1 0))
                    }))
                    (if (> local-mask 0) {
                        (setq any-bal true)
                        (print (str-merge "BAL LOCAL: " (mask-to-bin local-mask local-total)
                            " min=" (str-from-n global-min "%.3f")))
                    })
                })

                ; Slave ICs: update cached masks for 1Hz keepalive in main loop
                (loopforeach sd slave-data {
                    (var s-id (ix sd 0))
                    (var cells (ix sd 1))
                    (var ic1-cnt (ix sd 2))
                    (var ic2-cnt (ix sd 3))

                    (var ic1-volts (map (fn (i) (ix cells i)) (range ic1-cnt)))
                    (var ic2-volts (if (> ic2-cnt 0)
                        (map (fn (i) (ix cells (+ ic1-cnt i))) (range ic2-cnt))
                        '()
                    ))

                    (var ic1-mask (balance-ic-group ic1-volts global-min threshold max-ch))
                    (var ic2-mask (if (> ic2-cnt 0)
                        (balance-ic-group ic2-volts global-min threshold max-ch)
                        0
                    ))

                    (if (or (> ic1-mask 0) (> ic2-mask 0)) {
                        (setq any-bal true)
                        (print (str-merge "BAL S" (str-from-n s-id "%d")
                            " IC1:" (mask-to-bin ic1-mask ic1-cnt)
                            " IC2:" (mask-to-bin ic2-mask ic2-cnt)
                            " min=" (str-from-n global-min "%.3f")))
                    })
                    (setix slave-bal-mask-ic1 (- s-id 1) ic1-mask)
                    (setix slave-bal-mask-ic2 (- s-id 1) ic2-mask)
                })

                (if any-bal {
                    (setix bal-state 0 1)
                    ; Kick immediate keepalive so slaves get masks without 1Hz delay
                    (setix bal-keepalive-kick 0 1)

                    ; ===== PHASE 3: HOLD =====
                    ; Let balancing run ~30s. Main loop sends keepalive at 1Hz.
                    ; Check bal-request each second for responsive stop.
                    (var hold-cnt 0)
                    (loopwhile (and bal-request (< hold-cnt 30)) {
                        (sleep 1.0)
                        (setq hold-cnt (+ hold-cnt 1))
                    })
                } {
                    (setix bal-state 0 0)
                    (setq bal-request false)
                    (print "BAL: target reached")
                })
            } {
                ; Balancing blocked by voltage/cell availability
                (stop-all-balancing)
                (setq bal-request false)
                (if any-cells
                    (print "BAL: stopped (conditions not met)")
                    (print "BAL: stopped (no cells available)")
                )
            })
        })
    })

    ; If stop was requested mid-cycle, send stop immediately
    (if (and (not bal-request) (= (ix bal-state 0) 1)) (stop-all-balancing))
}))

; ============================================================================
; Main Loop
; ============================================================================

; Initialize local BQ76952 (single chip on master) - retry until ready
(def bq-ic1 (bms-get-param 'cells_ic1))
(if (> bq-ic1 0) {
    (print "BQ init: waiting for BQ76952...")
    (loopwhile (not (bms-init bq-ic1)) (sleep 1.0))
    (print (str-merge "BQ init OK: " (str-from-n bq-ic1 "%d") " cells"))
})

; Apply initial FET state matching default chg-allowed = false (charge OFF)
(bms-set-chg 0)

; Register BMS command events used by VESC Tool
(event-register-handler (spawn event-handler))
(event-enable 'event-bms-force-bal)
(event-enable 'event-bms-chg-allow)
(event-enable 'event-bms-reset-cnt)
(event-enable 'event-bms-zero-ofs)

; Spawn balance thread
(print "Spawning balance thread...")
(spawn 200 balance-thd)
(print "Balance thread spawned")

; 2 beeps = initialization complete
(pwm-set-duty 0.5 0) (sleep 0.1)
(pwm-set-duty 0.0 0) (sleep 0.1)
(pwm-set-duty 0.5 0) (sleep 0.1)
(pwm-set-duty 0.0 0)

; Track previous balance mask per slave for change detection (8 slaves)
(def prev-bal-mask (list 0 0 0 0 0 0 0 0))

; Main loop - fixed 20 Hz; heavy tasks gated to 10 Hz
(print "Entering main loop...")
(def loop-cnt 0)
(loopwhile t {
    ; Drain CAN buffer every 50ms (20 Hz)
    (master-can-read-all)

    ; Check for balance mask changes from slaves (runs every iteration for fast detection)
    (var sid 1)
    (loopwhile (<= sid 8) {
        (if (master-slave-active? sid) {
            (var status (master-get-slave-status sid))
            (if status {
                (var cur-mask (car status))
                (var prev-mask (ix prev-bal-mask (- sid 1)))
                (if (not-eq cur-mask prev-mask)
                    (setix prev-bal-mask (- sid 1) cur-mask)
                )
            })
        })
        (setq sid (+ sid 1))
    })

    ; Run 10 Hz tasks every 2nd iteration (100ms)
    (if (= (mod loop-cnt 2) 0) {
        ; Balance keepalive to slaves at 1 Hz while balancing is active.
        ; This resets slave 10s watchdog with lower CAN bus load.
        (if (and bal-request
                 (= (ix bal-state 0) 1)
                 (or (= (mod loop-cnt 20) 0) (= (ix bal-keepalive-kick 0) 1))) {
            (var tx-sid 1)
            (loopwhile (<= tx-sid 8) {
                (if (master-slave-active? tx-sid) {
                    (master-send-balance
                        tx-sid
                        (ix slave-bal-mask-ic1 (- tx-sid 1))
                        (ix slave-bal-mask-ic2 (- tx-sid 1))
                        0)
                })
                (setq tx-sid (+ tx-sid 1))
            })
            (setix bal-keepalive-kick 0 0)
        })

        ; Update VESC BMS display with local BQ + slave cell voltages
        (master-update-vesc-bms)

        ; Integrate Ah/Wh from filtered current (10 Hz -> dt = 0.1 s).
        ; Gate by min_current_ah_wh_cnt so quiescent-noise current does not drift counters.
        ; Sign convention: positive current accumulates positively (matches bms32).
        (var i-now (master-get-current))
        (var i-min-cnt (bms-get-param 'min_current_ah_wh_cnt))
        (if (> (abs i-now) i-min-cnt) {
            (var v-tot (get-bms-val 'bms-v-tot))
            (setq ah-cnt (+ ah-cnt (/ (* i-now 0.1) 3600.0)))
            (setq wh-cnt (+ wh-cnt (/ (* i-now v-tot 0.1) 3600.0)))
        })
        (set-bms-val 'bms-ah-cnt ah-cnt)
        (set-bms-val 'bms-wh-cnt wh-cnt)
        (set-bms-val 'bms-chg-allowed (if chg-allowed 1 0))

        ; Broadcast BMS data to ESC via VESC CAN protocol (for phone app)
        (send-bms-can)

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

    ; 50ms loop (20 Hz CAN drain, 10 Hz heavy tasks)
    (sleep 0.05)
})

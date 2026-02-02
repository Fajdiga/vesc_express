; JFBMS Master - Counter-based timing (spawn/sleep doesn't work in threads)

(print "=== JFBMS Master v4 ===")

; ============ CONFIGURATION ============
; All times in iterations (1 iteration = 100ms with sleep 0.1)
(def loop-sleep 0.1)      ; 100ms per iteration
(def bal-interval 10)     ; 10 iterations = 1 second
(def stats-interval 20)   ; 20 iterations = 2 seconds

; Master cells configuration
(def master-num-cells 32) ; Number of master's own cells (set to 0 to disable)
; =======================================

; Mutable state: state[0] = loop counter
(def state (list 0))

; Track which slaves were previously active (0 = not seen, 1 = seen)
; Index 0-7 for slave IDs 1-8
(def prev-active (list 0 0 0 0 0 0 0 0))

; Pending beep: counter value when beep should fire (0 = no pending beep)
; Index 0-7 for slave IDs 1-8
(def beep-at (list 0 0 0 0 0 0 0 0))

; ============================================================================
; Master Cell Simulation (generates test voltages for VESC Tool display)
; ============================================================================
; Generate a list of simulated cell voltages (3.65V - 3.85V range)
(defun generate-master-cells (num-cells seed) {
    (var cells '())
    (var v 3.75)
    (looprange i 0 num-cells {
        ; Simple variation based on cell index
        (setq v (+ 3.65 (* 0.02 (mod (+ i seed) 10))))
        (setq cells (append cells (list v)))
    })
    cells
})

; Print slave data
(defun print-slaves ()
    (progn
        (var slaves (master-get-active-slaves))
        (if (eq slaves nil)
            (print "No slaves")
            (loopforeach s slaves
                (progn
                    (print (str-merge "== Slave " (str-from-n s "%d") " =="))
                    (var cells (master-get-all-cells s))
                    (if cells
                        (progn
                            (var line "")
                            (var cnt 0)
                            (looprange i 0 32
                                (progn
                                    (var v (ix cells i))
                                    (if (> v 0.1)
                                        (progn
                                            (setq line (str-merge line "C" (str-from-n (+ i 1) "%d") ":" (str-from-n v "%.2f") " "))
                                            (setq cnt (+ cnt 1))
                                            (if (= (mod cnt 8) 0)
                                                (progn (print line) (setq line "")))))))
                            (if (> (str-len line) 0) (print line))))
                    (var temps (master-get-all-temps s))
                    (if (and temps (>= (length temps) 4)) {
                        (var t-bq1 (ix temps 0))
                        (var t-ext1 (ix temps 1))
                        (var t-ext2 (ix temps 2))
                        (var t-bq2 (ix temps 3))
                        (var line "Temps: ")
                        (if (and t-bq1 (> t-bq1 -200.0) (< t-bq1 200.0))
                            (setq line (str-merge line "BQ1:" (str-from-n t-bq1 "%.1f") "C ")))
                        (if (and t-ext1 (> t-ext1 -200.0) (< t-ext1 200.0))
                            (setq line (str-merge line "Ext1:" (str-from-n t-ext1 "%.1f") "C ")))
                        (if (and t-ext2 (> t-ext2 -200.0) (< t-ext2 200.0))
                            (setq line (str-merge line "Ext2:" (str-from-n t-ext2 "%.1f") "C ")))
                        (if (and t-bq2 (> t-bq2 -200.0) (< t-bq2 200.0))
                            (setq line (str-merge line "BQ2:" (str-from-n t-bq2 "%.1f") "C ")))
                        (if (> (str-len line) 8) (print line))
                    })
                    (var st (master-get-status s))
                    (if (and st (>= (length st) 2)) {
                        (var bal (ix st 0))
                        (var flt (ix st 1))
                        (print (str-merge "Bal:0x" (str-from-n bal "%08X") " F:" (str-from-n flt "%d")))
                    }))))))

; Init
(print "Init...")
(master-reset-data)
(master-clear-cells)
(print (str-merge "Config: loop=" (str-from-n loop-sleep "%.2f") "s, bal=" (str-from-n bal-interval "%d") " iter, stats=" (str-from-n stats-interval "%d") " iter"))
(print (str-merge "Master cells: " (str-from-n master-num-cells "%d")))

; Main loop
(loopwhile t {
    ; Process CAN messages from slaves
    (master-can-read-all)

    ; Check for newly connected slaves - schedule beep with 1s delay
    (var slaves (master-get-active-slaves))
    (if slaves
        (loopforeach s slaves {
            (var idx (- s 1))
            (if (and (>= idx 0) (< idx 8) (= (ix prev-active idx) 0)) {
                (print (str-merge "New slave " (str-from-n s "%d") " connected!"))
                (setix beep-at idx (+ (ix state 0) 10))
                (setix prev-active idx 1)
            })
        }))

    ; Fire pending beeps (1s after connection detected)
    (looprange i 0 8 {
        (var ba (ix beep-at i))
        (if (and (> ba 0) (>= (ix state 0) ba)) {
            (do-beeps (+ i 1) 2 100)
            (setix beep-at i 0)
        })
    })

    ; Set master's own cells (if configured)
    (if (> master-num-cells 0) {
        (var cnt (ix state 0))
        (var master-cells (generate-master-cells master-num-cells cnt))
        (master-set-cells master-cells)
    })

    ; Update VESC BMS display (shows master cells + slave cells)
    (master-update-vesc-bms)
    (master-check-timeouts 2000)

    ; Reset prev-active for slaves that went inactive (so reconnect triggers beep)
    (looprange i 0 8 {
        (if (and (= (ix prev-active i) 1) (not (master-slave-active? (+ i 1)))) {
            (setix prev-active i 0)
            (setix beep-at i 0)
        })
    })

    ; Increment counter
    (setix state 0 (+ (ix state 0) 1))
    (var cnt (ix state 0))

    ; Send balance command at interval
    ; Mask: bit 1 = cell 2 -> mask 2
    (if (= (mod cnt bal-interval) 0)
        (master-send-balance 1 2))

    ; Print stats at interval
    (if (= (mod cnt stats-interval) 0)
        (progn
            (print "")
            (print (str-merge "Avail:" (str-from-n (master-can-available) "%d")
                              " Overflow:" (str-from-n (master-can-overflow) "%d")))
            (print-slaves)))

    (sleep loop-sleep)
})

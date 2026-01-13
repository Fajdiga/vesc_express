; JFBMS Master - Counter-based timing (spawn/sleep doesn't work in threads)

(print "=== JFBMS Master v3 ===")

; ============ CONFIGURATION ============
; All times in iterations (1 iteration = 100ms with sleep 0.1)
(def loop-sleep 0.1)      ; 100ms per iteration
(def bal-interval 10)     ; 10 iterations = 1 second
(def stats-interval 20)   ; 20 iterations = 2 seconds
; =======================================

; Mutable state: state[0] = loop counter
(def state (list 0))

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
(print (str-merge "Config: loop=" (str-from-n loop-sleep "%.2f") "s, bal=" (str-from-n bal-interval "%d") " iter, stats=" (str-from-n stats-interval "%d") " iter"))

; Main loop
(loopwhile t {
    ; Process CAN messages
    (master-can-read-all)
    (master-update-vesc-bms)
    (master-check-timeouts 2000)

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

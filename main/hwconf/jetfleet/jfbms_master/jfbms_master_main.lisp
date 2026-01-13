; JFBMS Master - Direct CAN buffer (bypasses broken event system)

(print "=== JFBMS Master v2 - DEBUG ===")

(def last-print 0)

; Debug: Track message types received per slave
(def debug-msg-counts (list 0 0 0 0 0 0 0 0 0 0))  ; 10 message types (0-9)

; Debug: Parse and print CAN message with values
(defun debug-can-msg (id data) {
    (var slave-id (bitwise-and id 0x7F))
    (var msg-type (shr (bitwise-and id 0x780) 7))

    (if (<= msg-type 7) {
        ; Cell voltage message (4 cells, uint16 mV each)
        (var base-cell (* msg-type 4))
        (var v0 (+ (bufget-u8 data 0) (shl (bufget-u8 data 1) 8)))
        (var v1 (+ (bufget-u8 data 2) (shl (bufget-u8 data 3) 8)))
        (var v2 (+ (bufget-u8 data 4) (shl (bufget-u8 data 5) 8)))
        (var v3 (+ (bufget-u8 data 6) (shl (bufget-u8 data 7) 8)))
        (print (str-merge "CELLS[" (to-str base-cell) "-" (to-str (+ base-cell 3)) "]: "
                          (to-str v0) "mV " (to-str v1) "mV " (to-str v2) "mV " (to-str v3) "mV"))
    })

    (if (= msg-type 8) {
        ; Temperature message: BQ1-int, Ext1(TS1), Ext2(TS3), BQ2-int
        ; Raw values are int16 in 0.1°C, 0x7FFF = invalid
        (var t0 (+ (bufget-u8 data 0) (shl (bufget-u8 data 1) 8)))
        (var t1 (+ (bufget-u8 data 2) (shl (bufget-u8 data 3) 8)))
        (var t2 (+ (bufget-u8 data 4) (shl (bufget-u8 data 5) 8)))
        (var t3 (+ (bufget-u8 data 6) (shl (bufget-u8 data 7) 8)))
        (var line "TEMPS: ")
        ; Show only valid temps (not 0x7FFF = 32767)
        (if (not (= t0 32767)) {
            (if (> t0 32767) (setq t0 (- t0 65536)))
            (setq line (str-merge line "BQ1:" (str-from-n (/ t0 10.0) "%.1f") "C "))
        })
        (if (not (= t1 32767)) {
            (if (> t1 32767) (setq t1 (- t1 65536)))
            (setq line (str-merge line "Ext1:" (str-from-n (/ t1 10.0) "%.1f") "C "))
        })
        (if (not (= t2 32767)) {
            (if (> t2 32767) (setq t2 (- t2 65536)))
            (setq line (str-merge line "Ext2:" (str-from-n (/ t2 10.0) "%.1f") "C "))
        })
        (if (not (= t3 32767)) {
            (if (> t3 32767) (setq t3 (- t3 65536)))
            (setq line (str-merge line "BQ2:" (str-from-n (/ t3 10.0) "%.1f") "C "))
        })
        (print line)
    })

    (if (= msg-type 9) {
        ; Status message (4 bytes bal mask + 1 byte faults)
        (var bal (+ (bufget-u8 data 0)
                    (shl (bufget-u8 data 1) 8)
                    (shl (bufget-u8 data 2) 16)
                    (shl (bufget-u8 data 3) 24)))
        (var faults (bufget-u8 data 4))
        (print (str-merge "STATUS: BalMask=0x" (str-from-n bal "%08X") " Faults=" (to-str faults)))
    })
})

; Print slave data
(defun print-slaves ()
    (progn
        (var slaves (master-get-active-slaves))
        (if (eq slaves nil)
            (print "No slaves")
            (loopforeach s slaves
                (progn
                    (print (str-merge "== Slave " (to-str s) " =="))
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
                                            (setq line (str-merge line "C" (to-str (+ i 1)) ":" (str-from-n v "%.2f") " "))
                                            (setq cnt (+ cnt 1))
                                            (if (= (mod cnt 8) 0)
                                                (progn (print line) (setq line "")))))))
                            (if (> (str-len line) 0) (print line))))
                    (var temps (master-get-all-temps s))
                    (if (and temps (>= (length temps) 4)) {
                        ; Temperature mapping:
                        ; temps[0] = BQ1 internal die temp
                        ; temps[1] = TS1 external NTC
                        ; temps[2] = TS3 external NTC
                        ; temps[3] = BQ2 internal die temp
                        ; Invalid = 3276.7°C (0x7FFF raw) or < -200
                        (var t-bq1 (ix temps 0))
                        (var t-ext1 (ix temps 1))
                        (var t-ext2 (ix temps 2))
                        (var t-bq2 (ix temps 3))
                        (var line "Temps: ")
                        ; Only show valid temperatures (> -200 and < 200)
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
                        (print (str-merge "Bal:" (to-str bal) " F:" (to-str flt)))
                    }))))))

; Init
(print "Init...")
(master-reset-data)

; Main loop - DEBUG MODE
(print "Running in DEBUG mode - showing first 20 raw CAN messages...")
(def debug-count 0)
(def debug-phase 1)  ; 1 = collecting raw, 2 = normal operation

(loopwhile t {
    (if (= debug-phase 1) {
        ; Phase 1: Read and print first 20 raw messages
        (var msg (master-can-read))
        (if msg {
            (var id (car msg))
            (var data (cdr msg))
            (debug-can-msg id data)
            (setq debug-count (+ debug-count 1))
            (if (>= debug-count 20) {
                (print "")
                (print "=== Switching to normal operation ===")
                (setq debug-phase 2)
            })
        } {
            (sleep 0.01)
        })
    } {
        ; Phase 2: Normal operation
        (master-can-read-all)
        (master-update-vesc-bms)
        (master-check-timeouts 2000)

        ; Print summary every 2 seconds
        (var now (systime))
        (if (> (- now last-print) 2.0) {
            (print "")
            (print (str-merge "Avail:" (to-str (master-can-available))
                              " Overflow:" (to-str (master-can-overflow))))
            (print-slaves)
            (setq last-print now)
        })

        (sleep 0.05)
    })
})

; JFBMS Master - Direct CAN buffer (bypasses broken event system)

(print "=== JFBMS Master v2 ===")

(def last-print 0)

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
                    (if temps
                        (print (str-merge "T: " (str-from-n (ix temps 0) "%.1f") "/"
                                              (str-from-n (ix temps 1) "%.1f") "/"
                                              (str-from-n (ix temps 2) "%.1f") "/"
                                              (str-from-n (ix temps 3) "%.1f"))))
                    (var st (master-get-status s))
                    (if st (print (str-merge "Bal:" (str-from-n (ix st 0) "%08X") " F:" (to-str (ix st 1))))))))))

; Init
(print "Init...")
(master-reset-data)

; Main loop - fast processing, slow printing
(print "Running...")
(loopwhile t
    (progn
        ; Read and parse all buffered CAN messages (fast - every 50ms)
        (master-can-read-all)

        ; Update VESC BMS values for VESC Tool display
        (master-update-vesc-bms)

        ; Check timeouts
        (master-check-timeouts 2000)

        ; Print to terminal only every 1 second
        (var now (systime))
        (if (> (- now last-print) 1.0)
            (progn
                (print "")
                (print (str-merge "Avail:" (to-str (master-can-available))
                                  " Overflow:" (to-str (master-can-overflow))))
                (print-slaves)
                (setq last-print now)))

        ; Fast loop - 50ms cycle for real-time VESC Tool updates
        (sleep 0.05)))

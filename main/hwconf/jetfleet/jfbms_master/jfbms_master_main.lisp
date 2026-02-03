; JFBMS Master
; Receives cell voltages from slaves via CAN
; Displays in VESC Tool

(print "=== JFBMS Master ===")

; Config
(def slave-timeout 2000)

; Reset slave data
(master-reset-slaves)

(loopwhile t {
    ; Read and parse all buffered CAN messages from slaves
    (master-can-read-all)

    ; Update VESC BMS display with slave cell voltages
    (master-update-vesc-bms)

    ; Check for timed-out slaves
    (master-check-timeouts slave-timeout)

    ; 100ms loop (10 Hz)
    (sleep 0.1)
})

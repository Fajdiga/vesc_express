; JFBMS Master
; Receives cell voltages from slaves via CAN
; Displays in VESC Tool

(print "=== JFBMS Master ===")

; Config
(def slave-timeout 2000)

; Reset slave data
(master-reset-slaves)

; State: (counter prev-active-1 prev-active-2 ... prev-active-8)
; Using mutable list for loop persistence
(def state (list 0 0 0 0 0 0 0 0 0))

(loopwhile t {
    ; Read and parse all buffered CAN messages from slaves
    (master-can-read-all)

    ; Update VESC BMS display with slave cell voltages
    (master-update-vesc-bms)

    ; Check for timed-out slaves
    (master-check-timeouts slave-timeout)

    ; Increment counter (index 0)
    (setix state 0 (+ (ix state 0) 1))

    ; Every 50 iterations (5 seconds at 100ms loop)
    (if (>= (ix state 0) 50) {
        (setix state 0 0)

        ; Check each slave 1-8
        (var id 1)
        (loopwhile (<= id 8) {
            (var active (if (master-slave-active? id) 1 0))
            (var prev (ix state id))

            (if (and (= active 1) (not-eq prev 1))
                (print (str-merge "Slave " (str-from-n id "%d") " is connected"))
            )
            (if (and (= active 0) (= prev 1))
                (print (str-merge "Slave " (str-from-n id "%d") " disconnected"))
            )

            ; Update previous state
            (setix state id active)
            (setq id (+ id 1))
        })
    })

    ; 100ms loop (10 Hz)
    (sleep 0.1)
})

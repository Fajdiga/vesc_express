; JFBMS Master
; Receives cell voltages from slaves via CAN
; Displays in VESC Tool

(print "=== JFBMS Master ===")

; Config
(def slave-timeout 500)

; Reset slave data
(master-reset-slaves)

; Previous active state for each slave (1-8)
; Using mutable list for loop persistence
(def prev-active (list 0 0 0 0 0 0 0 0))

(loopwhile t {
    ; Read and parse all buffered CAN messages from slaves
    (master-can-read-all)

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
                    (master-send-balance alert-id 0 0x04)
                )
                (setq alert-id (+ alert-id 1))
            })
        })

        (setix prev-active (- id 1) active)
        (setq id (+ id 1))
    })

    ; 100ms loop (10 Hz)
    (sleep 0.1)
})

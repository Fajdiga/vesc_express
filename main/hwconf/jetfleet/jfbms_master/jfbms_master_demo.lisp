; JFBMS Master CAN RX Test
; Minimal script to test CAN reception

(print "=== CAN RX Test ===")
(print "Waiting for CAN messages...")
(print "If you see 'RX:' lines below, CAN is working.")
(print "")

; Simple receive loop - use can-recv-sid instead of event system
(loopwhile t {
    (var msg (can-recv-sid 1000))  ; 1 second timeout
    (if msg {
        (var id (ix msg 0))
        (var data (ix msg 1))
        (print (str-merge "RX: ID=" (str-from-n id "%d") " len=" (str-from-n (buflen data) "%d")))
    })
})

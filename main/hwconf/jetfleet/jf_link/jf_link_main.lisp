; JF Link aggregation and balancing start in the native firmware at boot.
; Keep this optional app as a diagnostic; a second Lisp controller would
; consume the same CAN queue and compete with the native balance commands.
(print "JF Link: native CAN aggregation and balancing start automatically.")

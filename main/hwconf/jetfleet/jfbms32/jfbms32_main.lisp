;;;;;;;;; User Settings ;;;;;;;;;

; TODO: Move into config?

; Wait this long for charger to start charging
(def charger-max-delay 10.0)
(def current-scale 0.9675) ; Shared V1/V2 meter calibration: 1.19A actual / 1.23A reported
(def sleep-unblock-en true) ; Enable automatic sleep unblocking
(def app-wdt-timeout 10) ; Seconds. Fed only after a successful control scan; same as JFBMS Master.
(def user-beeps-en true) ; Informational beeps. Fault and shutdown alarms always sound.
(def beep-duty-alarm 0.5) ; Loud 4 kHz drive for faults that need attention. Same as JFBMS Master.
(def beep-duty-quiet 0.03) ; Quiet drive for informational beeps. Same as JFBMS Master.
(def beep-freq-high 4000) ; OK / info tone, Hz
(def beep-freq-low 2700) ; attention tone, Hz
(def buzzer-pin 3)
(def bq-wake-alarm-period 3) ; Failed init attempts between BQ wake alarms. Set to 1 for every attempt.

;;;;;;;;; End User Settings ;;;;;;;;;

; State
(def trigger-bal-after-charge false)
(def charge-session-valid false)
(def bal-ok false)
(def is-balancing false)
(def is-charging false)
(def charge-ok false)
(def charge-complete false)
(def charge-no-current false)
(def charge-hw-refused false)
(def charge-current 0.0)
(def charge-current-filt 0.0)
(def balance-request-ts nil)
(def shutdown-in-progress false)
(def charge-complete-msg false)
(def charger-detected-prev false)
(def charge-block-beeped false)
(def charge-enable-beeped false)
(def c-min 0.0)
(def c-max 0.0)
(def t-min 0.0)
(def t-max 0.0)
(def t-mos 0.0)
(def t-ic 0.0)
(def temps-valid false)
(def charge-wakeup false)
(def init-done false)

(def chg-status "")
(def bq-status "")
(def bq-status-latched "")
(def bal-status "")
(def bq-safety-flags 0)
(def bq-hard-fault-mask 0)
(def bq-reset-requested false)
(def bq-reset-ts nil)
(def bq-reset-message "")
(def bal-off-failed false)
(def fail-close-failed false)

(def shutdown-reason-unknown 0)
(def shutdown-reason-timer 1)
(def shutdown-reason-low-soc-timer 2)
(def shutdown-reason-app 4)

(def last-bq-init-attempts 0)
(def last-bq-detect-08 0)
(def last-bq-detect-10 0)
(def last-bq-wake-stage 0)
(def last-bq-wake-time-s 0)

(def rtc-val '(
    (wakeup-cnt . 0)
    (sleep-enter-time-s . 0)
    (sleep-total-time-s . 0)
    (c-min . 3.5)
    (c-max . 3.5)
    (v-tot . 50.0)
    (soc . 0.5)
    (charge-fault . false)
    (bq-hard-fault-mask . 0)
    (updated . false)
    (last-bq-init-attempts . 0)
    (last-bq-detect-08 . 0)
    (last-bq-detect-10 . 0)
    (last-bq-wake-stage . 0)
    (last-bq-wake-time-s . 0)))

(def vtot 0.0)
(def vout 0.0)
(def vt-vchg 0.0)
(def iout 0.0)
(def soc -1.0)
(def i-zero-time 0.0)
(def chg-allowed true)
(def current-zero-offset 0.0)
(def com-force-on false)
(def com-mutex (mutex-create))
(def buz-mutex (mutex-create))
(def did-crash false)
(def crash-cnt 0)

@const-start

;;; Hack until problem is found ;;;
; Pre-load all functions that are loaded with the dynamic loader. This will
; make them end up in the image and there is no need to load them dynamically.

str-merge
foldl
foldr
zipwith
filter
str-cmp-asc
str-cmp-dsc
second
third
abs

defun
defunret
defmacro
loopfor
loopwhile
looprange
loopforeach
loopwhile-thd

;;; Hack End ;;;

; Invert the reporting sign and apply the shared meter calibration.
(defun bms-current-raw () (* (bms-get-current) -1.0 current-scale))
(defun bms-current () (- (bms-current-raw) current-zero-offset))

(defun beep-duty (times dt duty freq) {
    (mutex-lock buz-mutex)
    (pwm-start freq 0.0 0 buzzer-pin)
    (loopwhile (> times 0) {
        (pwm-set-duty duty 0)
        (sleep dt)
        (pwm-set-duty 0.0 0)
        (sleep dt)
        (setq times (- times 1))
    })
    (mutex-unlock buz-mutex)
})

; Alarm beeps always sound; user beeps are informational and can be disabled.
(defun beep (times dt) (beep-duty times dt beep-duty-alarm beep-freq-high))

(defun user-beep (times dt) (if user-beeps-en (beep-duty times dt beep-duty-quiet beep-freq-high)))

(defun user-beep-low (times dt) (if user-beeps-en (beep-duty times dt beep-duty-quiet beep-freq-low)))

; Play (freq-hz seconds) notes back to back; one buzzer user at a time.
(defun tone-seq (duty notes) {
    (mutex-lock buz-mutex)
    (loopforeach n notes {
        (pwm-start (first n) duty 0 buzzer-pin)
        (sleep (second n))
        (pwm-set-duty 0.0 0)
        (sleep 0.06)
    })
    (mutex-unlock buz-mutex)
})

(defun user-tones (notes) (if user-beeps-en (tone-seq beep-duty-quiet notes)))

; Rising = charging started, falling = charging blocked.
(defun charge-start-beep () (user-tones '((2700 0.2) (3600 0.2) (4800 0.3))))

(defun charge-block-beep () (user-tones '((4800 0.2) (3600 0.2) (2700 0.6))))

; Beep table, identical on JFBMS32 and JFBMS Master. Minimum beep 0.2 s.
; High tone (4 kHz) = OK/info, low tone (2.7 kHz) = needs attention.
; One result per plug-in.
; Quiet (user-beep / user-beep-low, off when user-beeps-en is false):
;   2 short high       init, settings applied, manual zero captured
;   1 long low         current zero calibration failed (Master only)
;   rising 3 notes     charging started (2.7 -> 3.6 -> 4.8 kHz)
;   3 short high       charge complete
;   4 short high       sleep unblocked
;   falling 3 notes    charge blocked (4.8 -> 3.6 -> 2.7 kHz, still blocked 3 s after plug-in)
;   2 long low         charge fault latched
; Loud (beep, always on):
;   15 short shutdown warning             5 short  shutdown failed (repeats)
;   3 long   sleep failed (max every 30 s)
;   1 long + N short  monitor not responding (JFBMS32: BQ wake stage N,
;                     Master: slave N lost)
;   5 x 0.4 s         ADC stall reset (Master only)
; Slave buzzer codes sent by the Master (patterns in jfbms_slave handle-beep):
;   0x01 to a slave that comes online, 0x03 charge complete, 0x10-0x13 charge
;   block/fault (over-temp, cell high, cell low, over-current); these follow
;   user-beeps-en. 0x04 on shutdown and when a slave is lost (always).
;   The slave plays 0x14 itself when its BQ fails to start.

(def sleep-fail-alarm-ts nil)

; Loud, rate limited: a failed sleep retries every control scan.
(defun sleep-fail-alarm () {
    (if (or (not sleep-fail-alarm-ts) (> (secs-since sleep-fail-alarm-ts) 30.0)) {
        (setq sleep-fail-alarm-ts (systime))
        (spawn (fn () (beep 3 0.6)))
    })
})

(def rtc-val-magic 124)

; If in deepsleep, this will return 4
; (bms-direct-cmd 1 0x00)

; Exit deepsleep
; (bms-subcmd-cmdonly 1 0x000e)

(defun bq-wake-report-due (attempts) (or
    (= attempts 1)
    (and (> bq-wake-alarm-period 0) (= (mod attempts bq-wake-alarm-period) 0))))

(defun check-bq-awake (ic read-failed-stage asleep-stage attempts) {
    (var status (bq-status-read ic))
    (if (= status -1) {
        (record-bq-wake-debug read-failed-stage attempts)
        (bq-wake-debug-beep last-bq-wake-stage)
        (exit-error 0)
    })
    (if (= status 4) {
        (var tries 0)
        (loopwhile (and (= status 4) (< tries 20)) {
            (bq-exit-deepsleep-all)
            (if (not shutdown-in-progress) (wdt-reset))
            (sleep 0.05)
            (setq status (bq-status-read ic))
            (setq tries (+ tries 1))
        })
        (if (or (= status -1) (= status 4)) {
            (record-bq-wake-debug (if (= status -1) read-failed-stage asleep-stage) attempts)
            (bq-wake-debug-beep last-bq-wake-stage)
            (exit-error 0)
        })
    })
})

(defun init-hw () {
    (var attempts 0)
    ; Restore the native gate interlock before init can allow BQ outputs.
    (bms-protection-lock bq-hard-fault-mask)
    (loopwhile (not (bms-init (bms-get-param 'cells_ic1) (bms-get-param 'cells_ic2))) {
        (setq attempts (+ attempts 1))
        (record-bq-wake-debug 0 attempts)

        ; Fault alarm: 6 fast marker beeps, then a stage code.
        ; Rate-limit it so a missing BQ warns loudly without wasting
        ; more pack energy on a continuous buzzer.
        (if (bq-wake-report-due attempts) (bq-wake-debug-beep last-bq-wake-stage))
        (bq-exit-deepsleep-all)
        (if (not shutdown-in-progress) (wdt-reset))
        (sleep (if (< attempts 5) 1.0 3.0))
    })
    (check-bq-awake 1 5 6 attempts)
    (if (> (bms-get-param 'cells_ic2) 0) (check-bq-awake 2 7 8 attempts))
})

(defun save-rtc-val () {
    (var tmp (flatten rtc-val))
    (bufcpy (rtc-data) 0 tmp 0 (buflen tmp))
    (bufset-u8 (rtc-data) 900 rtc-val-magic)
})

(defun rtc-get (name default) (let ((v (assoc rtc-val name))) (if v v default)))

(defun bq-probe-addr (addr) (match (trap (eval `(i2c-detect-addr ,addr))) ((exit-ok (? a)) (if a 1 0)) (_ 0)))

(defun bq-probe-stage () (cond
    ((and (= last-bq-detect-08 0) (= last-bq-detect-10 0)) 1) ; no BQ address responds
    ((and (= last-bq-detect-08 1) (= last-bq-detect-10 0)) 2) ; only default address responds
    ((and (= last-bq-detect-08 0) (= last-bq-detect-10 1)) 3) ; only BQ1 target address responds
    (true 4)                                                   ; both addresses respond
))

(defun bq-wake-stage-name (stage) (cond
    ((= stage 1) "no-address")
    ((= stage 2) "only-0x08")
    ((= stage 3) "only-0x10")
    ((= stage 4) "both-addresses")
    ((= stage 5) "bq1-status-read-failed")
    ((= stage 6) "bq1-stuck-deepsleep")
    ((= stage 7) "bq2-status-read-failed")
    ((= stage 8) "bq2-stuck-deepsleep")
    (true "none")))

(defun bq-wake-debug-beep (stage) {
    ; BQ not responding: one long marker, pause, then stage count.
    (beep 1 0.6)
    (sleep 0.4)
    (beep stage 0.2)
})

(defun bq-exit-deepsleep-all () {
    ; Try both expected BQ logical addresses. Failures are diagnostic only:
    ; the recovery loop below will re-run bms-init until communication is sane.
    (trap (bms-subcmd-cmdonly 1 0x000e))
    (trap (bms-subcmd-cmdonly 1 0x000e))
    (trap (bms-subcmd-cmdonly 2 0x000e))
    (trap (bms-subcmd-cmdonly 2 0x000e))
})

(defun bq-status-read (ic) (match (trap (eval `(bms-direct-cmd ,ic 0x00))) ((exit-ok (? a)) a) (_ -1)))

(defun record-bq-wake-debug (stage attempts) {
    (setq last-bq-init-attempts attempts)
    (setq last-bq-detect-08 (bq-probe-addr 0x08))
    (setq last-bq-detect-10 (bq-probe-addr 0x10))
    (setq last-bq-wake-stage (if (= stage 0) (bq-probe-stage) stage))
    (setq last-bq-wake-time-s (get-time-of-day-s))
    (setassoc rtc-val 'last-bq-init-attempts last-bq-init-attempts)
    (setassoc rtc-val 'last-bq-detect-08 last-bq-detect-08)
    (setassoc rtc-val 'last-bq-detect-10 last-bq-detect-10)
    (setassoc rtc-val 'last-bq-wake-stage last-bq-wake-stage)
    (setassoc rtc-val 'last-bq-wake-time-s last-bq-wake-time-s)
    (save-rtc-val)
    (if (bq-wake-report-due attempts) {
        (print "BQ wake:" (bq-wake-stage-name last-bq-wake-stage)
            "attempts" attempts
            "0x08" last-bq-detect-08
            "0x10" last-bq-detect-10)
    })
})

(defun sync-bq-wake-debug-globals () {
    (setq last-bq-init-attempts (rtc-number 'last-bq-init-attempts 0))
    (setq last-bq-detect-08 (rtc-number 'last-bq-detect-08 0))
    (setq last-bq-detect-10 (rtc-number 'last-bq-detect-10 0))
    (setq last-bq-wake-stage (rtc-number 'last-bq-wake-stage 0))
    (setq last-bq-wake-time-s (rtc-number 'last-bq-wake-time-s 0))
})

(defun shutdown-reason-name (reason) (cond
    ((= reason shutdown-reason-timer) "timer")
    ((= reason shutdown-reason-low-soc-timer) "low-soc-timer")
    ((= reason shutdown-reason-app) "app")
    (true "unknown")))

(defun shutdown-reason-beep (reason) {
    ; Final local warning before power-off; same 15 x 0.2 s alarm as the master.
    (beep 15 0.2)
})

(defun print-shutdown-reason (reason) { (print "Shutdown reason:" (shutdown-reason-name reason)) })

(defun prepare-external-wakeup () {
    ; JFBMS32 wakes from external requests on IO2.
    (bms-set-btn-wakeup-state 1)
})

(defun external-wake-active () (= (bms-get-btn) 1))

(defun external-wake-inactive () (= (bms-get-btn) 0))

(defun sleep-duration-s () (* (bms-get-param 'sleep) 3600))

(defun test-chg (samples) {
    ; Harmony16's 0.7 V margin, using the full pack across both BQs.
    (var vbat (apply + (with-com '(bms-get-vcells))))
    (if (not (> vbat 0.0)) (exit-error 0))
    (var threshold (+ vbat 0.7))
    ; Many chargers pulse before starting; sample until one exceeds the threshold.
    (var vchg 0.0)
    (looprange i 0 samples {
        (if (> i 0) (sleep 0.01))
        (setq vchg (bms-get-vchg))
        (if (> vchg threshold) (break))
    })
    (var res (> vchg threshold))
    (if res (setq charge-dis-ts (systime)))
    res
})

(defun truncate (n min max) (if (< n min) min (if (> n max) max n)))

; SOC from battery voltage
(defun calc-soc (v-cell) {
    (var empty (bms-get-param 'vc_empty))
    (var full (bms-get-param 'vc_full))
    (var den (- full empty))
    (if (= den 0.0) 0.0 (truncate (/ (- v-cell empty) den) 0.0 1.0))
})

(defun valid-pack-reading () (and
    init-done
    (> cell-num 0)
    (> c-min 1.0)
    (> c-max 2.0)
    (< c-min 5.0)
    (< c-max 5.0)
    (>= c-max c-min)
    (> vtot (* cell-num 1.5))
    (>= soc 0.0)))

; True when a real communication interface is connected.
(defun is-comm-connected () (or (connected-wifi) (connected-usb) (connected-ble)))

; True when VESC Tool is connected or sleep is intentionally blocked.
(defun is-connected () (or (is-comm-connected) (= (bms-get-param 'block_sleep) 1)))

(defunret can-active () {
    (var devs (can-list-devs))
    (if (eq devs nil) (return false))
    (looprange i 1 7 {
        (var res (can-msg-age (first devs) i))
        (if (and res (< res 0.1)) (return true))
    })
    false
})

(defun can-sum-current () {
    (var devs (can-list-devs))
    (var i-sum 0.0)
    (loopforeach d devs {
        (var res (can-msg-age d 4))
        (if (and res (< res 0.1)) {
            (var cur (canget-current-in d))
            (if (number? cur)
                (setq i-sum (+ i-sum cur)))
        })
    })
    i-sum
})

; Run expression with communication enabled. Use up to 4 attempts in case of
; glithces from transients. Disable communication again when it is not needed
; to save power.
(defun with-com (expr) {
    (mutex-lock com-mutex)
    (var result (trap {
        (bms-set-com 0)
        (var res (looprange i 0 4 {
            (match (trap (eval expr))
                ((exit-ok (? a)) (break a))
                (_ (if (= i 3) (exit-error 0))))
        }))
        (if (and (not (can-active)) (external-wake-inactive)
                (not com-force-on) (not (is-connected)) (not is-charging))
            (bms-set-com 1))
        res
    }))
    (mutex-unlock com-mutex)
    (match result ((exit-ok (? value)) value) (_ (exit-error 0)))
})

(defun com-force (en)
    (if en {
        (setq com-force-on true)
        (bms-set-com 0)
    } {
        (setq com-force-on false)
}))

(defun disable-balancing () {
    (var bal-off-ok false)
    (match (trap (bms-disable-balancing))
        ((exit-ok _) (setq bal-off-ok true))
        (_ {
            (setq bal-off-ok true)
            (looprange i 0 cell-num {
                (match (trap (with-com `(bms-set-bal ,i 0))) ((exit-ok _) nil) (_ (setq bal-off-ok false)))
            })
    }))
    (if bal-off-ok {
        (if bal-off-failed (print "Balancing disabled after retry"))
        (setq bal-off-failed false)
        (setq is-balancing false)
        (setq bal-status "")
    } {
        (if (not bal-off-failed) (print "Failed to disable balancing"))
        (setq bal-off-failed true)
    })
    bal-off-ok
})

(defun fail-close-outputs (clear-bal-trigger) {
    (var close-ok false)
    (match (trap (bms-fail-close-outputs))
        ((exit-ok _) (setq close-ok true))
        (_ {
            (trap (bms-set-chg 0))
            (setq close-ok (disable-balancing))
    }))
    (setq is-charging false)
    (setq charge-ok false)
    (setq charge-session-valid false)
    (if clear-bal-trigger (setq trigger-bal-after-charge false))
    (if close-ok {
        (if fail-close-failed (print "BMS fail-close recovered"))
        (setq fail-close-failed false)
        (setq bal-off-failed false)
        (setq is-balancing false)
        (setq bal-status "")
    } {
        (if (not fail-close-failed) (print "BMS fail-close failed"))
        (setq fail-close-failed true)
    })
    close-ok
})

(defun balance-safe-now () (and
    (= bq-hard-fault-mask 0)
    temps-valid
    (<= (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current))
    (>= c-min (bms-get-param 'vc_balance_min))
    (<= t-max (bms-get-param 't_bal_max_cell))
    (<= t-ic (bms-get-param 't_bal_max_ic))))

(defun temp-valid (value) (and (number? value) (>= value -50.0) (<= value 150.0)))

; The first unmet condition is also the reason shown when a charger is present.
(defun charge-block-reason () (cond
    ((not temps-valid) "TEMP_INVALID")
    ((!= bq-hard-fault-mask 0) "BQ_HARD_FAULT")
    ((bq-current-fault-active) "BQ_CURRENT_FAULT")
    (charge-hw-refused "CHG_HW_REFUSED")
    ((assoc rtc-val 'charge-fault) "FLT_CHG_OC")
    (charge-complete "CHG_COMPLETE")
    (charge-no-current "CHG_NO_CURRENT")
    ((not chg-allowed) "CHG_DISABLED")
    ((>= c-max (bms-get-param (if is-charging 'vc_charge_end 'vc_charge_start))) "CHG_CELL_HIGH")
    ((<= c-min (bms-get-param 'vc_charge_min)) "CHG_CELL_LOW")
    ((>= t-max (bms-get-param 't_charge_max)) "CHG_CELL_HOT")
    ((<= t-min (bms-get-param 't_charge_min)) "CHG_CELL_COLD")
    ((>= t-mos (bms-get-param 't_charge_max_mos)) "CHG_MOS_HOT")
    (true "")))

(defun bq-temp-settings-valid (v1 h1 has-ic2) {
    (var v2 (if has-ic2 (with-com '(bms-read-reg 2 0x92fd 1)) 0x3b)) ; default to valid when no IC2
    (var h2 (if has-ic2 (with-com '(bms-read-reg 2 0x9300 1)) 0x3b)) ; default to valid when no IC2
    (not (or
        (and (!= v1 0x3b) (!= v1 0x7b))
        (!= h1 0x3b)
        (and has-ic2 (and (!= v2 0x3b) (!= v2 0x7b)))
        (and has-ic2 (!= h2 0x3b))))
})

(defun update-temps () {
    ; Exit if either BQ still has invalid temperature settings after one retry.
    (var v1 (bms-read-reg 1 0x92fd 1))
    (var h1 (bms-read-reg 1 0x9300 1))
    (var has-ic2 (> (bms-get-param 'cells_ic2) 0))
    (if (not (bq-temp-settings-valid v1 h1 has-ic2)) {
        (print "Invalid temperature settings, retrying...")
        (sleep 0.01)
        (if (not (bq-temp-settings-valid (bms-read-reg 1 0x92fd 1) (bms-read-reg 1 0x9300 1) has-ic2)) {
            (print "BQs with invalid temperature settings")
            (exit-error 0)
        })
    })
    (var bms-temps (with-com '(bms-get-temps)))
    (var temp-ext-num (truncate (bms-get-param 'temp_num) 0 4))

    ; bms-temps: BQ1 IC, BQ1 TS1/TS3/ALERT/DCHG, BQ1 HDQ, BQ2 IC, BQ2 HDQ
    ; Validate each fitted sensor so a healthy sensor cannot hide a failed one.
    (var valid (and (temp-valid (ix bms-temps 0)) (temp-valid (ix bms-temps 5))))
    (looprange i 0 temp-ext-num { (if (not (temp-valid (ix bms-temps (+ i 1)))) (setq valid false)) })
    (if has-ic2 {
        (if (not (and (temp-valid (ix bms-temps 6)) (temp-valid (ix bms-temps 7)))) (setq valid false))
    })
    (setq temps-valid valid)
    (if (not temps-valid) {
        (set-chg false)
        (setq bal-ok false)
        (setq trigger-bal-after-charge false)
        (disable-balancing)
    })
    (var t-sorted (sort < (map (fn (x) (ix bms-temps (+ x 1))) (range 0 temp-ext-num))))

    ; Keep charging available with zero external cell sensors.
    (if (= (length t-sorted) 0) (setq t-sorted '(24)))
    (setq t-min (ix t-sorted 0))
    (setq t-max (ix t-sorted -1))
    (setq t-mos (if (and has-ic2 (> (ix bms-temps 7) (ix bms-temps 5))) (ix bms-temps 7) (ix bms-temps 5)))
    (setq t-ic  (if (and has-ic2 (> (ix bms-temps 6) (ix bms-temps 0))) (ix bms-temps 6) (ix bms-temps 0)))
    bms-temps
})

(defun bms-shutdown-failed-alarm () {
    (print "BMS hardware shutdown failed")
    ; Two attempts have failed. Alarm instead of rebooting into a shutdown loop.
    (loopwhile t {
        (wdt-reset)
        (beep 5 0.2)
        (sleep 5.0)
    })
})

(defun bms-shutdown-impl (save-counters reason) {
	(setq shutdown-in-progress true)
    (print "BMS shutdown sequence starting")
    (print-shutdown-reason reason)
    (fail-close-outputs true)
    (shutdown-reason-beep reason)
    (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
    (save-rtc-val)
    (if save-counters (save-settings))
    (wdt-reset)
    (looprange attempt 0 2 {
        (match (trap (bms-hw-shutdown)) ((exit-ok _) (break)) (_ nil))
        (sleep 0.1)
    })
    (bms-shutdown-failed-alarm)
})

(defun bms-shutdown () (bms-shutdown-impl true shutdown-reason-unknown))

; Counter settings are loaded before start-fun, so timer shutdown can save
; them again before entering hardware shutdown.
(defun bms-shutdown-timer () (bms-shutdown-impl true shutdown-reason-timer))

(defun bms-shutdown-low-soc-timer () (bms-shutdown-impl true shutdown-reason-low-soc-timer))

(defun bms-shutdown-app () (bms-shutdown-impl true shutdown-reason-app))

; Low SOC is a shutdown decision only after a deep-sleep timer wake. Normal
; wakes are for charger/button/CAN use and must continue through the normal
; SOC and charge-control flow. The charger guard is retained as a safety
; check if a timer wake races with charger detection.
(defun low-soc-timer-wake (wake-source chg-detected) (and
    (= wake-source 2)
    (not chg-detected)
    (valid-pack-reading)
    (< soc 0.05)
    (<= c-min (bms-get-param 'vc_empty))
    (not trigger-bal-after-charge)
    (external-wake-inactive)
    ; block_sleep is intentionally honored here so a fresh, unconfigured
    ; pack cannot shut itself down before setup is complete.
    (not (is-connected))
    (not (can-active))))

(defun shutdown-timer-due () (and
    (> (bms-get-param 'shutdown) 0)
    (>= (rtc-number 'sleep-total-time-s 0) (* (bms-get-param 'shutdown) 86400.0))))

(defun process-sleep-time () {
    (var source (bms-wakeup-source))
    (cond
        ; Woke up on GPIO/RTC IO: this is external use, so reset the
        ; shutdown-days counter.
        ((= source 1) { (setassoc rtc-val 'sleep-total-time-s 0) })
        ; Woke up on timer. Prefer the RTC wall-clock delta, but if it
        ; did not advance across deep sleep, a timer wake still means
        ; one configured sleep interval elapsed.
        ((= source 2) {
            (var entered (rtc-number 'sleep-enter-time-s 0))
            (if (> entered 0) {
                (var slept-time (- (get-time-of-day-s) entered))
                (var total-time (rtc-number 'sleep-total-time-s 0))
                (if (<= slept-time 0.0) (setq slept-time (sleep-duration-s)))
                (if (< total-time 0.0) (setq total-time 0))
                (setassoc rtc-val 'sleep-total-time-s (+ total-time slept-time))
            })
    }))
    (setassoc rtc-val 'sleep-enter-time-s 0)
    (save-rtc-val)
    (if (shutdown-timer-due) (bms-shutdown-timer))
    source
})

(defun start-fun () {
    (setassoc rtc-val 'wakeup-cnt (+ (rtc-number 'wakeup-cnt 0) 1))
    (var do-sleep true)
    (if (external-wake-active) { (setq do-sleep false) })
    (init-hw) ; Battery measurements must be available for charger detection.
    (var chg-detected (test-chg 5))
    (if (or charge-wakeup chg-detected) {
        (setq do-sleep false)
        (if (not (assoc rtc-val 'charge-fault)) { (setq charge-wakeup true) })
    })

    ; Reset charge fault when the charger is not connected at boot
    (if (not chg-detected) {
        (setassoc rtc-val 'charge-fault false)
        (setq charge-complete false)
        (setq charge-complete-msg false)
    })
    (if (is-connected) (setq do-sleep false))
    (if (can-active) (setq do-sleep false))
    (user-beep 2 0.2)
    (var wake-source (process-sleep-time))
    (if (can-active) (setq do-sleep false))
    (setq soc -2.0)
    (var v-cells nil)

    ; It takes a few reads to get valid voltages the first time
    (loopwhile (< soc -1.5) {
        (setq v-cells (with-com '(bms-get-vcells)))
        (var v-sorted (sort < v-cells))
        (setq c-min (ix v-sorted 0))
        (setq c-max (ix v-sorted -1))
        (setq soc (calc-soc c-min))
        (sleep 0.1)
    })
    (setassoc rtc-val 'c-min c-min)
    (setassoc rtc-val 'c-max c-max)
    (setq vtot (apply + v-cells))
    (setassoc rtc-val 'v-tot vtot)
    (setassoc rtc-val 'soc soc)
    (setassoc rtc-val 'updated true)
    (save-rtc-val)
    (update-temps)
    (setq bq-status (update-bq-status))
    (setq init-done true)
    (setq charge-ok (= (str-len (charge-block-reason)) 0))
    (var ichg 0.0)
    (if (and charge-ok charge-wakeup (test-chg 400)) {
        ; Startup may wait for the charger before main-ctrl exists. Arm
        ; the native scan watchdog and current supervisor before opening
        ; the gate, and refresh every safety input during that wait.
        (bms-control-start)
        (var startup-cells (sort < (with-com '(bms-get-vcells))))
        (setq c-min (first startup-cells))
        (setq c-max (ix startup-cells -1))
        (update-temps)
        (setq bq-status (update-bq-status))
        (setq ichg (- (bms-current)))
        (if (> ichg (bms-get-param 'max_charge_current)) (latch-charge-fault))
        (set-chg (= (str-len (charge-block-reason)) 0))
        (looprange i 0 (* charger-max-delay 10.0) {
            (sleep 0.1)
            (if (not shutdown-in-progress) (wdt-reset))
            (setq bq-status (update-bq-status))
            (var startup-cells (sort < (with-com '(bms-get-vcells))))
            (setq c-min (first startup-cells))
            (setq c-max (ix startup-cells -1))
            (update-temps)
            (setq ichg (- (bms-current)))
            (if (> ichg (bms-get-param 'max_charge_current)) (latch-charge-fault))
            (if (<= (bms-get-vchg) 5.0) { (set-chg false) (break) })
            (if (or (not (bms-control-ok))
                    (> (str-len (charge-block-reason)) 0)) {
                (set-chg false)
                (break)
            })
            ; A late or incomplete startup scan cannot renew the lease.
            (if (not (bms-control-feed)) {
                (fail-close-outputs true)
                (exit-error 0)
            })
            (if (> ichg (bms-get-param 'min_charge_current)) {
                (setq do-sleep false)
                (setq charge-session-valid true)
                (break)
            })
        })
    })

    ; Match VBMS32's low-SOC sleep policy, except that JFBMS32 shuts down
    ; instead of selecting a longer sleep interval. This is intentionally
    ; evaluated only after a deep-sleep timer wake.
    (if (low-soc-timer-wake wake-source chg-detected) (bms-shutdown-low-soc-timer))

    ; Trap bms-sleep failures so a transient mutex
    ; timeout or BQ NAK doesn't put ESP into deep sleep with the BQs
    ; still in ACTIVE mode (top-cell drain). On failure we defer the
    ; sleep-deep call and let the next start-fun retry try again.
    (if do-sleep
        (match (trap (with-com '(do-bms-sleep)))
            ((exit-ok _) {
                (print "bms-sleep ok, entering sleep-deep")
                (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
                (save-rtc-val)
                (prepare-external-wakeup)
                (sleep-deep (sleep-duration-s))
            })
            (_ {
                (print "bms-sleep FAILED in start-fun -- deferring sleep-deep, will retry")
                (sleep-fail-alarm)
                (sleep 1.0)
    })))
})

; === TODO===
;
; = Sleep =
;  - Go to sleep when key is left on

; Persistent settings
; Format: (label . (offset type))
(def eeprom-addrs '(
    (ver-code    . (0 i))
    (ah-cnt      . (1 f))
    (wh-cnt      . (2 f))
    (ah-chg-tot  . (3 f))
    (wh-chg-tot  . (4 f))
    (ah-dis-tot  . (5 f))
    (wh-dis-tot  . (6 f))
    (ah-cnt-soc  . (7 f))
    (bq-hard-fault . (8 i))
    (current-zero . (9 f))))

; Settings version
(def settings-version 243i32)

(defun read-setting (name)
    (let (
        (addr (first (assoc eeprom-addrs name)))
        (type (second (assoc eeprom-addrs name))))
        (cond
            ((eq type 'i) (eeprom-read-i addr))
            ((eq type 'f) (eeprom-read-f addr))
            ((eq type 'b) (!= (eeprom-read-i addr) 0)))))

(defun write-setting (name val)
    (let (
        (addr (first (assoc eeprom-addrs name)))
        (type (second (assoc eeprom-addrs name))))
        (cond
            ((eq type 'i) (eeprom-store-i addr val))
            ((eq type 'f) (eeprom-store-f addr val))
            ((eq type 'b) (eeprom-store-i addr (if val 1 0))))))

(defun number-or (value fallback) (if (number? value) value fallback))

(defun rtc-number (name fallback) (number-or (rtc-get name fallback) fallback))

(defun sanitize-rtc-val () {
    (setassoc rtc-val 'wakeup-cnt (rtc-number 'wakeup-cnt 0))
    (setassoc rtc-val 'sleep-enter-time-s (rtc-number 'sleep-enter-time-s 0))
    (setassoc rtc-val 'sleep-total-time-s (rtc-number 'sleep-total-time-s 0))
    (setassoc rtc-val 'c-min (rtc-number 'c-min 3.5))
    (setassoc rtc-val 'c-max (rtc-number 'c-max 3.5))
    (setassoc rtc-val 'v-tot (rtc-number 'v-tot 50.0))
    (setassoc rtc-val 'soc (truncate (rtc-number 'soc 0.5) 0.0 1.0))
    (setassoc rtc-val 'last-bq-init-attempts (rtc-number 'last-bq-init-attempts 0))
    (setassoc rtc-val 'last-bq-detect-08 (rtc-number 'last-bq-detect-08 0))
    (setassoc rtc-val 'last-bq-detect-10 (rtc-number 'last-bq-detect-10 0))
    (setassoc rtc-val 'last-bq-wake-stage (rtc-number 'last-bq-wake-stage 0))
    (setassoc rtc-val 'last-bq-wake-time-s (rtc-number 'last-bq-wake-time-s 0))
})

(defun save-settings () {
    (write-setting 'ah-cnt ah-cnt)
    (write-setting 'wh-cnt wh-cnt)
    (write-setting 'ah-chg-tot ah-chg-tot)
    (write-setting 'wh-chg-tot wh-chg-tot)
    (write-setting 'ah-dis-tot ah-dis-tot)
    (write-setting 'wh-dis-tot wh-dis-tot)
    (write-setting 'ah-cnt-soc ah-cnt-soc)
})

(defun lpf (val sample tc) (- val (* tc (- val sample))))

(defun status-append (base part)
    (if (> (str-len part) 0)
        (if (> (str-len base) 0) (str-merge base " | " part) part)
        base))

(defun bq-fault-str-one (prefix flags) {
    (var s "")
    (loopforeach fault '((0x80 "_SCD") (0x40 "_OCD2") (0x20 "_OCD1")
                        (0x10 "_OCC") (0x08 "_COV") (0x04 "_CUV")
                        (0x4000 "_SCDL") (0x2000 "_OCDL") (0x0200 "_HWDF")) {
        (if (!= (bitwise-and flags (first fault)) 0)
            (setq s (status-append s (str-merge prefix (second fault)))))
    })
    s
})

(defun update-bq-status () {
    ; BQ1 alone has the shunt; import native faults even after hardware recovery.
    (var native-mask (bms-protection-status))
    (setq bq-safety-flags (bitwise-or
        (bitwise-and (bms-direct-cmd 1 0x03) 0xff)
        (shl (bitwise-and (bms-direct-cmd 1 0x07) 0xff) 8)))
    (var observed-mask (bitwise-or (bitwise-and bq-safety-flags 0x62f0) native-mask))
    (if (!= observed-mask 0) {
        (set-chg false)
        (var latched (bitwise-or bq-hard-fault-mask observed-mask))
        (if (!= latched bq-hard-fault-mask) {
            (spawn (fn () (user-beep-low 2 0.6))) ; charge fault
            (persist-bq-hard-fault latched)
            (cancel-bq-reset "")
        })
    })
    (bq-fault-str-one "BQ1" bq-safety-flags)
})

(defun bq-current-fault-active () (!= (bitwise-and bq-safety-flags 0x62f0) 0))

(defun persist-bq-hard-fault (mask) {
    (setq bq-hard-fault-mask mask)
    (setq rtc-val (setassoc rtc-val 'bq-hard-fault-mask mask))
    (save-rtc-val)
    ; Write only on latch changes. Marker distinguishes an unused EEPROM slot.
    (write-setting 'bq-hard-fault (bitwise-or 0x4a460000 mask))
})

(defun load-bq-hard-fault () {
    (var stored (number-or (read-setting 'bq-hard-fault) 0))
    (var flash-mask (if (= (bitwise-and stored 0xffff0000) 0x4a460000)
        (bitwise-and stored 0x62f0) 0))
    (setq bq-hard-fault-mask (bitwise-or flash-mask
        (bitwise-and (rtc-number 'bq-hard-fault-mask 0) 0x62f0)))
})

(defun cancel-bq-reset (message) {
    (setq bq-reset-requested false)
    (setq bq-reset-ts nil) ; nil means no recovery command has been sent.
    (setq bq-reset-message message)
})

(defun handle-charge-allow (allow) {
    (cancel-bq-reset "")
    (setq chg-allowed (= allow 1))
    (if chg-allowed (setq charge-no-current false))
    ; Each Chg En is a new request, even when permission was already true.
    (setq bq-reset-requested (and chg-allowed (!= bq-hard-fault-mask 0)))
})

(defun bq-reset-step () {
    (set-chg false)
    (if (or (not chg-allowed) (not temps-valid) (not (bms-control-ok))
            (> (abs (bms-get-current)) 0.2)) (exit-error 0))
    ; Permanent faults are not recoverable through Chg En.
    (loopforeach reg '(0x0b 0x0d 0x0f 0x11) {
        (if (!= (bitwise-and (bms-direct-cmd 1 reg) 0xff) 0) (exit-error 0))
    })
    (if (not bq-reset-ts) {
        (setq bq-reset-ts (systime))
        ; One send per press; integer 0 means a failed command.
        (loopforeach latch '((0x4000 0x009c) (0x2000 0x009b)) {
            (if (!= (bitwise-and bq-safety-flags (first latch)) 0)
                (if (!= (bms-subcmd-cmdonly 1 (second latch)) 1) (exit-error 0)))
        })
    })
    (setq bq-status (update-bq-status))
    (if (or (not bq-reset-requested) (not chg-allowed)) (exit-error 0))
    (if (not (bq-current-fault-active)) {
        ; Revalidate authorization immediately before native DDSG release.
        ; Native reset keeps the MCU gate off and verifies faults/current/DDSG.
        (if (not (and bq-reset-requested chg-allowed temps-valid (bms-control-ok)))
            (exit-error 0))
        (if (!= (bms-protection-reset) 1) (exit-error 0))
        (if (or (not bq-reset-requested) (not chg-allowed)) {
            (bms-protection-lock bq-hard-fault-mask)
            (exit-error 0)
        })
        (setassoc rtc-val 'charge-fault false)
        (persist-bq-hard-fault 0)
        (cancel-bq-reset "")
        (setq bq-status-latched "")
        (setq charge-ts (systime))
    } {
        ; Wait for the BQ safety engine; never re-send the recovery command.
        (setq bq-reset-message "RESET_PENDING")
        (if (> (secs-since bq-reset-ts) 6.0) (exit-error 0))
    })
})

(defun process-bq-reset () {
    (if bq-reset-requested {
        ; One attempt per press: with-com's automatic retry is inappropriate here.
        (mutex-lock com-mutex)
        (var result (trap { (bms-set-com 0) (bq-reset-step) }))
        (mutex-unlock com-mutex)
        (match result
            ((exit-ok _) true)
            (_ (cancel-bq-reset "RESET_BLOCKED")))
    })
})

(defun latch-charge-fault () {
    (if (not (assoc rtc-val 'charge-fault)) (spawn (fn () (user-beep-low 2 0.6)))) ; charge fault
    (setassoc rtc-val 'charge-fault true)
})

(defun set-chg (chg) {
    (if (and chg temps-valid (bms-control-ok) (= bq-hard-fault-mask 0)
            (not (bq-current-fault-active))) {
        (var enabled (match (trap { (bms-set-com 0) (bms-set-chg 1) })
            ((exit-ok (? value)) value) (_ false)))
        (setq charge-hw-refused (not enabled))
        (if enabled {
            (if (not is-charging) {
                (setq charge-ts (systime))
                (if (not charge-enable-beeped) {
                    (setq charge-enable-beeped true)
                    (spawn charge-start-beep)
                })
            })
            (setq is-charging true)
        } {
            (bms-set-chg 0)
            (setq is-charging false)
            (setq charge-ok false)
            (setq charge-session-valid false)
        })
    } {
        ; Trigger balancing only when a real, fault-free charge session
        ; ends. Voltage alone must not arm balancing.
        (if (and
            is-charging
            charge-session-valid
            temps-valid
            (bms-control-ok)
            (not (assoc rtc-val 'charge-fault))
            (= bq-hard-fault-mask 0)
            (not (bq-current-fault-active))
        ) {
            (setq trigger-bal-after-charge true)
            (setq balance-request-ts (systime))
        })
        (setq charge-session-valid false)
        (bms-set-chg 0)
        (setq is-charging false)
    })
})

; Leave the charger uninterrupted during its startup grace. Afterwards, low
; port voltage or current ends charging without gate-off measurement probes.
(defun charge-control-step () {
    (var low-current (<= charge-current-filt (bms-get-param 'min_charge_current)))
    (var charger-detected (> vt-vchg 5.0))
    ; Refresh presence before considering any five-second disconnect reset.
    (if charger-detected (setq charge-dis-ts (systime)))
    (if (and (not charger-detected) (> (secs-since charge-dis-ts) 5.0)) {
        (setassoc rtc-val 'charge-fault false)
        (setq charge-complete false)
        (setq charge-complete-msg false)
        (setq charge-no-current false)
        (setq charge-block-beeped false)
        (setq charge-enable-beeped false)
    })
    (if (and charger-detected (not charger-detected-prev)) {
        (setq charge-ts (systime))
        (setq charge-block-beeped false)
        (setq charge-enable-beeped false)
        (print "CHG: charger detected")
    })
    (setq charger-detected-prev charger-detected)

    ; The measured end voltage completes charging immediately.
    (if (and is-charging (>= c-max (bms-get-param 'vc_charge_end))) {
        (if (not charge-complete) {
            (spawn (fn () (user-beep 3 0.2)))
            (print "CHG complete: CELL_LIMIT")
        })
        (setq charge-complete true)
        (setq charge-complete-msg true)
    })
    ; Top-up, as on JFBMS Master: once post-charge balancing is finished and
    ; the settled cells fall below vc_charge_start, a new charge may start.
    (if (and charge-complete (not is-charging) (not is-balancing)
            (not trigger-bal-after-charge)
            (< c-max (bms-get-param 'vc_charge_start))) {
        (setq charge-complete false)
        (setq charge-complete-msg false)
        (setq charge-ts (systime))
        (print "CHG rearmed below vc_charge_start")
    })
    (if (and is-charging (> charge-current (bms-get-param 'max_charge_current))) (latch-charge-fault))
    (if (and is-charging charger-detected (not charge-complete)
            (>= (secs-since charge-ts) charger-max-delay) low-current) {
        (if (and charge-session-valid (>= c-max (bms-get-param 'vc_charge_start))) {
            (setq charge-complete true)
            (setq charge-complete-msg true)
            (spawn (fn () (user-beep 3 0.2)))
            (print "CHG complete: CURRENT_TAPER")
        } (setq charge-no-current true))
    })
    (process-bq-reset)
    ; Retry a transient native refusal after the next complete safety scan.
    (setq charge-hw-refused false)
    (setq charge-ok (= (str-len (charge-block-reason)) 0))
    (if (and charger-detected charge-ok
            (or is-charging (> vt-vchg (+ vtot 0.7)))) {
        (set-chg true)
    } {
        (set-chg false)
        ; Starting hysteresis is not evidence that the battery is full.
        (if charge-complete (setq ah-cnt-soc (bms-get-param 'batt_ah)))
    })
    charger-detected
})

(defun send-can-info () {
    (var buf-canid35 (array-create 8))
    (var ah-left (* (bms-get-param 'batt_ah) (- 1.0 soc)))
    (var min-left (if (< iout -1.0) (* (/ ah-left (- iout)) 60.0) 0.0))
    (bufset-i16 buf-canid35 0 (* soc 1000)) ; Battery A SOC
    (bufset-u8 buf-canid35 2 (if is-charging 1 0)) ; Battery A Charging
    (bufset-u16 buf-canid35 3 min-left) ; Battery A Charge Time Minutes
    (bufset-u16 buf-canid35 5 (* (bms-get-param 'batt_ah) 10.0))
    (can-send-sid 35 buf-canid35)
    (send-bms-can)
})

(defun do-bms-sleep () { (bms-sleep) })

(defun main-ctrl () {
    ; Opt in only for normal operation. Direct hardware calls do not arm this.
    (bms-control-start)
    (loopwhile t {
        ; Exit if any of the BQs has fallen asleep
        (if (or
            (= (bms-direct-cmd 1 0x00) 4)
            (and (> (bms-get-param 'cells_ic2) 0) (= (with-com '(bms-direct-cmd 2 0x00)) 4)))
            (exit-error 0))
        (setq bq-status (update-bq-status))
        (if (> (str-len bq-status) 0) (setq bq-status-latched bq-status))
        (if (and (> (str-len bq-status-latched) 0) (= bq-hard-fault-mask 0) (> (secs-since charge-dis-ts) 5.0)) {
            (setq bq-status-latched "")
        })
        (var v-cells (with-com '(bms-get-vcells)))
        (var bms-temps (update-temps))
        (var temp-ext-num (truncate (bms-get-param 'temp_num) 0 4))
        (var c-sorted (sort < v-cells))
        (setq c-min (ix c-sorted 0))
        (setq c-max (ix c-sorted -1))
        (setq vtot (apply + v-cells))
        (setq vout (with-com '(bms-get-vout)))
        (setq vt-vchg (bms-get-vchg))
        (setq charge-current (- (with-com '(bms-current))))
        ; Same EMA as the master (0.85 per ~100 ms scan) for end-of-charge only;
        ; over-current checks keep the raw value.
        (setq charge-current-filt (if is-charging (lpf charge-current-filt charge-current 0.15) charge-current))
        (setq iout (+ (- charge-current) (can-sum-current)))
        (if (and is-charging (> charge-current (bms-get-param 'min_charge_current))) {
            (setq charge-session-valid true)
        })
        (if (and is-balancing (not (balance-safe-now))) {
            (setq bal-ok false)
            (disable-balancing)
        })
        (var vc-len (length v-cells))
        (set-bms-val 'bms-cell-num vc-len)
        (var cell0-report-offset (bms-cell0-report-offset iout))
        (looprange i 0 vc-len {
            (set-bms-val 'bms-v-cell i (- (ix v-cells i) (if (= i 0) cell0-report-offset 0.0)))
            (set-bms-val 'bms-bal-state i (bms-get-bal i))
        })
        (set-bms-val 'bms-temp-adc-num (+ 5 temp-ext-num))
        (set-bms-val 'bms-temps-adc 0 t-ic) ; IC
        (set-bms-val 'bms-temps-adc 1 t-min) ; Cell Min
        (set-bms-val 'bms-temps-adc 2 t-max) ; Cell Max
        (set-bms-val 'bms-temps-adc 3 t-mos) ; Mosfet
        (set-bms-val 'bms-temps-adc 4 -300.0) ; Ambient
        (looprange i 0 temp-ext-num { (set-bms-val 'bms-temps-adc (+ 5 i) (ix bms-temps (+ i 1))) })
        (set-bms-val 'bms-data-version 1)
        (set-bms-val 'bms-v-cell-min c-min)
        (set-bms-val 'bms-v-cell-max c-max)
        (var batt-ah (bms-get-param 'batt_ah))
        (if (> batt-ah 0.0) (setq ah-cnt-soc (truncate ah-cnt-soc 0.0 batt-ah)))
        (if (and (= (bms-get-param 'soc_use_ah) 1) (> batt-ah 0.0)) {
            ; Coulomb counting
            (setq soc (/ ah-cnt-soc batt-ah))
        } {
            (if (>= soc 0.0)
                (setq soc (lpf soc (calc-soc c-min) (truncate (* 100.0 (bms-get-param 'soc_filter_const)) 0.0 1.0)))
                (setq soc (calc-soc c-min)))
        })
        (var dt (secs-since t-last))
        (setq t-last (systime))
        (var ah (* iout (/ dt 3600.0)))
        (if (> batt-ah 0.0) (setq ah-cnt-soc (truncate (- ah-cnt-soc ah) 0.0 batt-ah)))

        ; Ah and Wh cnt
        (if (> (abs iout) (bms-get-param 'min_current_ah_wh_cnt)) {
            (var wh (* ah vtot))
            (setq ah-cnt (+ ah-cnt ah))
            (setq wh-cnt (+ wh-cnt wh))
            (if (> iout 0.0) {
                (setq ah-dis-tot (+ ah-dis-tot ah))
                (setq wh-dis-tot (+ wh-dis-tot wh))
            } {
                (setq ah-chg-tot (- ah-chg-tot ah))
                (setq wh-chg-tot (- wh-chg-tot wh))
            })
        })
        (set-bms-val 'bms-v-tot vtot)
        (set-bms-val 'bms-v-charge vt-vchg)
        (set-bms-val 'bms-i-in-ic iout)
        (set-bms-val 'bms-temp-ic t-ic)
        (set-bms-val 'bms-temp-cell-max t-max)
        (set-bms-val 'bms-soc soc)
        (set-bms-val 'bms-soh 1.0)
        (set-bms-val 'bms-ah-cnt ah-cnt)
        (set-bms-val 'bms-wh-cnt wh-cnt)
        (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
        (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
        (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
        (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)
        (with-com '(send-can-info))

        ;;; Charge control

        (var charger-detected (charge-control-step))

        ; Report the output state after applying this scan's charge decision.
        (setq chg-status (cond
            ((not temps-valid) "TEMP_INVALID")
            ((assoc rtc-val 'charge-fault) {
                (setq charge-complete-msg false)
                "FLT_CHG_OC"
            })
            (charge-complete-msg "CHG_COMPLETE")
            (is-charging {
                (setq charge-complete-msg false)
                "CHARGING"
            })
            (charger-detected {
                (var reason (charge-block-reason))
                (if (> (str-len reason) 0) reason
                    (if (<= vt-vchg (+ vtot 0.7)) "CHG_VOLTAGE_LOW" "CHG_NO_CURRENT"))
            })
            (true "")))
        ; Only a block that outlasts plug-in transients; complete is not blocked.
        (if (and charger-detected (not is-charging) (> (str-len chg-status) 0)
                (not-eq chg-status "CHG_COMPLETE") (> (secs-since charge-ts) 3.0)
                (not charge-block-beeped)) {
            (setq charge-block-beeped true)
            (spawn charge-block-beep)
        })

        ; Set combined BMS status
        (var bq-status-display (if (> (str-len bq-status) 0) bq-status bq-status-latched))
        (if (!= bq-hard-fault-mask 0) {
            (setq bq-status-display (status-append
                (bq-fault-str-one "BQ1" bq-hard-fault-mask)
                (if (> (str-len bq-reset-message) 0) bq-reset-message "PRESS_CHG_EN_TO_RESET")))
        })
        (var output-fault-status "")
        (if bal-off-failed (setq output-fault-status (status-append output-fault-status "BAL_OFF_FAIL")))
        (if fail-close-failed (setq output-fault-status (status-append output-fault-status "FAIL_CLOSE_FAIL")))
        (if (and
            charge-complete-msg
            (or (> (str-len bq-status-display) 0) (> (str-len output-fault-status) 0))
        ) {
            (setq charge-complete-msg false)
            (setq chg-status "")
        })
        (set-bms-val 'bms-status
            (status-append
                (status-append (status-append chg-status bq-status-display) bal-status)
                output-fault-status))

        ;;; Sleep
        (setassoc rtc-val 'c-min c-min)
        (setassoc rtc-val 'c-max c-max)
        (setassoc rtc-val 'v-tot vtot)
        (setassoc rtc-val 'soc soc)
        (setassoc rtc-val 'updated true)
        (save-rtc-val)

        ; Measure time without current
        (if (> (abs iout) (bms-get-param 'min_current_sleep))
            (setq i-zero-time 0.0)
            (setq i-zero-time (+ i-zero-time dt)))

        ; Go to sleep when button is off, not balancing and not connected
        (if (and trigger-bal-after-charge balance-request-ts
                (> (secs-since balance-request-ts) 60.0) (not is-balancing))
            (setq trigger-bal-after-charge false))
        (if (and (external-wake-inactive) (> i-zero-time 1.0) (not is-charging) (not charger-detected) (not trigger-bal-after-charge) (not is-balancing) (not (is-connected)) (not (can-active))) {
            (sleep 0.1)
            (if (external-wake-inactive) {
                (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
                (save-rtc-val)
                (save-settings)
                ; See start-fun for rationale.
                (match (trap (with-com '(do-bms-sleep)))
                    ((exit-ok _) {
                        (print "bms-sleep ok, entering sleep-deep")
                        (prepare-external-wakeup)
                        (sleep-deep (sleep-duration-s))
                    })
                    (_ {
                        (print "bms-sleep FAILED in main-ctrl idle path -- deferring sleep-deep, will retry")
                        (sleep-fail-alarm)
                        (sleep 1.0)
                }))
            })
        })

        ; Set SOC to 0 below 2.9V and not under load.
        (if (and (> i-zero-time 10.0) (<= c-min (bms-get-param 'vc_empty))) { (setq ah-cnt-soc 0.0) })

        ; Only a completed scan with valid sensors renews the five-second timer.
        ; A late scan cannot clear a timeout; supervised reinitialization is needed.
        (if temps-valid {
            (if (not (bms-control-feed)) {
                (set-bms-val 'bms-status "CTRL_TIMEOUT")
                (fail-close-outputs true)
                (exit-error 0)
            })
        })
        (if (not shutdown-in-progress) (wdt-reset))
        (sleep 0.1)
    })
})

; Balancing
(defun balance () (loopwhile t {
    ; Disable balancing and wait for a bit to get clean
    ; measurements
    (looprange i 0 cell-num (with-com `(bms-set-bal ,i 0)))
    (sleep 2.0)
    (var v-cells (with-com '(bms-get-vcells)))
    (var vc-len (length v-cells))
    (var cells-sorted (sort (fn (x y) (> (ix x 1) (ix y 1)))
        (map (fn (x) (list x (ix v-cells x))) (range vc-len))))
    (var c-min (second (ix cells-sorted -1)))
    (var c-max (second (ix cells-sorted 0)))
    (if trigger-bal-after-charge (setq bal-ok true))
    (if is-charging { (setq bal-ok false) })
    (if (not temps-valid) (setq bal-ok false))
    (if (not (bms-control-ok)) (setq bal-ok false))
    (if (> (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current)) {
        (setq bal-ok false)
        (if (> iout (bms-get-param 'balance_max_current)) { (setq trigger-bal-after-charge false) })
    })
    (if (< c-min (bms-get-param 'vc_balance_min)) { (setq bal-ok false) })
    (if (> t-max (bms-get-param 't_bal_max_cell)) { (setq bal-ok false) })
    (if (> t-ic (bms-get-param 't_bal_max_ic)) { (setq bal-ok false) })
    (if (<= (bms-get-param 'max_bal_ch) 0) {
        (setq bal-ok false)
        (setq trigger-bal-after-charge false)
    })
    (if bal-ok {
        ; Same rule as JFBMS Master: highest cells first, never two adjacent
        ; cells on the same BQ, at most max_bal_ch channels per BQ.
        (var bal-chs (map (fn (x) 0) (range vc-len)))
        (var ch-cnt 0)
        (var ic1-cells (first active-hw-config))
        (var ic-cnt (list 0 0))
        (var max-ch (bms-get-param 'max_bal_ch))
        (var threshold (bms-get-param (if is-balancing 'vc_balance_end 'vc_balance_start)))
        (loopforeach c cells-sorted {
            (var n-cell (first c))
            (var ic (if (< n-cell ic1-cells) 0 1))
            (if (and
                (> (- (second c) c-min) threshold)
                (< (ix ic-cnt ic) max-ch)
                (or (= n-cell 0) (= n-cell ic1-cells) (= (ix bal-chs (- n-cell 1)) 0))
                (or (= n-cell (- vc-len 1)) (= n-cell (- ic1-cells 1)) (= (ix bal-chs (+ n-cell 1)) 0))) {
                    (setix bal-chs n-cell 1)
                    (setix ic-cnt ic (+ (ix ic-cnt ic) 1))
                    (setq ch-cnt (+ ch-cnt 1))
            })
        })
        (if (> ch-cnt 0) {
            (setq trigger-bal-after-charge false)
            (looprange i 0 vc-len {
                (if (not (bms-control-ok)) (exit-error 0))
                (with-com `(bms-set-bal ,i ,(ix bal-chs i)))
            })
            (setq is-balancing true)
        } {
            (setq bal-ok false)
            (setq trigger-bal-after-charge false)
        })
    })
    (if (not bal-ok) { (disable-balancing) })
    (setq bal-status (if is-balancing "BAL" ""))
    ; 30 s balancing, then the 2 s settle above, as on JFBMS Master.
    (var bal-ok-before bal-ok)
    (looprange i 0 30 {
        (if (not (eq bal-ok-before bal-ok)) (break))
        (if (and trigger-bal-after-charge (not is-balancing)) (break))
        (sleep 1.0)
    })
}))

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-chg-allow (? allow)) (handle-charge-allow allow))
            ((event-bms-reset-cnt (? ah) (? wh)) {
                (if (= ah 1) (setq ah-cnt 0.0))
                (if (= wh 1) (setq wh-cnt 0.0))
            })
            ((event-bms-force-bal (? v)) (if (= v 1) (setq bal-ok true) (setq bal-ok false)))
            (event-bms-zero-ofs {
                (var zero (with-com '(bms-current-raw)))
                (if (and (number? zero) (>= zero -2.0) (<= zero 2.0)) {
                    (if (write-setting 'current-zero zero) {
                        (setq current-zero-offset zero)
                        (print "CAL: zero captured and stored")
                        (spawn (fn () (user-beep 2 0.2)))
                    } (print "CAL: could not store current zero"))
                } (print "CAL: zero rejected; remove current before calibration"))
            })
            ((event-data-rx ? data) (handle-app-data data))
            (_ nil)
)))

(defun handle-app-data (data)
    (match (trap (read data))
        ((exit-ok (bms-shutdown)) {
            (print "APPUI requested BMS shutdown")
            (spawn (fn () (bms-shutdown-app)))
        })
        (_ (print "Ignoring unsupported APPUI command"))))

; First three entries are cell counts and external temperature-sensor count.
(defun bms-hw-config () (map (fn (param) (bms-get-param param))
    '(cells_ic1 cells_ic2 temp_num temp_res hw_occ_current hw_ocd_current psw_scd_tres)))

(defun main () {
    (loopwhile-thd ("io-diag" 100) t {
        (var previous nil)
        (loopwhile t {
            (sleep 5.0)
            (var counts (list (bms-i2c-errors)))
            (if (and (not-eq counts previous) (> (apply + counts) 0)) {
                (print "I2C/CRC errors:" counts)
                (setq previous counts)
            })
        })
    })
    ; Compatibility Check
    (loopwhile (!= (bms-fw-version) 9) {
        (if (< (bms-fw-version) 9)
            (print "Firmware too old, please update")
            (print "Package too old, please update"))
        (bms-set-com 0) ; Enable CAN
        (sleep 5)
    })
    (if (> app-wdt-timeout 0) (wdt-configure true app-wdt-timeout) (wdt-disable))
    (set-fw-name "")
    (def charge-dis-ts (systime))
    (def t-last (systime))
    (def charge-ts (systime))

    ; Buzzer
    (pwm-start beep-freq-high 0.0 0 buzzer-pin)
    (if (= (bufget-u8 (rtc-data) 900) rtc-val-magic) {
        (var tmp (unflatten (rtc-data)))
        (if tmp (setq rtc-val tmp))
    })
    (sanitize-rtc-val)
    (load-bq-hard-fault)
    (sync-bq-wake-debug-globals)
    (def active-hw-config (bms-hw-config))
    (def cell-num (+ (first active-hw-config) (second active-hw-config)))

    ; Timer shutdown can happen inside start-fun, before the normal main
    ; loop starts. Load counters now so save-settings is always safe.
    (var stored-version (read-setting 'ver-code))
    (def settings-valid (or (= stored-version 242i32) (= stored-version settings-version)))
    (var saved-zero (if (= stored-version settings-version) (read-setting 'current-zero) 0.0))
    (setq current-zero-offset (if (and (number? saved-zero) (>= saved-zero -2.0) (<= saved-zero 2.0)) saved-zero 0.0))
    (if (= stored-version 242i32) {
        (write-setting 'current-zero current-zero-offset)
        (write-setting 'ver-code settings-version)
    })
    (def ah-cnt (number-or (read-setting 'ah-cnt) 0.0))
    (def wh-cnt (number-or (read-setting 'wh-cnt) 0.0))
    (def ah-chg-tot (number-or (read-setting 'ah-chg-tot) 0.0))
    (def wh-chg-tot (number-or (read-setting 'wh-chg-tot) 0.0))
    (def ah-dis-tot (number-or (read-setting 'ah-dis-tot) 0.0))
    (def wh-dis-tot (number-or (read-setting 'wh-dis-tot) 0.0))
    (def ah-cnt-soc (if settings-valid
        (number-or (read-setting 'ah-cnt-soc) -1.0)
        (* (rtc-number 'soc 0.5) (bms-get-param 'batt_ah))))
    (loopwhile t {
        (match (trap (start-fun))
            ((exit-ok (? a)) (break))
            (_ (fail-close-outputs true)))
        (sleep 1.0)
    })

    ; If the settings version changed, preserve the accumulated Ah/Wh
    ; counters. Only re-seed the SOC counter from the measured cell
    ; voltage and mark the EEPROM as using the current layout.
    (if (not settings-valid) {
        (setq ah-cnt-soc (* (calc-soc c-min) (bms-get-param 'batt_ah)))
        (write-setting 'ah-cnt-soc ah-cnt-soc)
        (write-setting 'current-zero current-zero-offset)
        (write-setting 'ver-code settings-version)
        (setq settings-valid true)
    })
    (event-register-handler (spawn event-handler))
    (event-enable 'event-bms-chg-allow)
    (event-enable 'event-bms-reset-cnt)
    (event-enable 'event-bms-force-bal)
    (event-enable 'event-bms-zero-ofs)
    (event-enable 'event-data-rx)
    (set-bms-val 'bms-cell-num cell-num)
    (set-bms-val 'bms-can-id (can-local-id))
    (loopwhile-thd ("main-ctrl" 200) t {
        (trap (main-ctrl))
        (setq did-crash true)
        (loopwhile did-crash (sleep 1.0))
    })
    (loopwhile-thd ("balance" 200) t {
        (trap (balance))
        (setq did-crash true)
        (loopwhile did-crash (sleep 1.0))
    })
    (loopwhile-thd ("re-init" 200) t {
        (var cfg (bms-hw-config))
        (if (not-eq cfg active-hw-config) {
            (print "BMS config changed, reinitializing hardware")
            (cancel-bq-reset "")
            ; Fail closed before any BQ communication that could block.
            (fail-close-outputs true)
            (com-force true)
            (init-hw)
            (spawn (fn () (user-beep 2 0.2)))
            (setq active-hw-config cfg)
            (setq cell-num (+ (first cfg) (second cfg)))
            (set-bms-val 'bms-cell-num cell-num)
            (set-bms-val 'bms-temp-adc-num (+ 5 (truncate (third cfg) 0 4)))
            (com-force false)
        })
        (if did-crash {
            (cancel-bq-reset "")
            ; Fail closed immediately. init-hw can loop while recovering
            ; BQ communication, so do not leave the charge gate enabled
            ; or balance channels active while recovery is waiting on the bus.
            (fail-close-outputs true)
            (com-force true)
            (init-hw)
            (com-force false)
            (fail-close-outputs true)
            (setq did-crash false)
            (setq crash-cnt (+ crash-cnt 1))
        })
        (sleep 0.1)
    })
    (loopwhile-thd ("fail-close-retry" 100) t {
        (if fail-close-failed { (fail-close-outputs true) })
        (if (and bal-off-failed (not fail-close-failed)) { (disable-balancing) })
        (sleep 0.5)
    })
    (loopwhile-thd ("sleep-unblock" 100) t {
        (var sleep-unblock-ok (fn () (and
            (= (bms-get-param 'block_sleep) 1)
            (< (- c-max c-min) 0.05)
            (> c-min 2.4)
            (> (secs-since 0) 3600)
            sleep-unblock-en)))
        (var should-unblock true)
        (looprange i 0 60 {
            (if (not (sleep-unblock-ok)) { (setq should-unblock false) })
            (sleep 1.0)
        })
        (if should-unblock {
            (bms-set-param 'block_sleep 0)
            (bms-store-cfg)
            (print "Block sleep disabled")
            (user-beep 4 0.2)
        })
    })
})

@const-end

(image-save)
(main)

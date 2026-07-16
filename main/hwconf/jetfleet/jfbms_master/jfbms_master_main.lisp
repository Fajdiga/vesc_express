; JFBMS Master
; Pack controller for JFBMS slave boards. The master owns charge, sleep,
; shutdown, counters, status and balance decisions. Cell voltages and cell
; temperatures are taken only from slave CAN data.

;;;;;;;;;; User settings ;;;;;;;;;;

(def charger-max-delay 10.0) ; Seconds to wait for charger current after enabling charge FET.
(def app-wdt-timeout 120)    ; Seconds. Set to 0 to disable the app watchdog.
(def user-beeps-en true)
(def beep-duty-normal 0.5)
(def sleep-unblock-en true)

;;;;;;;;;; State ;;;;;;;;;;

(def slave-timeout-ms 300)
(def chg-allowed true)
(def charge-ok false)
(def is-charging false)
(def charge-complete false)
(def charge-complete-msg false)
(def charger-detected-prev false)
(def trigger-bal-after-charge false)

(def bal-request false)
(def bal-auto-retry-ts (systime))
(def bal-status "")
(def chg-status "")
(def pack-status "")

(def c-min 0.0)
(def c-max 0.0)
(def vtot 0.0)
(def vt-vchg 0.0)
(def iout 0.0)
(def t-min 24.0)
(def t-max 24.0)
(def t-ic 24.0)
(def t-mos 24.0)
(def soc -1.0)
(def cell-num 0)
(def temp-data-ok false)
(def cell-temp-mon-en false)
(def pack-data-ok false)
(def slave-data-fresh false)
(def init-done false)
(def i-zero-time 0.0)
(def bal-off-failed false)
(def fail-close-failed false)
(def control-crash-count 0)

(def ah-cnt 0.0)
(def wh-cnt 0.0)
(def ah-chg-tot 0.0)
(def wh-chg-tot 0.0)
(def ah-dis-tot 0.0)
(def wh-dis-tot 0.0)
(def ah-cnt-soc -1.0)

(def charge-dis-ts (systime))
(def charge-ts (systime))
(def t-last (systime))
(def rtc-val '(
    (charge-fault . false)
    (sleep-enter-time-s . 0)
    (sleep-total-time-s . 0)
))
(def rtc-val-magic 127)

(def prev-active (list 0 0 0 0 0 0 0 0))
(def prev-bal-mask (list 0 0 0 0 0 0 0 0))
(def prev-can-overflow 0)

; Balancing state. state[0] = 1 if currently balancing, 0 if idle.
(def bal-state (list 0))
(def slave-bal-mask-ic1 (list 0 0 0 0 0 0 0 0))
(def slave-bal-mask-ic2 (list 0 0 0 0 0 0 0 0))
(def bal-keepalive-kick (list 0))

(def shutdown-reason-unknown 0)
(def shutdown-reason-timer 1)
(def shutdown-reason-low-soc-start 2)
(def shutdown-reason-low-soc-main 3)
(def shutdown-reason-app 4)

(def loop-cnt 0)
(def buz-mutex (mutex-create))

@const-start

; Pre-load dynamically provided helpers so they are included in the image.
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

;;;;;;;;;; Generic helpers ;;;;;;;;;;

(defun trap-value (expr fallback) {
    (match (trap (eval expr))
        ((exit-ok (? value)) value)
        (_ fallback)
    )
})

(defun save-rtc-val () {
    (var tmp (flatten rtc-val))
    (bufcpy (rtc-data) 0 tmp 0 (buflen tmp))
    (bufset-u8 (rtc-data) 900 rtc-val-magic)
})

(defun load-rtc-val () {
    (if (= (bufget-u8 (rtc-data) 900) rtc-val-magic) {
        (var tmp (unflatten (rtc-data)))
        (if tmp {
            (setq rtc-val tmp)
            (sanitize-rtc-val)
        })
    })
})

(defun param-or (name fallback) {
    (match (trap (bms-get-param name))
        ((exit-ok (? value)) value)
        (_ fallback)
    )
})

(defun truncate (n min max)
    (if (< n min)
        min
        (if (> n max)
            max
            n
)))

(defun cfg-num-slaves () (truncate (param-or 'num_slaves 1) 1 8))

(defun bool-int (v) (if v 1 0))

(defun lpf (val sample tc)
    (- val (* tc (- val sample)))
)

(defun calc-soc (v-cell) {
    (var empty (bms-get-param 'vc_empty))
    (var full (bms-get-param 'vc_full))
    (var den (- full empty))

    (if (= den 0.0)
        0.0
        (truncate (/ (- v-cell empty) den) 0.0 1.0)
    )
})

; Persistent counters. Keep the layout aligned with jfbms32.
(def eeprom-addrs '(
    (ver-code    . (0 i))
    (ah-cnt      . (1 f))
    (wh-cnt      . (2 f))
    (ah-chg-tot  . (3 f))
    (wh-chg-tot  . (4 f))
    (ah-dis-tot  . (5 f))
    (wh-dis-tot  . (6 f))
    (ah-cnt-soc  . (7 f))
))

(def settings-version 243i32)

(defun read-setting (name)
    (let (
            (addr (first (assoc eeprom-addrs name)))
            (type (second (assoc eeprom-addrs name)))
        )
        (cond
            ((eq type 'i) (eeprom-read-i addr))
            ((eq type 'f) (eeprom-read-f addr))
            ((eq type 'b) (!= (eeprom-read-i addr) 0))
)))

(defun write-setting (name val)
    (let (
            (addr (first (assoc eeprom-addrs name)))
            (type (second (assoc eeprom-addrs name)))
        )
        (cond
            ((eq type 'i) (eeprom-store-i addr val))
            ((eq type 'f) (eeprom-store-f addr val))
            ((eq type 'b) (eeprom-store-i addr (if val 1 0)))
)))

(defun number-or (value fallback)
    (if (number? value) value fallback)
)

(defun rtc-number (name fallback)
    (number-or (assoc rtc-val name) fallback)
)

(defun sanitize-rtc-val () {
    (setassoc rtc-val 'sleep-enter-time-s (rtc-number 'sleep-enter-time-s 0))
    ; Older images stored this counter as a float. Keep accumulated time while
    ; migrating to exact integer seconds.
    (setassoc rtc-val 'sleep-total-time-s (to-i (rtc-number 'sleep-total-time-s 0)))
})

(defun restore-settings () {
    (setq ah-cnt 0.0)
    (setq wh-cnt 0.0)
    (setq ah-chg-tot 0.0)
    (setq wh-chg-tot 0.0)
    (setq ah-dis-tot 0.0)
    (setq wh-dis-tot 0.0)
    (setq ah-cnt-soc (if (valid-pack-reading)
        (* (calc-soc c-min) (bms-get-param 'batt_ah))
        -1.0
    ))

    (write-setting 'ah-cnt ah-cnt)
    (write-setting 'wh-cnt wh-cnt)
    (write-setting 'ah-chg-tot ah-chg-tot)
    (write-setting 'wh-chg-tot wh-chg-tot)
    (write-setting 'ah-dis-tot ah-dis-tot)
    (write-setting 'wh-dis-tot wh-dis-tot)
    (write-setting 'ah-cnt-soc ah-cnt-soc)
    (write-setting 'ver-code settings-version)
})

(defun load-settings () {
    (if (not-eq (read-setting 'ver-code) settings-version)
        (restore-settings)
    )

    (setq ah-cnt (number-or (read-setting 'ah-cnt) 0.0))
    (setq wh-cnt (number-or (read-setting 'wh-cnt) 0.0))
    (setq ah-chg-tot (number-or (read-setting 'ah-chg-tot) 0.0))
    (setq wh-chg-tot (number-or (read-setting 'wh-chg-tot) 0.0))
    (setq ah-dis-tot (number-or (read-setting 'ah-dis-tot) 0.0))
    (setq wh-dis-tot (number-or (read-setting 'wh-dis-tot) 0.0))
    (setq ah-cnt-soc (number-or (read-setting 'ah-cnt-soc) -1.0))
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

(defun status-append (base part)
    (if (> (str-len part) 0)
        (if (> (str-len base) 0)
            (str-merge base "|" part)
            part
        )
        base
))

(defun temp-valid (temp) (and (>= temp -40.0) (<= temp 120.0)))

(defun display-temp (temp) (if (temp-valid temp) temp 0.0))

(defun all-temps-valid () (and
    (temp-valid t-ic)
    (temp-valid t-mos)
    (or
        (not cell-temp-mon-en)
        (and (temp-valid t-min) (temp-valid t-max))
    )
))

; True when a real communication interface is connected.
(defun is-comm-connected () (or (connected-wifi) (connected-usb) (connected-ble)))

; True when VESC Tool is connected or sleep is intentionally blocked.
(defun is-connected () (or (is-comm-connected) (= (bms-get-param 'block_sleep) 1)))

(defun external-wake-active () (= (master-get-enable) 1))

(defun external-wake-inactive () (= (master-get-enable) 0))

(defun prepare-external-wakeup () {
    ; JFBMS master wakes from external requests on ENABLE.
    (master-set-enable-wakeup-state 1)
})

(defun local-sensor-status () (master-local-sensor-status))

(defun current-data-ok () (!= (bitwise-and (local-sensor-status) 0x01) 0))

(defun charger-data-ok () (!= (bitwise-and (local-sensor-status) 0x02) 0))

(defun pcb-temp-data-ok () (!= (bitwise-and (local-sensor-status) 0x04) 0))

(defun can-active () {
    (var devs (can-list-devs))
    (var active false)

    (if (not (eq devs nil)) {
        (looprange i 1 7 {
            (var age (can-msg-age (first devs) i))
            (if (and age (< age 0.1)) (setq active true))
        })
    })

    active
})

(defun sleep-duration-s () {
    (var dur (* (bms-get-param 'sleep) 3600.0))

    ; sleep-deep 0 means no timer wakeup, which is not useful for this master.
    (if (< dur 1.0)
        1.0
        dur
    )
})

(defun test-chg (samples) {
    ; Many chargers pulse before starting, so sample a few times.
    (var vchg 0.0)

    (looprange i 0 samples {
        (if (> i 0) (sleep 0.01))
        (setq vchg (master-get-vchg))
        (if (> vchg (bms-get-param 'v_charge_detect)) (break))
    })

    (var detected (> vchg (bms-get-param 'v_charge_detect)))
    (if detected (setq charge-dis-ts (systime)))
    detected
})

(defun valid-pack-reading () (and
    init-done
    pack-data-ok
    (> cell-num 0)
    (> c-min 1.0)
    (> c-max 2.0)
    (< c-min 5.0)
    (< c-max 5.0)
    (>= c-max c-min)
    (> vtot (* cell-num 1.5))
    (>= soc 0.0)
))

;;;;;;;;;; Buzzer and output control ;;;;;;;;;;

(defun beep-duty (times dt duty) {
    (mutex-lock buz-mutex)

    (loopwhile (> times 0) {
        (pwm-set-duty duty 0)
        (sleep dt)
        (pwm-set-duty 0.0 0)
        (sleep dt)
        (setq times (- times 1))
    })

    (mutex-unlock buz-mutex)
})

(defun beep (times dt) {
    (beep-duty times dt beep-duty-normal)
})

(defun user-beep (times dt) {
    (if user-beeps-en (beep times dt))
})

(defun set-chg (chg) {
    (if chg
        {
            (if (not is-charging) (setq charge-ts (systime)))
            (gpio-write 5 1)
            (setq is-charging true)
        }
        {
            ; Trigger balancing when charging ends after a real charge session.
            (if (and is-charging (> (secs-since charge-ts) 10.0))
                (setq trigger-bal-after-charge true)
            )

            (master-fail-close-local)
            (setq is-charging false)
        }
    )
})

(defun send-slave-beep (code) {
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (master-slave-active? sid)
            (master-send-balance sid
                (ix slave-bal-mask-ic1 (- sid 1))
                (ix slave-bal-mask-ic2 (- sid 1))
                code)
        )
        (setq sid (+ sid 1))
    })
})

;;;;;;;;;; Slave data aggregation ;;;;;;;;;;

(defun update-temp-globals () {
    (var max-sid (cfg-num-slaves))
    (var sid 1)
    (var temps-ok true)

    (setq t-ic -300.0)
    (setq t-min 300.0)
    (setq t-max -300.0)
    (setq t-mos (trap-value '(master-get-temp-pcb) -300.0))
    (setq cell-temp-mon-en false)

    (loopwhile (<= sid max-sid) {
        (if (master-slave-active? sid) {
            ; Slave temp order: BQ1 IC, BQ1 cell, BQ2 IC, BQ2 cell.
            ; Status field 4 is the two-bit external-sensor enable mask. The
            ; fallback requires both sensors if the status list is malformed.
            (var status (master-get-slave-status sid))
            (var temp-flags (if (and status (>= (length status) 5))
                (ix status 4)
                3
            ))
            (var bq1-cell-en (!= (bitwise-and temp-flags 0x01) 0))
            (var bq2-cell-en (!= (bitwise-and temp-flags 0x02) 0))
            (var temps (master-get-slave-temps sid))
            (var expected (if (> (master-get-cells-ic2 sid) 0) 4 2))

            (if (and temps (>= (length temps) expected)) {
                ; IC die temperatures are mandatory for every fitted BQ.
                (if (temp-valid (ix temps 0))
                    (if (> (ix temps 0) t-ic) (setq t-ic (ix temps 0)))
                    (setq temps-ok false)
                )

                ; External NTCs are mandatory only when explicitly enabled.
                (if bq1-cell-en {
                    (setq cell-temp-mon-en true)
                    (if (temp-valid (ix temps 1)) {
                        (if (< (ix temps 1) t-min) (setq t-min (ix temps 1)))
                        (if (> (ix temps 1) t-max) (setq t-max (ix temps 1)))
                    } {
                        (setq temps-ok false)
                    })
                })

                (if (= expected 4) {
                    (if (temp-valid (ix temps 2))
                        (if (> (ix temps 2) t-ic) (setq t-ic (ix temps 2)))
                        (setq temps-ok false)
                    )

                    (if bq2-cell-en {
                        (setq cell-temp-mon-en true)
                        (if (temp-valid (ix temps 3)) {
                            (if (< (ix temps 3) t-min) (setq t-min (ix temps 3)))
                            (if (> (ix temps 3) t-max) (setq t-max (ix temps 3)))
                        } {
                            (setq temps-ok false)
                        })
                    })
                })
            } {
                (setq temps-ok false)
            })
        })

        (setq sid (+ sid 1))
    })

    (if (not cell-temp-mon-en) {
        (setq t-min -300.0)
        (setq t-max -300.0)
    })

    (setq temp-data-ok (and temps-ok (all-temps-valid)))
})

(defun scan-pack-from-slaves () {
    (var max-sid (cfg-num-slaves))
    (var sid 1)
    (var active-count 0)
    (var missing false)
    (var slave-fault false)
    (var bad-cell false)
    (var stale-slave false)
    (var any-cells false)

    (setq cell-num 0)
    (setq vtot 0.0)
    (setq c-min 9.0)
    (setq c-max 0.0)

    (loopwhile (<= sid max-sid) {
        (if (master-slave-active? sid) {
            (setq active-count (+ active-count 1))
            (if (not (master-slave-fresh? sid))
                (setq stale-slave true))

            (var status (master-get-slave-status sid))
            (var faults (if status (ix status 1) 0))
            (var s-ic1 (master-get-cells-ic1 sid))
            (var s-ic2 (master-get-cells-ic2 sid))
            (var cnt (+ s-ic1 s-ic2))
            (var cells (master-get-slave-cells sid))

            (if (> (bitwise-and faults 0x03) 0)
                (setq slave-fault true)
            )

            (if (and cells (> cnt 0) (= (length cells) cnt)) {
                (loopforeach v cells {
                    (if (or (< v 1.0) (> v 5.0)) {
                        (setq bad-cell true)
                    } {
                        (setq any-cells true)
                        (setq cell-num (+ cell-num 1))
                        (setq vtot (+ vtot v))
                        (if (< v c-min) (setq c-min v))
                        (if (> v c-max) (setq c-max v))
                    })
                })
            } {
                (setq bad-cell true)
            })
        } {
            (setq missing true)
        })

        (setq sid (+ sid 1))
    })

    (if (not any-cells) {
        (setq c-min 0.0)
        (setq c-max 0.0)
    })

    (setq pack-status
        (cond
            (missing "WAIT_SLAVE")
            (slave-fault "SLAVE_FAULT")
            (bad-cell "BAD_CELL")
            ((not any-cells) "NO_CELL")
            (stale-slave "STALE_SLAVE")
            ((not temp-data-ok) "TEMP_NA")
            (true "")
        )
    )

    (setq slave-data-fresh (and
        any-cells
        (not missing)
        (not slave-fault)
        (not bad-cell)
        (not stale-slave)
    ))
    (setq pack-data-ok (and
        any-cells
        (not missing)
        (not slave-fault)
        (not bad-cell)
    ))

    (trap (set-bms-val 'bms-cell-num cell-num))
    (trap (set-bms-val 'bms-v-tot vtot))
    (trap (set-bms-val 'bms-v-cell-min c-min))
    (trap (set-bms-val 'bms-v-cell-max c-max))
    (trap (set-bms-val 'bms-temp-ic (display-temp t-ic)))
    (trap (set-bms-val 'bms-temp-cell-max (display-temp t-max)))

    pack-data-ok
})

(defun check-can-health () {
    (var overflow (master-can-overflow))
    (if (> overflow prev-can-overflow) {
        (print (str-merge "CAN RX overflow: +" (str-from-n (- overflow prev-can-overflow) "%d")
            " total=" (str-from-n overflow "%d")))
        (setq prev-can-overflow overflow)
    })
})

(defun refresh-pack-data () {
    (master-can-read-all)
    (master-check-timeouts slave-timeout-ms)
    (check-can-health)
    (master-update-vesc-bms)

    (setq vt-vchg (master-get-vchg))
    (setq iout (master-get-current))
    (update-temp-globals)
    (scan-pack-from-slaves)
})

;;;;;;;;;; Shutdown and sleep ;;;;;;;;;;

(defun shutdown-reason-name (reason)
    (cond
        ((= reason shutdown-reason-timer) "timer")
        ((= reason shutdown-reason-low-soc-start) "low-soc-start")
        ((= reason shutdown-reason-low-soc-main) "low-soc-main")
        ((= reason shutdown-reason-app) "app")
        (true "unknown")
))

(defun shutdown-reason-beep (reason) {
    (var count (if (> reason 0) reason 5))
    (user-beep count 0.25)
    (sleep 0.8)
    (user-beep 3 0.06)
})

(defun fail-close-outputs (clear-bal-trigger) {
    (var local-ok false)
    (match (trap (master-fail-close-local))
        ((exit-ok _) (setq local-ok true))
        (_ (match (trap (gpio-write 5 0))
            ((exit-ok _) (setq local-ok true))
            (_ nil)
        ))
    )

    (setq is-charging false)
    (setq charge-ok false)
    (if clear-bal-trigger (clear-balance-request))
    (var bal-close-ok (stop-all-balancing))
    (var close-ok (and local-ok bal-close-ok))

    (if close-ok {
        (if fail-close-failed (print "BMS fail-close recovered"))
        (setq fail-close-failed false)
    } {
        (if (not fail-close-failed) (print "BMS fail-close failed"))
        (setq fail-close-failed true)
    })

    close-ok
})

(defun bms-shutdown-impl (reason) {
    (print "BMS shutdown sequence starting")
    (print "Shutdown reason:" (shutdown-reason-name reason))
    (set-bms-val 'bms-status (str-merge "SHUTDOWN_" (shutdown-reason-name reason)))
    (send-slave-beep 0x04)
    (fail-close-outputs true)
    (shutdown-reason-beep reason)
    (save-settings)
    (master-shutdown)

    ; If the hardware did not remove power, stay fail-closed and keep warning.
    (sleep 2.0)
    (loopwhile t {
        (fail-close-outputs true)
        (beep 10 0.05)
        (sleep 1.0)
    })
})

(defun bms-shutdown-low-soc-start () (bms-shutdown-impl shutdown-reason-low-soc-start))
(defun bms-shutdown-low-soc-main () (bms-shutdown-impl shutdown-reason-low-soc-main))
(defun bms-shutdown-app () (bms-shutdown-impl shutdown-reason-app))
(defun shutdown-master () (bms-shutdown-app))

(defun low-soc-unused () (and
    (valid-pack-reading)
    (< soc 0.05)
    (not trigger-bal-after-charge)
    (< vt-vchg (bms-get-param 'v_charge_detect))
    (external-wake-inactive)
    (not (is-connected))
    (not (can-active))
))

(defun reset-sleep-total-time () {
    (setassoc rtc-val 'sleep-total-time-s 0)
    (save-rtc-val)
})

(defun process-sleep-time () {
    (var source (master-wakeup-source))

    (cond
        ; External enable use resets the shutdown-days timer.
        ((= source 1) {
            (setassoc rtc-val 'sleep-total-time-s 0)
        })
        ; Count actual timer sleep, not the requested duration.
        ((= source 2) {
            (var entered (rtc-number 'sleep-enter-time-s 0))
            (var now (master-get-time-of-day-s))
            (if (and entered (> entered 0) (> now entered)) {
                (setassoc rtc-val 'sleep-total-time-s
                    (+ (rtc-number 'sleep-total-time-s 0) (- now entered)))
            })
        })
    )

    (setassoc rtc-val 'sleep-enter-time-s 0)

    (if (or
            (external-wake-active)
            (> vt-vchg (bms-get-param 'v_charge_detect))
            (is-comm-connected)
            (can-active)
        ) {
        (setassoc rtc-val 'sleep-total-time-s 0)
        (save-rtc-val)
    } {
        (save-rtc-val)
        (if (and
                (charger-data-ok)
                (> (bms-get-param 'shutdown) 0)
                (>= (rtc-number 'sleep-total-time-s 0) (* (bms-get-param 'shutdown) 86400))
            )
            (bms-shutdown-impl shutdown-reason-timer)
        )
    })
})

(defun update-sleep-shutdown-timer () {
    ; Any real external use resets the shutdown-days counter.
    (if (or
            (external-wake-active)
            (> vt-vchg (bms-get-param 'v_charge_detect))
            (is-comm-connected)
            (can-active)
        ) {
        (if (> (rtc-number 'sleep-total-time-s 0) 0)
            (reset-sleep-total-time)
        )
    } {
        (if (and
                (charger-data-ok)
                (> (bms-get-param 'shutdown) 0)
                (>= (rtc-number 'sleep-total-time-s 0) (* (bms-get-param 'shutdown) 86400))
            )
            (bms-shutdown-impl shutdown-reason-timer)
        )
    })
})

(defun sleep-allowed-now () (and
    (> i-zero-time 1.0)
    (current-data-ok)
    (charger-data-ok)
    (external-wake-inactive)
    (not is-charging)
    (not trigger-bal-after-charge)
    (not bal-request)
    (= (ix bal-state 0) 0)
    (not (test-chg 1))
    (not (is-connected))
    (not (can-active))
))

(defun enter-master-sleep () {
    ; Debounce and re-check all asynchronous wake/connection conditions.
    (sleep 0.1)
    (if (sleep-allowed-now) {
        (print "Entering BMS sleep")
        (var dur (sleep-duration-s))
        (set-bms-val 'bms-status "SLEEP")

        (if (fail-close-outputs false) {
            (save-settings)
            (setassoc rtc-val 'sleep-enter-time-s (master-get-time-of-day-s))
            (save-rtc-val)

            (gpio-write 6 1) ; COM off, active low.
            (gpio-hold 6 1)
            (gpio-hold-deepsleep 1)
            (prepare-external-wakeup)
            (sleep 0.05)

            ; Do not enter deep sleep if ENABLE changed during preparation.
            (if (external-wake-active) {
                (gpio-hold-deepsleep 0)
                (gpio-hold 6 0)
                (gpio-write 6 0)
                (setassoc rtc-val 'sleep-enter-time-s 0)
                (save-rtc-val)
                (print "Sleep cancelled by ENABLE")
            } {
                (sleep-deep dur)
            })
        } {
            (print "Sleep deferred because fail-close did not complete")
        })
    })
})

;;;;;;;;;; Charge and counter control ;;;;;;;;;;

(defun update-soc-and-counters (dt) {
    (var batt-ah (bms-get-param 'batt_ah))

    (if (and pack-data-ok (< ah-cnt-soc 0.0))
        (setq ah-cnt-soc (* (calc-soc c-min) batt-ah))
    )

    (if (and pack-data-ok (> batt-ah 0.0)) {
        (var ah (* iout (/ dt 3600.0)))
        (setq ah-cnt-soc (truncate (- ah-cnt-soc ah) 0.0 batt-ah))

        (if (= (bms-get-param 'soc_use_ah) 1) {
            (setq soc (/ ah-cnt-soc batt-ah))
        } {
            (if (>= soc 0.0)
                (setq soc (lpf soc (calc-soc c-min) (* 100.0 (bms-get-param 'soc_filter_const))))
                (setq soc (calc-soc c-min))
            )
        })

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
    })

    (set-bms-val 'bms-soc (if (>= soc 0.0) soc 0.0))
    (set-bms-val 'bms-soh 1.0)
    (set-bms-val 'bms-ah-cnt ah-cnt)
    (set-bms-val 'bms-wh-cnt wh-cnt)
    (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
    (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
    (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
    (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)
})

(defun charge-temp-ok () (or
    (= (param-or 't_charge_mon_en 1) 0)
    (and
        temp-data-ok
        (< t-mos (bms-get-param 't_charge_max_mos))
        (or
            (not cell-temp-mon-en)
            (and
                (< t-max (bms-get-param 't_charge_max))
                (> t-min (bms-get-param 't_charge_min))
            )
        )
    )
))

(defun update-charge-control () {
    (if (and is-charging (>= c-max (bms-get-param 'vc_charge_end))) {
        ; Latch completion until charger disconnect. This avoids charge cycling
        ; when unloaded cell voltage relaxes below vc_charge_start.
        (setq charge-complete true)
        (setq charge-complete-msg true)
        (setq trigger-bal-after-charge true)
        (send-slave-beep 0x03)
    })

    (setq charge-ok (and
        pack-data-ok
        (current-data-ok)
        (charger-data-ok)
        (< c-max (if is-charging
            (bms-get-param 'vc_charge_end)
            (bms-get-param 'vc_charge_start)
        ))
        (> c-min (bms-get-param 'vc_charge_min))
        (charge-temp-ok)
        chg-allowed
        (not (assoc rtc-val 'charge-fault))
        (not charge-complete)
    ))

    ; If charging is enabled and maximum charge current is exceeded, latch a
    ; charge fault until the charger has been removed for several seconds.
    (if (and is-charging (> (- iout) (bms-get-param 'max_charge_current))) {
        (setq charge-ok false)
        (setassoc rtc-val 'charge-fault true)
        (save-rtc-val)
    })

    (if (and (assoc rtc-val 'charge-fault) (> (secs-since charge-dis-ts) 5.0)) {
        (setassoc rtc-val 'charge-fault false)
        (save-rtc-val)
    })

    (if (and charge-complete (> (secs-since charge-dis-ts) 5.0)) {
        (setq charge-complete false)
        (setq charge-complete-msg false)
    })

    (var charger-detected (test-chg 1))
    (if (and charger-detected (not charger-detected-prev))
        (setq charge-ts (systime))
    )
    (setq charger-detected-prev charger-detected)

    (if (and charger-detected charge-ok) {
        (if (< (secs-since charge-ts) charger-max-delay)
            (set-chg true)
            (set-chg (> (- iout) (bms-get-param 'min_charge_current)))
        )
    } {
        (set-chg nil)

        ; Reset coulomb SOC when battery is full.
        (if (and pack-data-ok (>= c-max (bms-get-param 'vc_charge_start))) {
            (setq ah-cnt-soc (bms-get-param 'batt_ah))
            (setq trigger-bal-after-charge true)
        })
    })

    (setq chg-status
        (cond
            ((assoc rtc-val 'charge-fault) {
                (setq charge-complete-msg false)
                "FLT_CHG_OC"
            })
            (charge-complete-msg "CHG_DONE")
            (is-charging {
                (setq charge-complete-msg false)
                "CHARGING"
            })
            (true "")
        )
    )

    (set-bms-val 'bms-chg-allowed (bool-int chg-allowed))
})

;;;;;;;;;; Balancing ;;;;;;;;;;

; Pick non-adjacent cells from one BQ76952 group. Cells are split into even and
; odd local indexes, then the stronger group is selected.
(defun balance-ic-group (voltages c-min threshold max-ch) {
    (var n (length voltages))

    (if (= n 0) 0 {
        (var even-grp '())
        (var odd-grp '())

        (looprange i 0 n {
            (var v (ix voltages i))
            (if (> (- v c-min) threshold) {
                (if (= (mod i 2) 0)
                    (setq even-grp (cons (cons i v) even-grp))
                    (setq odd-grp (cons (cons i v) odd-grp))
                )
            })
        })

        (var even-sorted (sort (fn (a b) (> (cdr a) (cdr b))) even-grp))
        (var odd-sorted (sort (fn (a b) (> (cdr a) (cdr b))) odd-grp))
        (var use-even true)

        (if (> (length odd-sorted) (length even-sorted))
            (setq use-even false)
        )

        (if (= (length odd-sorted) (length even-sorted)) {
            (var even-sum 0.0)
            (loopforeach p even-sorted
                (setq even-sum (+ even-sum (- (cdr p) c-min))))

            (var odd-sum 0.0)
            (loopforeach p odd-sorted
                (setq odd-sum (+ odd-sum (- (cdr p) c-min))))

            (if (> odd-sum even-sum) (setq use-even false))
        })

        (var grp (if use-even even-sorted odd-sorted))
        (var mask 0)
        (var cnt 0)

        (loopforeach c grp {
            (if (>= cnt max-ch) (break))
            (setq mask (+ mask (shl 1 (car c))))
            (setq cnt (+ cnt 1))
        })

        mask
    })
})

(defun mask-to-bin (mask n) {
    (var s "")
    (looprange i 0 n {
        (setq s (str-merge s (if (> (bitwise-and mask (shl 1 i)) 0) "1" "0")))
    })
    s
})

(defun clear-cached-balancing () {
    (looprange i 0 8 {
        (setix slave-bal-mask-ic1 i 0)
        (setix slave-bal-mask-ic2 i 0)
    })
})

(defun send-cached-balance-masks (beep-code) {
    (var sid 1)
    (var max-sid (cfg-num-slaves))
    (var all-ok true)

    (loopwhile (<= sid max-sid) {
        (if (master-slave-active? sid) {
            (if (not (master-send-balance
                    sid
                    (ix slave-bal-mask-ic1 (- sid 1))
                    (ix slave-bal-mask-ic2 (- sid 1))
                    beep-code))
                (setq all-ok false)
            )
        })
        (setq sid (+ sid 1))
    })

    all-ok
})

(defun send-zero-balance-all () {
    (var all-ok true)

    ; Send several copies and include temporarily inactive configured slaves.
    ; Their local 10-second watchdog is the final fallback if CAN is unavailable.
    (looprange attempt 0 3 {
        (looprange sid 1 (+ (cfg-num-slaves) 1) {
            (if (not (master-send-balance sid 0 0 0))
                (setq all-ok false)
            )
        })
        (sleep 0.02)
    })

    all-ok
})

(defun stop-all-balancing () {
    (clear-cached-balancing)
    (var bal-off-ok (send-zero-balance-all))
    (setix bal-keepalive-kick 0 0)
    (setix bal-state 0 0)
    (setq bal-status "")
    (setq bal-off-failed (not bal-off-ok))
    bal-off-ok
})

(defun balance-safe-now () (and
    pack-data-ok
    temp-data-ok
    (current-data-ok)
    (not is-charging)
    (<= (* (abs iout) (if (= (ix bal-state 0) 1) 0.8 1.0)) (bms-get-param 'balance_max_current))
    (>= c-min (bms-get-param 'vc_balance_min))
    (or
        (not cell-temp-mon-en)
        (<= t-max (bms-get-param 't_bal_max_cell))
    )
    (<= t-ic (bms-get-param 't_bal_max_ic))
))

(defun all-configured-slaves-fresh () {
    (var all-fresh true)
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (not (and (master-slave-active? sid) (master-slave-fresh? sid)))
            (setq all-fresh false))
        (setq sid (+ sid 1))
    })

    all-fresh
})

(defun all-configured-slaves-settled () {
    (var all-settled true)
    (var active-count 0)
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (and (master-slave-active? sid) (master-slave-fresh? sid)) {
            (setq active-count (+ active-count 1))
            (if (not (master-get-slave-settled? sid))
                (setq all-settled false))
        } {
            (setq all-settled false)
        })
        (setq sid (+ sid 1))
    })

    (and (> active-count 0) all-settled)
})

(defun balance-needed-now () {
    (var max-ch (bms-get-param 'max_bal_ch))
    (var threshold (bms-get-param 'vc_balance_start))
    (var needed false)
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (and (master-slave-active? sid) (master-slave-fresh? sid)) {
            (var cells (master-get-slave-cells sid))
            (var ic1-cnt (master-get-cells-ic1 sid))
            (var ic2-cnt (master-get-cells-ic2 sid))
            (var cnt (+ ic1-cnt ic2-cnt))

            (if (and cells (> cnt 0) (= (length cells) cnt)) {
                (var ic1-volts (map (fn (i) (ix cells i)) (range ic1-cnt)))
                (var ic2-volts (if (> ic2-cnt 0)
                    (map (fn (i) (ix cells (+ ic1-cnt i))) (range ic2-cnt))
                    '()
                ))
                (var ic1-mask (balance-ic-group ic1-volts c-min threshold max-ch))
                (var ic2-mask (if (> ic2-cnt 0)
                    (balance-ic-group ic2-volts c-min threshold max-ch)
                    0
                ))

                (if (or (> ic1-mask 0) (> ic2-mask 0))
                    (setq needed true)
                )
            })
        })
        (setq sid (+ sid 1))
    })

    needed
})

(defun start-balance-request () {
    (setq bal-request true)
    (setix bal-keepalive-kick 0 1)
})

(defun try-manual-balance-request () {
    (refresh-pack-data)
    (if (and
            (balance-safe-now)
            (all-configured-slaves-fresh)
            (all-configured-slaves-settled)
            (balance-needed-now)
        ) {
        (start-balance-request)
        true
    } {
        false
    })
})

(defun clear-balance-request () {
    (setq bal-request false)
    (setq trigger-bal-after-charge false)
})

(defun defer-auto-balance-request () {
    (setq bal-request false)
    (setq bal-auto-retry-ts (systime))
})

(defun fail-close-active-balance () {
    (if (= (ix bal-state 0) 1) {
        (master-fail-close-local)
        (setq is-charging false)
        (setq charge-ok false)
    })
})

(defun balance-thd () (loopwhile t {
    (if (and
            trigger-bal-after-charge
            (not is-charging)
            (> (secs-since bal-auto-retry-ts) 5.0)
        )
        (start-balance-request)
    )

    (if (not bal-request) {
        (if (= (ix bal-state 0) 1) {
            (print "BAL: stopped")
            (stop-all-balancing)
        })
        (sleep 0.2)
    } {
        (refresh-pack-data)

        (if (not (balance-safe-now)) {
            (print "BAL: blocked by pack conditions")
            (fail-close-active-balance)
            (stop-all-balancing)
            (if trigger-bal-after-charge
                (defer-auto-balance-request)
                (clear-balance-request)
            )
        })

        (if bal-request {
            ; Phase 1: stop balancing and wait only while slaves report
            ; unsettled data. If they are already settled, respond immediately.
            (clear-cached-balancing)
            (send-cached-balance-masks 0)
            (master-can-read-all)
            (master-check-timeouts slave-timeout-ms)

            (var settle-wait 0)
            (var max-settle-wait 50) ; 50 x 100ms = 5s timeout
            (var settled-ready (all-configured-slaves-settled))

            (loopwhile (and bal-request (not settled-ready) (< settle-wait max-settle-wait)) {
                (sleep 0.1)
                (master-can-read-all)
                (master-check-timeouts slave-timeout-ms)
                (setq settle-wait (+ settle-wait 1))

                (if (all-configured-slaves-settled) {
                    (setq settled-ready true)
                    (break)
                })
            })

            (if (not settled-ready) {
                (print "BAL: stopped (slaves not settled)")
                (stop-all-balancing)
                (if trigger-bal-after-charge
                    (defer-auto-balance-request)
                    (clear-balance-request)
                )
            })
        })

        (if bal-request {
            ; Phase 2: compute balance masks from settled slave cell voltages.
            (refresh-pack-data)

            (if (not (and (balance-safe-now) (all-configured-slaves-fresh))) {
                (print "BAL: stopped (conditions changed)")
                (fail-close-active-balance)
                (stop-all-balancing)
                (if trigger-bal-after-charge
                    (defer-auto-balance-request)
                    (clear-balance-request)
                )
            })
        })

        (if bal-request {
            (var max-ch (bms-get-param 'max_bal_ch))
            (var threshold (if (= (ix bal-state 0) 1)
                (bms-get-param 'vc_balance_end)
                (bms-get-param 'vc_balance_start)
            ))
            (var any-bal false)
            (var sid 1)
            (var max-sid (cfg-num-slaves))

            (clear-cached-balancing)

            (loopwhile (<= sid max-sid) {
                (if (and (master-slave-active? sid) (master-slave-fresh? sid)) {
                    (var cells (master-get-slave-cells sid))
                    (var ic1-cnt (master-get-cells-ic1 sid))
                    (var ic2-cnt (master-get-cells-ic2 sid))
                    (var cnt (+ ic1-cnt ic2-cnt))

                    (if (and cells (> cnt 0) (= (length cells) cnt)) {
                        (var ic1-volts (map (fn (i) (ix cells i)) (range ic1-cnt)))
                        (var ic2-volts (if (> ic2-cnt 0)
                            (map (fn (i) (ix cells (+ ic1-cnt i))) (range ic2-cnt))
                            '()
                        ))
                        (var ic1-mask (balance-ic-group ic1-volts c-min threshold max-ch))
                        (var ic2-mask (if (> ic2-cnt 0)
                            (balance-ic-group ic2-volts c-min threshold max-ch)
                            0
                        ))

                        (if (or (> ic1-mask 0) (> ic2-mask 0)) {
                            (setq any-bal true)
                            (print (str-merge "BAL S" (str-from-n sid "%d")
                                " IC1:" (mask-to-bin ic1-mask ic1-cnt)
                                " IC2:" (mask-to-bin ic2-mask ic2-cnt)
                                " min=" (str-from-n c-min "%.3f")))
                        })

                        (setix slave-bal-mask-ic1 (- sid 1) ic1-mask)
                        (setix slave-bal-mask-ic2 (- sid 1) ic2-mask)
                    })
                })
                (setq sid (+ sid 1))
            })

            (if any-bal {
                (setix bal-state 0 1)
                (setq bal-status "BAL")
                (setix bal-keepalive-kick 0 1)

                ; Phase 3: hold for about 30 s. Main loop sends keepalive.
                (var hold-cnt 0)
                (loopwhile (and bal-request (< hold-cnt 30)) {
                    (sleep 1.0)
                    (setq hold-cnt (+ hold-cnt 1))
                })
            } {
                (print "BAL: target reached")
                (stop-all-balancing)
                (clear-balance-request)
            })
        })
    })

    (if (and (not bal-request) (= (ix bal-state 0) 1))
        (stop-all-balancing)
    )
}))

;;;;;;;;;; Events and status ;;;;;;;;;;

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-force-bal (? v)) {
                (if (= v 1) {
                    (if (try-manual-balance-request)
                        (print "BAL CMD: start")
                        (print "BAL CMD: ignored")
                    )
                } {
                    (print "BAL CMD: stop")
                    (clear-balance-request)
                    (stop-all-balancing)
                })
            })
            ((event-bms-chg-allow (? allow)) {
                (setq chg-allowed (= allow 1))
                (if (not chg-allowed) (set-chg nil))
                (set-bms-val 'bms-chg-allowed (bool-int chg-allowed))
                (print (str-merge "CHG: " (if chg-allowed "allowed" "blocked")))
            })
            ((event-bms-reset-cnt (? ah) (? wh)) {
                (if (= ah 1) {
                    (setq ah-cnt 0.0)
                    (setq ah-chg-tot 0.0)
                    (setq ah-dis-tot 0.0)
                    (set-bms-val 'bms-ah-cnt 0.0)
                    (set-bms-val 'bms-ah-cnt-chg-total 0.0)
                    (set-bms-val 'bms-ah-cnt-dis-total 0.0)
                })
                (if (= wh 1) {
                    (setq wh-cnt 0.0)
                    (setq wh-chg-tot 0.0)
                    (setq wh-dis-tot 0.0)
                    (set-bms-val 'bms-wh-cnt 0.0)
                    (set-bms-val 'bms-wh-cnt-chg-total 0.0)
                    (set-bms-val 'bms-wh-cnt-dis-total 0.0)
                })
            })
            (event-bms-zero-ofs {
                (print "CAL: zero current")
                (master-calibrate-current)
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
        (_ (print "Ignoring unsupported APPUI command"))
))

(defun update-status () {
    (var s "")

    (setq s (status-append s chg-status))
    (setq s (status-append s bal-status))
    (setq s (status-append s pack-status))
    (if (not (master-local-sensors-valid?))
        (setq s (status-append s "ADC_FAULT"))
    )
    (if bal-off-failed
        (setq s (status-append s "BAL_OFF_FAIL"))
    )
    (if fail-close-failed
        (setq s (status-append s "FAIL_CLOSE_FAIL"))
    )

    (if (and chg-allowed (not charge-ok) (not is-charging) (test-chg 1) (= (str-len chg-status) 0))
        (setq s (status-append s "CHG_BLOCK"))
    )

    (set-bms-val 'bms-status s)
})

(defun update-slave-presence () {
    (var id 1)
    (var max-id (cfg-num-slaves))

    (loopwhile (<= id max-id) {
        (var active (if (master-slave-active? id) 1 0))
        (var prev (ix prev-active (- id 1)))

        (if (and (= active 1) (not-eq prev 1))
            (print (str-merge "Slave " (str-from-n id "%d") " connected"))
        )

        (if (and (= active 0) (= prev 1)) {
            (print (str-merge "Slave " (str-from-n id "%d") " disconnected"))
            (set-chg nil)
            (stop-all-balancing)
            (send-slave-beep 0x04)
        })

        (setix prev-active (- id 1) active)
        (setq id (+ id 1))
    })
})

(defun event-supervisor () {
    (loopwhile t {
        (match (trap (event-handler))
            ((exit-ok _) nil)
            (_ {
                (print "Event handler crashed, restarting")
                (fail-close-outputs true)
            })
        )
        (sleep 0.2)
    })
})

(defun balance-supervisor () {
    (loopwhile t {
        (match (trap (balance-thd))
            ((exit-ok _) nil)
            (_ {
                (print "Balance controller crashed, restarting fail-closed")
                (fail-close-outputs true)
            })
        )
        (sleep 0.2)
    })
})

(defun fail-close-retry-thd () {
    (loopwhile t {
        (if (or fail-close-failed bal-off-failed)
            (fail-close-outputs true)
        )
        (sleep 0.5)
    })
})

(defun sleep-unblock-thd () {
    (loopwhile t {
        (var sleep-unblock-ok (fn () (and
            (= (bms-get-param 'block_sleep) 1)
            pack-data-ok
            (< (- c-max c-min) 0.05)
            (> c-min 2.4)
            (> (secs-since 0) 3600)
            sleep-unblock-en
        )))

        (var should-unblock true)
        (looprange i 0 60 {
            (if (not (sleep-unblock-ok))
                (setq should-unblock false)
            )
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

;;;;;;;;;; Main loop ;;;;;;;;;;

(defun main-control-step () {
    ; Drain CAN at 20 Hz.
    (master-can-read-all)

    ; Track balance mask changes from slaves quickly.
    (var sid-fast 1)
    (loopwhile (<= sid-fast (cfg-num-slaves)) {
        (if (master-slave-active? sid-fast) {
            (var status (master-get-slave-status sid-fast))
            (if status {
                (var cur-mask (car status))
                (var prev-mask (ix prev-bal-mask (- sid-fast 1)))
                (if (not-eq cur-mask prev-mask)
                    (setix prev-bal-mask (- sid-fast 1) cur-mask)
                )
            })
        })
        (setq sid-fast (+ sid-fast 1))
    })

    ; 10 Hz control and display work.
    (if (= (mod loop-cnt 2) 0) {
        (var dt (secs-since t-last))
        (setq t-last (systime))

        (refresh-pack-data)

        (if pack-data-ok
            (update-soc-and-counters dt)
        )

        (update-charge-control)
        (update-sleep-shutdown-timer)

        ; Balance keepalive to slaves at 1 Hz while balancing is active.
        (if (and bal-request
                 (= (ix bal-state 0) 1)
                 (or (= (mod loop-cnt 20) 0) (= (ix bal-keepalive-kick 0) 1))) {
            (if (not (send-cached-balance-masks 0)) {
                (print "BAL: keepalive transmit failed")
                (clear-balance-request)
                (stop-all-balancing)
            })
            (setix bal-keepalive-kick 0 0)
        })

        (update-status)
        (send-bms-can)
        (update-slave-presence)

        ; Measure idle time for sleep and shutdown decisions.
        (if (or
                (not (current-data-ok))
                (> (abs iout) (bms-get-param 'min_current_sleep))
            )
            (setq i-zero-time 0.0)
            (setq i-zero-time (+ i-zero-time dt))
        )

        ; Set SOC to 0 below empty voltage and not under load.
        (if (and pack-data-ok (> i-zero-time 10.0) (<= c-min (bms-get-param 'vc_empty)))
            (setq ah-cnt-soc 0.0)
        )

        (if (and (low-soc-unused) (> i-zero-time 1.0) (<= c-min (bms-get-param 'vc_empty)))
            (bms-shutdown-low-soc-main)
        )

        ; The master has no local BQ to put to sleep; slaves handle their own
        ; BQ state and the master only drops COM/ESP.
        (if (sleep-allowed-now)
            (enter-master-sleep)
        )
    })

    (setq loop-cnt (+ loop-cnt 1))
})

(defun main () {
    (print "=== JFBMS Master ===")

    ; Reset values that must be relative to this boot, not image creation.
    (setq bal-auto-retry-ts (systime))
    (setq charge-dis-ts (systime))
    (setq charge-ts (systime))
    (setq t-last (systime))
    (setq loop-cnt 0)

    (if (> app-wdt-timeout 0)
        (wdt-configure true app-wdt-timeout)
        (wdt-disable)
    )

    ; COM enable low (active), charge off.
    (gpio-hold-deepsleep 0)
    (gpio-hold 6 0)
    (gpio-write 6 0)
    (gpio-write 5 0)

    ; Buzzer on GPIO8.
    (pwm-start 4000 0.0 0 8)

    (load-rtc-val)
    (master-reset-slaves)

    (event-register-handler (spawn 200 event-supervisor))
    (event-enable 'event-bms-force-bal)
    (event-enable 'event-bms-chg-allow)
    (event-enable 'event-bms-reset-cnt)
    (event-enable 'event-bms-zero-ofs)
    (event-enable 'event-data-rx)

    (print "Waiting for slave CAN data...")
    (looprange i 0 30 {
        (refresh-pack-data)
        (if pack-data-ok (break))
        (sleep 0.1)
    })

    (setq init-done true)
    (load-settings)
    (process-sleep-time)

    (if pack-data-ok
        (print (str-merge "Pack ready: slaves=" (str-from-n (cfg-num-slaves) "%d")
            " cells=" (str-from-n cell-num "%d")))
        {
            (print "No complete slave CAN pack data yet")
            (trap (can-debug))
        }
    )

    ; 2 beeps = initialization complete.
    (user-beep 2 0.1)

    (spawn 200 balance-supervisor)
    (spawn 100 fail-close-retry-thd)
    (spawn 100 sleep-unblock-thd)

    (loopwhile t {
        (match (trap (main-control-step))
            ((exit-ok _) nil)
            (_ {
                (setq control-crash-count (+ control-crash-count 1))
                (print "Main controller crashed, retrying fail-closed")
                (fail-close-outputs true)
                (set-bms-val 'bms-status "CONTROL_FAULT")
                (sleep 0.5)
            })
        )

        (wdt-reset)
        (sleep 0.05)
    })
})

@const-end

(image-save)
(main)

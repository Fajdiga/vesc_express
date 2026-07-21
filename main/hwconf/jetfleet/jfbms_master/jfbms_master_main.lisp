; JFBMS Master
; Pack controller for JFBMS slave boards. The master owns charge, sleep,
; shutdown, counters, status and balance decisions. Cell voltages and cell
; temperatures are taken only from slave CAN data.

;;;;;;;;;; User settings ;;;;;;;;;;

(def app-wdt-timeout 10)     ; Recover a wedged controller quickly; C cuts charge in 300 ms.
(def user-beeps-en true)
(def beep-duty-normal 0.5)
(def sleep-unblock-en true)
(def balance-active-time-s 30.0)
(def balance-keepalive-period-s 1.0)
(def control-max-qualify-dt 0.25)
(def soc-checkpoint-min-time-s 10.0)
(def soc-checkpoint-max-time-s 300.0)
(def soc-checkpoint-delta 0.02)

;;;;;;;;;; State ;;;;;;;;;;

; Operational pack data may tolerate BQ/CAN jitter while balancing. The
; independent C charge watchdog still cuts CHG_EN after 300 ms without a fresh
; complete safe snapshot.
(def slave-control-timeout-ms 1000)
(def chg-allowed true)
(def charge-ok false)
(def is-charging false)
(def charger-detected-prev false)
(def charge-block-beeped false)
(def charge-enable-beeped false)
(def trigger-bal-after-charge false)
(def charge-complete false)
(def charge-dis-ts (systime))
(def last-fast-trip-count -1)

(def bal-auto-retry-ts (systime))
(def balance-active-start-ts (systime))
(def balance-cycle-threshold 0.0)
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
(def manual-bal-active false)
(def calibration-running false)
(def current-zero-ready false)

(def ah-cnt 0.0)
(def wh-cnt 0.0)
(def ah-chg-tot 0.0)
(def wh-chg-tot 0.0)
(def ah-dis-tot 0.0)
(def wh-dis-tot 0.0)
(def ah-cnt-soc -1.0)
(def soc-checkpoint-ah -1.0)
(def soc-checkpoint-ts (systime))
(def primary-can-status-ts (systime))

(def t-last (systime))
(def rtc-val '(
    (charge-fault . false)
    (charge-complete . false)
    (short-count . 0)
    (short-service . false)
    (sleep-enter-time-s . 0)
    (sleep-total-time-s . 0)
))
(def rtc-val-magic 127)

(def prev-active (list 0 0 0 0 0 0 0 0))
(def prev-bal-mask (list 0 0 0 0 0 0 0 0))
(def prev-can-overflow 0)

; Balancing state. Only IDLE may permit CHG_EN to rise.
(def bal-state-idle 0)
(def bal-state-requested 1)
(def bal-state-active 2)
(def bal-state-stopping 3)
(def bal-state (list bal-state-idle))
(def slave-bal-mask-ic1 (list 0 0 0 0 0 0 0 0))
(def slave-bal-mask-ic2 (list 0 0 0 0 0 0 0 0))

(def shutdown-reason-unknown 0)
(def shutdown-reason-timer 1)
(def shutdown-reason-low-soc-start 2)
(def shutdown-reason-low-soc-main 3)
(def shutdown-reason-app 4)

(def loop-cnt 0)
(def buz-mutex (mutex-create))
(def pack-refresh-mutex (mutex-create))

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

; The primary VESC CAN connector is optional. A zero status rate is standalone
; mode and must result in no transmit attempts: an empty acknowledged CAN bus
; otherwise enters bus-off recovery and can stall the controller repeatedly.
; TWAI1 slave traffic is independent and remains enabled.
(defun update-primary-can-status () {
    (var rate (truncate (param-or 'can_status_rate_hz 0) 0 200))
    (if (and
            (> rate 0)
            (>= (secs-since primary-can-status-ts) (/ 1.0 rate))
        ) {
        (setq primary-can-status-ts (systime))
        (send-bms-can)
    })
})

(defun bool-int (v) (if v 1 0))

(defun balance-state-is (state) (= (ix bal-state 0) state))

; Hardware-owned fast overcurrent status. Missing or malformed extensions fail
; closed: charging remains disabled until the ADC monitor reports armed.
(defun fast-oc-status ()
    ; C returns: (latched armed trip-count last-raw current-a trip-time-s direction).
    ; A missing extension must fail closed without inventing a latched trip.
    (trap-value '(master-fast-oc-status) '(nil nil 0 0 0.0 0.0 0))
)

(defun fast-oc-latched () {
    (var status (fast-oc-status))
    (or
        (eq status nil)
        (< (length status) 2)
        (not (eq (ix status 0) nil))
    )
})

(defun fast-oc-armed () {
    (var status (fast-oc-status))
    (and
        status
        (>= (length status) 2)
        (not (eq (ix status 1) nil))
    )
})

(defun c-balance-inhibited ()
    (trap-value '(master-balance-inhibited?) false)
)

(defun charge-pack-fresh ()
    (trap-value '(master-pack-charge-fresh?) false)
)

(defun balance-in-progress () (or
    (not (balance-state-is bal-state-idle))
    (c-balance-inhibited)
))

; Keep the balance request visible in C before Lisp exposes REQUESTED state.
(defun set-c-balance-request (requested) {
    (trap-value (list 'master-balance-request (bool-int requested)) false)
})

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

(def settings-version-legacy 243i32)
(def settings-version 244i32)

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
    (setassoc rtc-val 'charge-fault (if (assoc rtc-val 'charge-fault) true false))
    (setassoc rtc-val 'charge-complete (if (assoc rtc-val 'charge-complete) true false))
    (setassoc rtc-val 'short-count (truncate (rtc-number 'short-count 0) 0 1000))
    (setassoc rtc-val 'short-service (if (assoc rtc-val 'short-service) true false))
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
    ; Counter layout is unchanged in 244. Promote 243 in place so the safety
    ; state-machine update does not erase customer Ah/Wh/SOC history.
    (var stored-version (read-setting 'ver-code))
    ; not-eq is used here because a fresh EEPROM can return nil for ver-code;
    ; the numeric = operator raises a type error when comparing nil to i32.
    (if (not-eq stored-version settings-version-legacy)
        (if (not-eq stored-version settings-version)
            (restore-settings)
        )
        (write-setting 'ver-code settings-version)
    )

    (setq ah-cnt (number-or (read-setting 'ah-cnt) 0.0))
    (setq wh-cnt (number-or (read-setting 'wh-cnt) 0.0))
    (setq ah-chg-tot (number-or (read-setting 'ah-chg-tot) 0.0))
    (setq wh-chg-tot (number-or (read-setting 'wh-chg-tot) 0.0))
    (setq ah-dis-tot (number-or (read-setting 'ah-dis-tot) 0.0))
    (setq wh-dis-tot (number-or (read-setting 'wh-dis-tot) 0.0))
    (setq ah-cnt-soc (number-or (read-setting 'ah-cnt-soc) -1.0))
    (setq soc-checkpoint-ah ah-cnt-soc)
    (setq soc-checkpoint-ts (systime))
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

; Persist SOC periodically without writing EEPROM at the 10 Hz control rate.
; Charge-complete and empty-voltage anchors call this with force=true.
(defun checkpoint-soc (force reason) {
    (var batt-ah (bms-get-param 'batt_ah))
    (var age (secs-since soc-checkpoint-ts))
    (var delta (if (and (> batt-ah 0.0) (>= soc-checkpoint-ah 0.0))
        (/ (abs (- ah-cnt-soc soc-checkpoint-ah)) batt-ah)
        1.0
    ))
    (var due (and
        (>= ah-cnt-soc 0.0)
        (or
            force
            (>= age soc-checkpoint-max-time-s)
            (and
                (>= age soc-checkpoint-min-time-s)
                (>= delta soc-checkpoint-delta)
            )
        )
    ))

    (if due {
        (write-setting 'ah-cnt-soc ah-cnt-soc)
        (setq soc-checkpoint-ah ah-cnt-soc)
        (setq soc-checkpoint-ts (systime))
        (if force (print (str-merge "SOC checkpoint: " reason)))
    })
    due
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

(defun charger-status () {
    ; C continuously debounces independent raw ADC frames. The returned list is
    ; (valid detected voltage sample-age-ms).
    (trap-value '(master-charger-status) '(nil nil 0.0 999999))
})

(defun test-chg (samples) {
    (var status (charger-status))
    (var detected (and
        status
        (>= (length status) 2)
        (not (eq (ix status 0) nil))
        (not (eq (ix status 1) nil))
    ))

    ; Keep JFBMS32 disconnect semantics: refresh for the whole detected period.
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
    (var requested (if chg true false))
    (var was-charging is-charging)
    (var allowed true)

    ; Charging has priority. A charge request first performs the checked
    ; three-pass zero-mask handoff; CHG_EN cannot rise while STOPPING fails.
    (if (and requested (fast-oc-latched))
        (setq allowed false)
    )
    ; Current is deliberately 0 A until the one-second zero capture completes.
    ; Never let a direct caller bypass that pre-charge step.
    (if (and requested (not current-zero-ready))
        (setq allowed false)
    )
    (if (and requested (balance-in-progress))
        (setq allowed (stop-all-balancing))
    )

    (var ok false)
    (if allowed {
        (match (trap (master-set-chg (bool-int requested)))
            ((exit-ok (? result)) (setq ok result))
            (_ (setq ok false))
        )
    })

    (if (and requested ok) {
        (setq is-charging true)
        (if (not was-charging) {
            (if (not charge-enable-beeped) {
                (setq charge-enable-beeped true)
                (spawn (fn () (user-beep 2 0.07)))
            })
        })
    } {
        (if requested (trap (master-set-chg 0)))
        (setq is-charging false)
    })

    ok
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
            ; Status field 4 is the two-bit external-sensor enable mask.
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
                (looprange i 0 expected {
                    (var is-ic (or (= i 0) (= i 2)))
                    (var is-cell (= (mod i 2) 1))
                    (var enabled (if (= i 1) bq1-cell-en bq2-cell-en))
                    (var required (or is-ic enabled))
                    (var temp (ix temps i))

                    (if (and is-cell enabled) (setq cell-temp-mon-en true))
                    (if (temp-valid temp) {
                        (if is-ic {
                            (if (> temp t-ic) (setq t-ic temp))
                        } {
                            (if (and is-cell enabled) {
                                (if (< temp t-min) (setq t-min temp))
                                (if (> temp t-max) (setq t-max temp))
                            })
                        })
                    } {
                        (if required (setq temps-ok false))
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
            (if (not (master-slave-fresh? sid))
                (setq stale-slave true))

            (var status (master-get-slave-status sid))
            (var faults (if status (ix status 1) 0))
            (var s-ic1 (master-get-cells-ic1 sid))
            (var s-ic2 (master-get-cells-ic2 sid))
            (var cnt (+ s-ic1 s-ic2))
            (var cells (master-get-slave-cells sid))

            (if (> (bitwise-and faults 0x0B) 0)
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
        (not stale-slave)
        temp-data-ok
    ))

    ; The C master-update-vesc-bms extension is the single owner of standard
    ; VESC cell topology, voltages, totals, extrema, and temperatures. Lisp
    ; keeps its own scan values for control decisions but must not race the C
    ; publication used by VESC Tool and standard BMS CAN frames.

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
    ; Main and balance contexts share these globals. Serialize the full refresh
    ; so neither context can observe a half-updated pack generation.
    (mutex-lock pack-refresh-mutex)
    (var result (trap (progn
        (master-can-read-all)
        (master-check-timeouts slave-control-timeout-ms)
        (check-can-health)
        (master-update-vesc-bms)
        (setq vt-vchg (master-get-vchg))
        (setq iout (master-get-current))
        (update-temp-globals)
        (scan-pack-from-slaves)
        true
    )))
    (mutex-unlock pack-refresh-mutex)
    (match result
        ((exit-ok _) true)
        (_ {
            (setq pack-data-ok false)
            (setq slave-data-fresh false)
            false
        })
    )
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

(defun capture-current-zero () {
    (print "CAL: CHG_EN off, waiting 1 second for zero current")
    ; Calibration only needs the charger switch open. Do not enter the balance
    ; shutdown/CAN handoff here: that path can wait on a missing slave and has
    ; nothing to do with measuring the local current-sense zero.
    (master-set-chg 0)
    (setq is-charging false)
    (setq charge-ok false)
    ; Lisp sleep yields, so VESC Tool and CAN processing remain responsive.
    (sleep 1.0)
    (print "CAL: sampling current ADC")
    (master-calibrate-current)
})

(defun current-calibration-thd () {
    (setq current-zero-ready false)
    ; Trap the whole sequence so the running latch is always released after an
    ; extension or CAN failure; charging simply remains locked off.
    (var calibrated (trap-value '(capture-current-zero) false))
    (setq current-zero-ready calibrated)
    (setq calibration-running false)
    (if calibrated {
        (print "CAL: zero captured and stored")
        (spawn (fn () (user-beep 2 0.08)))
    } {
        (print "CAL: failed; charging remains off")
        (spawn (fn () (user-beep 1 0.35)))
    })
})

(defun start-current-calibration () {
    (if (not calibration-running) {
        ; Set the latch before spawning so repeated 10 Hz charge checks cannot
        ; queue several calibration contexts.
        (setq calibration-running true)
        (spawn 160 current-calibration-thd)
        true
    } false)
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
    (balance-state-is bal-state-idle)
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

(defun pack-generation () (trap-value '(master-get-pack-generation) 0))

(defun qualify-dt-valid (dt) (and
    (> dt 0.0)
    (<= dt control-max-qualify-dt)
))

(defun set-soc-value (new-soc source reason force-log) {
    (var bounded (truncate new-soc 0.0 1.0))
    (var previous soc)
    (var batt-ah (bms-get-param 'batt_ah))
    (var coulomb-candidate (if (> batt-ah 0.0)
        (truncate (/ ah-cnt-soc batt-ah) 0.0 1.0)
        0.0
    ))
    (var voltage-candidate (calc-soc c-min))

    (if (or
            force-log
            (< previous 0.0)
            (> (abs (- bounded previous)) 0.05)
        )
        (print (str-merge
            "SOC " source "/" reason
            " prev=" (str-from-n previous "%.3f")
            " new=" (str-from-n bounded "%.3f")
            " ah=" (str-from-n coulomb-candidate "%.3f")
            " volt=" (str-from-n voltage-candidate "%.3f")
            " I=" (str-from-n iout "%.2f")
            " min=" (str-from-n c-min "%.3f")
            " max=" (str-from-n c-max "%.3f")
            " gen=" (str-from-n (pack-generation) "%d")
        ))
    )

    (setq soc bounded)
    (set-bms-val 'bms-soc bounded)
})

(defun update-soc-and-counters (dt) {
    (var batt-ah (bms-get-param 'batt_ah))
    (var dt-ok (qualify-dt-valid dt))

    (if (and pack-data-ok (< ah-cnt-soc 0.0))
        (setq ah-cnt-soc (* (calc-soc c-min) batt-ah))
    )

    (if (and pack-data-ok (> batt-ah 0.0)) {
        ; Do not extrapolate current across a scheduler stall.
        (var integration-dt (if dt-ok dt 0.0))
        (var ah (* iout (/ integration-dt 3600.0)))
        (setq ah-cnt-soc (truncate (- ah-cnt-soc ah) 0.0 batt-ah))

        (var coulomb-soc (/ ah-cnt-soc batt-ah))
        (var voltage-soc (calc-soc c-min))

        (if (= (bms-get-param 'soc_use_ah) 1) {
            (set-soc-value coulomb-soc "COULOMB" "TRACK" false)
        } {
            (if (>= soc 0.0)
                (set-soc-value
                    (lpf soc voltage-soc
                        (truncate (* 100.0 (bms-get-param 'soc_filter_const)) 0.0 1.0))
                    "VOLTAGE" "TRACK" false)
                (set-soc-value voltage-soc "VOLTAGE" "INITIAL" true)
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

    (checkpoint-soc false "PERIODIC")
    (if (< soc 0.0) (set-bms-val 'bms-soc 0.0))
    (set-bms-val 'bms-soh 1.0)
    (set-bms-val 'bms-ah-cnt ah-cnt)
    (set-bms-val 'bms-wh-cnt wh-cnt)
    (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
    (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
    (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
    (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)
})

(defun fast-oc-direction () {
    (var status (fast-oc-status))
    (if (and status (>= (length status) 7)) (ix status 6) 0)
})

(defun record-fast-oc () {
    (var status (fast-oc-status))
    (if (and status (>= (length status) 3) (not (eq (ix status 0) nil))) {
        (var trips (ix status 2))
        (if (not-eq trips last-fast-trip-count) {
            (setq last-fast-trip-count trips)
            (var count (+ (rtc-number 'short-count 0) 1))
            (setassoc rtc-val 'short-count count)
            (if (>= count 3) (setassoc rtc-val 'short-service true))
            (save-rtc-val)
            (spawn (fn () (user-beep 1 0.45)))
        })
    })
})

(defun finish-charge (reason) {
    (if (not charge-complete) {
        (set-chg false)
        (setq charge-complete true)
        (setassoc rtc-val 'charge-complete true)
        (save-rtc-val)
        (setq ah-cnt-soc (bms-get-param 'batt_ah))
        (set-soc-value 1.0 "CHARGE" reason true)
        (checkpoint-soc true reason)
        (setq trigger-bal-after-charge true)
        (setq bal-auto-retry-ts (systime))
        (send-slave-beep 0x03)
        (spawn (fn () (user-beep 3 0.10)))
        (print (str-merge "CHG complete: " reason))
    })
})

(defun clear-session-after-disconnect () {
    (var changed false)
    (if (fast-oc-latched) (trap (master-clear-fast-oc)))
    (if (and (assoc rtc-val 'charge-fault) (not (assoc rtc-val 'short-service))) {
        (setassoc rtc-val 'charge-fault false)
        (setq changed true)
    })
    (if (assoc rtc-val 'charge-complete) {
        (setassoc rtc-val 'charge-complete false)
        (setq changed true)
    })
    ; A completed, fault-free charge proves the current path is healthy.
    (if (and charge-complete (> (rtc-number 'short-count 0) 0)
            (not (assoc rtc-val 'short-service))) {
        (setassoc rtc-val 'short-count 0)
        (setq changed true)
    })
    (if changed (save-rtc-val))

    (setq charge-complete false)
    (setq charge-block-beeped false)
    (setq charge-enable-beeped false)
})

(defun clear-service-faults () {
    (if (and
            (not (test-chg 1))
            (> (secs-since charge-dis-ts) 5.0)
            (current-data-ok)
            (< (abs iout) 1.0)
            (trap-value '(master-clear-fast-oc) false)
        ) {
        (setassoc rtc-val 'charge-fault false)
        (setassoc rtc-val 'short-count 0)
        (setassoc rtc-val 'short-service false)
        (save-rtc-val)
        (print "CHG faults cleared safely")
        true
    } {
        (print "CHG fault clear rejected: unplug charger for 5s and remove current")
        false
    })
})

(defun charge-block-reason (charger-detected) {
    (cond
        ((assoc rtc-val 'short-service) "FLT_SHORT_LOCK")
        ((fast-oc-latched) (if (> (fast-oc-direction) 0) "FLT_FAST_OC_REV" "FLT_FAST_OC_CHG"))
        ((not (fast-oc-armed)) "FLT_FAST_ADC")
        ((assoc rtc-val 'charge-fault) "FLT_CHG_OC")
        (charge-complete "CHG_COMPLETE")
        ((not chg-allowed) "CHG_DISABLED")
        ((not pack-data-ok) "WAIT_SLAVE")
        ((not slave-data-fresh) "CAN_STALE")
        ((and charger-detected (not (charge-pack-fresh))) "CAN_CHG_STALE")
        ; Calibration is charge-related status. Do not show CAL_ZERO while the
        ; charger is absent.
        ((not charger-detected) "")
        ((not current-zero-ready) "CAL_ZERO")
        ((not (current-data-ok)) "ADC_CURRENT")
        ((not (charger-data-ok)) "ADC_CHARGER")
        ((>= c-max (if is-charging
            (bms-get-param 'vc_charge_end)
            (bms-get-param 'vc_charge_start))) "CELL_HIGH")
        ((<= c-min (bms-get-param 'vc_charge_min)) "CELL_LOW")
        ((and (= (param-or 't_charge_mon_en 1) 1) (not temp-data-ok)) "TEMP_DATA")
        ((and (= (param-or 't_charge_mon_en 1) 1)
            (>= t-mos (bms-get-param 't_charge_max_mos))) "TEMP_MOS_HIGH")
        ((and (= (param-or 't_charge_mon_en 1) 1) cell-temp-mon-en
            (>= t-max (bms-get-param 't_charge_max))) "TEMP_CELL_HIGH")
        ((and (= (param-or 't_charge_mon_en 1) 1) cell-temp-mon-en
            (<= t-min (bms-get-param 't_charge_min))) "TEMP_CELL_LOW")
        (true "")
    )
})

(defun update-charge-control (dt) {
    (var charger-detected (test-chg 1))
    (var charge-current (- iout))

    (record-fast-oc)

    ; The first plug-in captures and stores zero. Later sessions reuse it.
    (if (and charger-detected (not charger-detected-prev)) {
        (setq charge-block-beeped false)
        (setq charge-enable-beeped false)
        (spawn (fn () (user-beep 1 0.08)))
        (print "CHG: charger detected")
    })
    (setq charger-detected-prev charger-detected)

    (if (and charger-detected (not current-zero-ready) (not calibration-running))
        (start-current-calibration)
    )

    ; Same simple slow-current guard as JFBMS32. The C fast comparator remains
    ; an independent backstop.
    (if (and is-charging (> charge-current (bms-get-param 'max_charge_current))) {
        (setq charge-ok false)
        (if (not (assoc rtc-val 'charge-fault)) {
            (setassoc rtc-val 'charge-fault true)
            (save-rtc-val)
        })
        (set-chg false)
    })

    ; Five seconds unplugged starts a completely new session.
    (if (and (charger-data-ok) (not charger-detected)) {
        (if (> (secs-since charge-dis-ts) 5.0) {
            (set-chg false)
            (clear-session-after-disconnect)
        })
    })

    ; Cell voltage is the charge-complete decision, as in JFBMS32.
    (if (and is-charging pack-data-ok
            (>= c-max (bms-get-param 'vc_charge_end)))
        (finish-charge "CELL_LIMIT")
    )

    (var block-reason (charge-block-reason charger-detected))
    (setq charge-ok (and charger-detected (= (str-len block-reason) 0)))

    (if charge-ok {
        (var balance-stopped (if (balance-in-progress)
            (stop-all-balancing)
            true
        ))
        (if balance-stopped {
            (set-chg true)
        } {
            (set-chg false)
        })
    } {
        (set-chg false)
    })

    (setq chg-status (if is-charging "CHARGING" block-reason))

    (if (and charger-detected (not is-charging)
            (> (str-len block-reason) 0) (not charge-block-beeped)) {
        (setq charge-block-beeped true)
        (spawn (fn () (user-beep 1 0.30)))
        (print (str-merge "CHG blocked: " block-reason))
    })

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

(defun any-cached-balancing () {
    (var any false)
    (looprange i 0 8 {
        (if (or (> (ix slave-bal-mask-ic1 i) 0)
                (> (ix slave-bal-mask-ic2 i) 0))
            (setq any true)
        )
    })
    any
})

(defun mask-bit-count (mask) {
    (var count 0)
    (looprange i 0 16 {
        (if (!= (bitwise-and mask (shl 1 i)) 0)
            (setq count (+ count 1))
        )
    })
    count
})

(defun manual-balance-cell (cell enable) {
    ; VESC Tool overrides should be immediate. Map the pack cell index to one
    ; slave/IC, update the cached mask, and send it once. The balance thread
    ; supplies the one-second keepalive and applies the same safety gate.
    (var sid 1)
    (var base 0)
    (var found false)
    (var target-sid 0)
    (var target-ic 0)
    (var target-bit 0)
    (var target-v 0.0)

    (loopwhile (and (<= sid (cfg-num-slaves)) (not found)) {
        (var ic1-cnt (master-get-cells-ic1 sid))
        (var ic2-cnt (master-get-cells-ic2 sid))
        (var count (+ ic1-cnt ic2-cnt))
        (if (and (>= cell base) (< cell (+ base count))) {
            (var local (- cell base))
            (var cells (master-get-slave-cells sid))
            (if (and cells (= (length cells) count)) {
                (setq target-sid sid)
                (setq target-v (ix cells local))
                (if (< local ic1-cnt) {
                    (setq target-ic 1)
                    (setq target-bit (shl 1 local))
                } {
                    (setq target-ic 2)
                    (setq target-bit (shl 1 (- local ic1-cnt)))
                })
                (setq found true)
            })
        })
        (setq base (+ base count))
        (setq sid (+ sid 1))
    })

    (if (or (not found) (and (> enable 0) (not (balance-safe-now)))) {
        ; A disable request is always safe. If topology disappeared, fail
        ; closed by clearing every slave instead of leaving an unknown mask on.
        (if (= enable 0) (stop-all-balancing))
        (print "BAL OVR blocked: invalid cell or unsafe pack state")
        false
    } {
        (if (not manual-bal-active) {
            (setq trigger-bal-after-charge false)
            (clear-cached-balancing)
        })
        (var index (- target-sid 1))
        (var old-mask (if (= target-ic 1)
            (ix slave-bal-mask-ic1 index)
            (ix slave-bal-mask-ic2 index)))
        (var new-mask (if (> enable 0)
            (if (= (bitwise-and old-mask target-bit) 0)
                (+ old-mask target-bit)
                old-mask)
            (if (> (bitwise-and old-mask target-bit) 0)
                (- old-mask target-bit)
                old-mask)))
        (var mask-safe (and
            (= (bitwise-and new-mask (shr new-mask 1)) 0)
            (<= (mask-bit-count new-mask) (bms-get-param 'max_bal_ch))
            (or (= enable 0) (>= target-v (bms-get-param 'vc_balance_min)))
        ))

        (if (not mask-safe) {
            (print "BAL OVR blocked: voltage, adjacency, or channel limit")
            false
        } {
            (if (= target-ic 1)
                (setix slave-bal-mask-ic1 index new-mask)
                (setix slave-bal-mask-ic2 index new-mask)
            )
            (if (any-cached-balancing) {
                (master-set-chg 0)
                (setq is-charging false)
                (setq manual-bal-active true)
                (set-c-balance-request true)
                (if (send-cached-balance-masks 0) {
                    (setix bal-state 0 bal-state-active)
                    (setq bal-status "BAL_OVR")
                    (print (str-merge "BAL OVR cell " (str-from-n cell "%d")
                        (if (> enable 0) " on" " off")))
                    true
                } {
                    (stop-all-balancing)
                    false
                })
            } {
                (stop-all-balancing)
                true
            })
        })
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
    (trap-value '(master-stop-balance-sync) false)
})

(defun stop-all-balancing () {
    (setq manual-bal-active false)
    (setix bal-state 0 bal-state-stopping)
    (clear-cached-balancing)
    ; C reports success only after all three physical zero-mask passes finish.
    (var zero-ok (send-zero-balance-all))
    (var release-ok (if zero-ok (set-c-balance-request false) false))
    (var bal-off-ok (and zero-ok release-ok))
    (if bal-off-ok {
        (setix bal-state 0 bal-state-idle)
        (setq bal-status "")
    } {
        ; STOPPING keeps CHG_EN low and is retried by the balance supervisor.
        (setq bal-status "BAL_STOP")
    })
    (setq bal-off-failed (not bal-off-ok))
    bal-off-ok
})

(defun zero-balancing-preserve-request () {
    (clear-cached-balancing)
    (var request-ok (set-c-balance-request true))
    (var zero-ok (and request-ok (send-zero-balance-all)))
    (if zero-ok {
        (setix bal-state 0 bal-state-requested)
        (setq bal-off-failed false)
    } {
        (setix bal-state 0 bal-state-stopping)
        (setq bal-status "BAL_STOP")
        (setq bal-off-failed true)
    })
    zero-ok
})

(defun balance-safe-now () (and
    pack-data-ok
    slave-data-fresh
    temp-data-ok
    (not is-charging)
    (<= (* (abs iout)
            (if (balance-state-is bal-state-active) 0.8 1.0))
        (bms-get-param 'balance_max_current))
    (>= c-min (bms-get-param 'vc_balance_min))
    (or
        (not cell-temp-mon-en)
        (<= t-max (bms-get-param 't_bal_max_cell))
    )
    (<= t-ic (bms-get-param 't_bal_max_ic))
))

; Return the first safety condition that prevents balancing. Keep this aligned
; with balance-safe-now so field logs identify the real cause instead of only
; reporting the generic "pack conditions" message.
(defun balance-block-reason ()
    (cond
        ((not pack-data-ok) (str-merge "pack data: " pack-status))
        ((not slave-data-fresh) "slave data stale")
        ((not temp-data-ok) "temperature data invalid")
        (is-charging "charging is active")
        ((> (* (abs iout)
                (if (balance-state-is bal-state-active) 0.8 1.0))
            (bms-get-param 'balance_max_current))
            (str-merge "current=" (str-from-n iout "%.2f")
                "A limit=" (str-from-n (bms-get-param 'balance_max_current) "%.2f") "A"))
        ((< c-min (bms-get-param 'vc_balance_min))
            (str-merge "cell-min=" (str-from-n c-min "%.3f")
                "V limit=" (str-from-n (bms-get-param 'vc_balance_min) "%.3f") "V"))
        ((and cell-temp-mon-en (> t-max (bms-get-param 't_bal_max_cell)))
            (str-merge "cell-temp=" (str-from-n t-max "%.1f")
                "C limit=" (str-from-n (bms-get-param 't_bal_max_cell) "%.1f") "C"))
        ((> t-ic (bms-get-param 't_bal_max_ic))
            (str-merge "ic-temp=" (str-from-n t-ic "%.1f")
                "C limit=" (str-from-n (bms-get-param 't_bal_max_ic) "%.1f") "C"))
        (true "")
    )
)

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

(defun slave-balance-masks (sid threshold max-ch) {
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
        (list
            (balance-ic-group ic1-volts c-min threshold max-ch)
            (if (> ic2-cnt 0)
                (balance-ic-group ic2-volts c-min threshold max-ch)
                0
            )
        )
    } nil)
})

(defun balance-needed-now () {
    (var max-ch (bms-get-param 'max_bal_ch))
    (var threshold (bms-get-param 'vc_balance_start))
    (var needed false)
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (and (master-slave-active? sid) (master-slave-fresh? sid)) {
            (var masks (slave-balance-masks sid threshold max-ch))
            (if masks {
                (if (or (> (ix masks 0) 0) (> (ix masks 1) 0))
                    (setq needed true)
                )
            })
        })
        (setq sid (+ sid 1))
    })

    needed
})

(defun start-balance-request () {
    (if (and (balance-state-is bal-state-idle) (not (c-balance-inhibited))) {
        (if (set-c-balance-request true) {
            ; A new manual or post-charge session uses the start threshold.
            ; Repeated 30-second phases switch to the end threshold below.
            (setq balance-cycle-threshold (bms-get-param 'vc_balance_start))
            (setix bal-state 0 bal-state-requested)
            (setq bal-status "BAL_REQ")
            true
        } {
            (setq bal-status "BAL_STOP")
            (setix bal-state 0 bal-state-stopping)
            false
        })
    } {
        false
    })
})

(defun try-manual-balance-request () {
    (refresh-pack-data)
    (var block-reason (balance-block-reason))
    (if (and (= (str-len block-reason) 0)
            (all-configured-slaves-fresh)
            (balance-needed-now)) {
        (start-balance-request)
        true
    } {
        (if (> (str-len block-reason) 0)
            (print (str-merge "BAL CMD: blocked: " block-reason))
            (if (not (all-configured-slaves-fresh))
                (print "BAL CMD: blocked: configured slave data stale")
                (print "BAL CMD: no cells above start threshold")
            )
        )
        false
    })
})

(defun clear-balance-request () {
    (if (balance-in-progress) (stop-all-balancing))
    (setq trigger-bal-after-charge false)
})

(defun fail-close-active-balance () {
    (if (balance-in-progress) {
        (master-set-chg 0)
        (setq is-charging false)
        (setq charge-ok false)
    })
})

(defun update-balance-masks (threshold) {
    (var max-ch (bms-get-param 'max_bal_ch))
    (var any-bal false)
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (clear-cached-balancing)
    (loopwhile (<= sid max-sid) {
        (if (and (master-slave-active? sid) (master-slave-fresh? sid)) {
            (var masks (slave-balance-masks sid threshold max-ch))
            (if masks {
                (var ic1-cnt (master-get-cells-ic1 sid))
                (var ic2-cnt (master-get-cells-ic2 sid))
                (var ic1-mask (ix masks 0))
                (var ic2-mask (ix masks 1))

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

    any-bal
})

(defun balance-cycle-failed (message) {
    (print message)
    ; JFBMS32 cancels the automatic post-charge request when discharge current
    ; makes balancing unsafe. Other temporary safety conditions keep retrying.
    (if (> iout (bms-get-param 'balance_max_current))
        (setq trigger-bal-after-charge false)
    )
    (fail-close-active-balance)
    (stop-all-balancing)
    (if trigger-bal-after-charge
        (setq bal-auto-retry-ts (systime))
    )
})

; Same basic loop as JFBMS32: turn all channels off, wait two seconds for clean
; voltages, select non-adjacent high cells, then refresh the same masks once per
; second. No acknowledgement/settled state machine is needed; the slave status
; bitmap is the source used by VESC Tool, and each slave has its own watchdog.
(defun balance-thd () {
    (var keepalive-ts (systime))

    (loopwhile t {
        (if (and (balance-state-is bal-state-idle) (c-balance-inhibited)) {
            (setix bal-state 0 bal-state-stopping)
            (setq bal-status "BAL_STOP")
        })

        (if (balance-state-is bal-state-stopping)
            (stop-all-balancing)
        )

        (if (and
                (balance-state-is bal-state-idle)
                trigger-bal-after-charge
                (not is-charging)
                (> (secs-since bal-auto-retry-ts) 5.0)
            )
            (start-balance-request)
        )

        (if (balance-state-is bal-state-requested) {
            (refresh-pack-data)
            (var block-reason (balance-block-reason))
            (if (or (> (str-len block-reason) 0)
                    (not (all-configured-slaves-fresh))) {
                (balance-cycle-failed (str-merge "BAL: blocked: "
                    (if (> (str-len block-reason) 0)
                        block-reason
                        "configured slave data stale")))
            } {
                (if (not (zero-balancing-preserve-request)) {
                    (balance-cycle-failed "BAL: could not send zero masks")
                } {
                    (setq bal-status "BAL_SETTLE")
                    (sleep 2.0)
                    (refresh-pack-data)
                    (if (not (balance-safe-now)) {
                        (balance-cycle-failed "BAL: unsafe after settle")
                    } {
                        (if (update-balance-masks balance-cycle-threshold) {
                            (if (send-cached-balance-masks 0) {
                                (setix bal-state 0 bal-state-active)
                                (setq bal-status "BAL")
                                (setq balance-active-start-ts (systime))
                                (setq keepalive-ts (systime))
                            } {
                                (balance-cycle-failed "BAL: transmit failed")
                            })
                        } {
                            (print "BAL: target reached")
                            (clear-balance-request)
                        })
                    })
                })
            })
        })

        (if (balance-state-is bal-state-active) {
            (refresh-pack-data)
            (var block-reason (balance-block-reason))
            (if (or (> (str-len block-reason) 0)
                    (not (all-configured-slaves-fresh))) {
                (balance-cycle-failed (str-merge "BAL: stopped: "
                    (if (> (str-len block-reason) 0)
                        block-reason
                        "configured slave data stale")))
                (setq keepalive-ts (systime))
            } {
                (if manual-bal-active {
                    (if (>= (secs-since keepalive-ts) balance-keepalive-period-s) {
                        (setq keepalive-ts (systime))
                        (if (not (send-cached-balance-masks 0))
                            (balance-cycle-failed "BAL OVR: keepalive transmit failed")
                        )
                    })
                } {
                    (if (>= (secs-since balance-active-start-ts) balance-active-time-s) {
                        (setq balance-cycle-threshold (bms-get-param 'vc_balance_end))
                        (setix bal-state 0 bal-state-requested)
                    } {
                        (if (>= (secs-since keepalive-ts) balance-keepalive-period-s) {
                            (setq keepalive-ts (systime))
                            (if (not (send-cached-balance-masks 0))
                                (balance-cycle-failed "BAL: keepalive transmit failed")
                            )
                        })
                    })
                })
            })
        } {
            (setq keepalive-ts (systime))
        })

        (sleep 0.1)
    })
})

;;;;;;;;;; Events and status ;;;;;;;;;;

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-bal-ovr (? cell) (? enable)) {
                (manual-balance-cell cell (if (> enable 0) 1 0))
            })
            ((event-bms-force-bal (? v)) {
                (if (= v 1) {
                    (if manual-bal-active (stop-all-balancing))
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
                (if (and chg-allowed (or
                        (assoc rtc-val 'short-service)
                        (assoc rtc-val 'charge-fault)
                        (fast-oc-latched)))
                    (clear-service-faults)
                )
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
                (if (or (= ah 1) (= wh 1)) {
                    (save-settings)
                    (print "BMS counters reset and stored")
                })
            })
            (event-bms-zero-ofs {
                (print "CAL: zero-current calibration requested")
                (start-current-calibration)
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
    (if calibration-running
        (setq s (status-append s "CALIBRATING"))
    )
    ; An uncaptured current zero is an intentional 0 A startup state, not an
    ; ADC fault. Charger voltage and PCB temperature must still be valid.
    (if (or (not (charger-data-ok)) (not (pcb-temp-data-ok)))
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

        (update-charge-control dt)
        (update-sleep-shutdown-timer)

        (update-status)
        (update-primary-can-status)
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
        (if (and
                pack-data-ok
                (> i-zero-time 10.0)
                (<= c-min (bms-get-param 'vc_empty))
                (> ah-cnt-soc 0.0)
            ) {
            (setq ah-cnt-soc 0.0)
            (set-soc-value 0.0 "VOLTAGE" "EMPTY" true)
            (checkpoint-soc true "EMPTY")
        })

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
    (var boot-status (trap-value '(master-boot-status) '(0 0)))
    (if (and boot-status (>= (length boot-status) 2))
        (print (str-merge "Boot count=" (str-from-n (ix boot-status 0) "%d")
            " reset-reason=" (str-from-n (ix boot-status 1) "%d")))
    )

    ; Reset values that must be relative to this boot, not image creation.
    (setq bal-auto-retry-ts (systime))
    (setq trigger-bal-after-charge false)
    (setq balance-active-start-ts (systime))
    (setq balance-cycle-threshold (bms-get-param 'vc_balance_start))
    (setq charge-complete false)
    (setq charger-detected-prev false)
    (setq charge-block-beeped false)
    (setq charge-enable-beeped false)
    (setq manual-bal-active false)
    (setq calibration-running false)
    (var current-cal (trap-value '(master-current-calibration) '(nil 1.65 0.0 nil)))
    (setq current-zero-ready (and current-cal (>= (length current-cal) 1)
        (ix current-cal 0)))
    (if current-zero-ready
        (print (str-merge "CAL: using stored zero "
            (str-from-n (ix current-cal 1) "%.4f") " V"))
    )
    (setq last-fast-trip-count -1)
    (setq charge-dis-ts (systime))
    (setq t-last (systime))
    (setq primary-can-status-ts (systime))
    (setq loop-cnt 0)

    (var primary-can-rate (truncate (param-or 'can_status_rate_hz 0) 0 200))
    (if (= primary-can-rate 0)
        (print "Primary CAN status disabled (standalone mode)")
        (print (str-merge "Primary CAN status enabled at "
            (str-from-n primary-can-rate "%d") " Hz"))
    )

    (if (> app-wdt-timeout 0)
        (wdt-configure true app-wdt-timeout)
        (wdt-disable)
    )

    ; COM enable low (active), charge off.
    (gpio-hold-deepsleep 0)
    (gpio-hold 6 0)
    (gpio-write 6 0)
    (set-chg false)
    (set-bms-val 'bms-can-id (can-local-id))

    ; Buzzer on GPIO8.
    (pwm-start 4000 0.0 0 8)

    (load-rtc-val)
    (setq charge-complete (if (assoc rtc-val 'charge-complete) true false))

    (master-reset-slaves)

    (event-register-handler (spawn 200 event-supervisor))
    (event-enable 'event-bms-bal-ovr)
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

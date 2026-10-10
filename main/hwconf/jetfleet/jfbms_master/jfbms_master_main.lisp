; JFBMS Master
; Pack controller for JFBMS slave boards. The master owns charge, sleep,
; shutdown, counters, status and balance decisions. Cell voltages and cell
; temperatures are taken only from slave CAN data.

;;;;;;;;;; User settings ;;;;;;;;;;

(def app-wdt-timeout 10)     ; Seconds. Fed only after a successful control step; same as JFBMS32.
(def user-beeps-en true)     ; Informational beeps. Fault and shutdown alarms always sound.
(def beep-duty-alarm 0.5)    ; Loud 4 kHz drive for faults that need attention. Same as JFBMS32.
(def beep-duty-quiet 0.03)   ; Quiet drive for informational beeps. Same as JFBMS32.
(def beep-freq-high 4000) ; OK / info tone, Hz
(def beep-freq-low 2700) ; attention tone, Hz
(def buzzer-pin 8)
(def sleep-unblock-en true)
(def charger-max-delay 10.0) ; Let the charger establish current before enforcing the minimum.
(def charge-start-margin-v 0.7) ; Charger must exceed pack voltage by this before CHG_EN rises.
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
(def charge-session-valid false)
(def charger-detected-prev false)
(def charge-block-beeped false)
(def charge-block-printed "")
(def charge-enable-beeped false)
(def trigger-bal-after-charge false)
(def charge-complete false)
(def charge-no-current false)
(def active-config-generation 0)
(def balance-cache-generation 0)
(def charge-ts (systime))
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
(def low-soc-timer-wake-pending false)
(def cell-num 0)
(def temp-data-ok false)
(def cell-temp-mon-en false)
; Scratch copies used while scan-pack-from-slaves accumulates.
(def s-cell-num 0)
(def s-vtot 0.0)
(def s-c-min 0.0)
(def s-c-max 0.0)
(def s-t-ic 24.0)
(def s-t-min 24.0)
(def s-t-max 24.0)
(def s-t-mos 24.0)
(def s-cell-temp-mon-en false)
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

(def t-last (systime))
(def rtc-val '(
    (charge-fault . false)
    (charge-complete . false)
    (short-count . 0)
    (short-service . false)
    (sleep-enter-time-s . 0)
    (sleep-total-time-s . 0)))
(def rtc-val-magic 127)

(def prev-active (list 0 0 0 0 0 0 0 0))
(def prev-can-overflow 0)

; Balancing state. Only IDLE may permit CHG_EN to rise.
(def bal-state-idle 0)
(def bal-state-requested 1)
(def bal-state-active 2)
(def bal-state-stopping 3)
(def bal-state (list bal-state-idle))
(def slave-bal-mask-ic1 (list 0 0 0 0 0 0 0 0))
(def slave-bal-mask-ic2 (list 0 0 0 0 0 0 0 0))

(def shutdown-reason-timer 1)
(def shutdown-reason-low-soc-timer 2)
(def shutdown-reason-app 4)

(def loop-cnt 0)
(def buz-mutex (mutex-create))
(def pack-refresh-mutex (mutex-create))

@const-start

; Pre-load dynamically provided helpers so they are included in the image.
str-merge
second
abs

defun
defunret
loopwhile
looprange
loopforeach

;;;;;;;;;; Generic helpers ;;;;;;;;;;

(defun trap-value (expr fallback) {
    (match (trap (eval expr))
        ((exit-ok (? value)) value)
        (_ fallback))
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
        (_ fallback))
})

(defun truncate (n min max) (if (< n min) min (if (> n max) max n)))

(defun cfg-num-slaves () (truncate (param-or 'num_slaves 1) 1 8))

; Primary VESC CAN (TWAI0), same as JFBMS32 and VBMS32: frame 35 with SOC,
; charging, remaining minutes and capacity, then the standard BMS status, every
; 10 Hz control step. TWAI1 slave traffic is independent.
(defun send-can-info () {
    (if (and (> (get-bms-val 'bms-data-version) 0)
             (<= (get-bms-val 'bms-msg-age) 0.3)) {
    (var soc-out (truncate soc 0.0 1.0))
    (var buf-canid35 (array-create 8))
    (var ah-left (* (bms-get-param 'batt_ah) (- 1.0 soc-out)))
    (var min-left (if (< iout -1.0) (* (/ ah-left (- iout)) 60.0) 0.0))
    (bufset-i16 buf-canid35 0 (* soc-out 1000)) ; Battery A SOC
    (bufset-u8 buf-canid35 2 (if is-charging 1 0)) ; Battery A Charging
    (bufset-u16 buf-canid35 3 min-left) ; Battery A Charge Time Minutes
    (bufset-u16 buf-canid35 5 (* (bms-get-param 'batt_ah) 10.0))
    (can-send-sid 35 buf-canid35)
    (send-bms-can)
    })
})

; ESC input current reported on the primary CAN bus, as on JFBMS32. The local
; shunt sees only the charge path, so discharge through the ESC comes from CAN.
(defun can-sum-current () {
    (var i-sum 0.0)
    (loopforeach d (can-list-devs) {
        (var age (can-msg-age d 4))
        (if (and age (< age 0.1)) {
            (var cur (canget-current-in d))
            (if (number? cur)
                (setq i-sum (+ i-sum cur)))
        })
    })
    i-sum
})

(defun bool-int (v) (if v 1 0))

(defun balance-state-is (state) (= (ix bal-state 0) state))

; Hardware-owned fast overcurrent status. Missing or malformed extensions fail
; closed: charging remains disabled until the ADC monitor reports armed.
(defun fast-oc-status ()
    ; C returns: (latched armed trip-count last-raw current-a trip-time-s direction).
    ; A missing extension must fail closed without inventing a latched trip.
    (trap-value '(master-fast-oc-status) '(nil nil 0 0 0.0 0.0 0)))

(defun fast-oc-latched () {
    (var status (fast-oc-status))
    (or (eq status nil) (< (length status) 2) (not (eq (ix status 0) nil)))
})

(defun fast-oc-armed () {
    (var status (fast-oc-status))
    (and status (>= (length status) 2) (not (eq (ix status 1) nil)))
})

(defun c-balance-inhibited () (trap-value '(master-balance-inhibited?) false))

(defun charge-pack-fresh () (trap-value '(master-pack-charge-fresh?) false))

(defun balance-in-progress () (or (not (balance-state-is bal-state-idle)) (c-balance-inhibited)))

; Keep the balance request visible in C before Lisp exposes REQUESTED state.
(defun set-c-balance-request (requested) {
    (trap-value (list 'master-balance-request (bool-int requested)) false)
})

(defun lpf (val sample tc) (- val (* tc (- val sample))))

(defun calc-soc (v-cell) {
    (var empty (bms-get-param 'vc_empty))
    (var full (bms-get-param 'vc_full))
    (var den (- full empty))

    (if (= den 0.0) 0.0 (truncate (/ (- v-cell empty) den) 0.0 1.0))
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
    (ah-cnt-soc  . (7 f))))

(def settings-version-legacy 243i32)
(def settings-version 244i32)

(defun read-setting (name) {
    (var entry (assoc eeprom-addrs name))
    (if (eq (second entry) 'i) (eeprom-read-i (first entry)) (eeprom-read-f (first entry)))
})

(defun write-setting (name val) {
    (var entry (assoc eeprom-addrs name))
    (if (eq (second entry) 'i) (eeprom-store-i (first entry) val) (eeprom-store-f (first entry) val))
})

(defun number-or (value fallback) (if (number? value) value fallback))

(defun rtc-number (name fallback) (number-or (assoc rtc-val name) fallback))

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
    (setq ah-cnt-soc (if (valid-pack-reading) (* (calc-soc c-min) (bms-get-param 'batt_ah)) -1.0))

    (save-settings)
    (write-setting 'ver-code settings-version)
})

(defun load-settings () {
    ; Counter layout is unchanged in 244. Promote 243 in place so the safety
    ; state-machine update does not erase customer Ah/Wh/SOC history.
    (var stored-version (read-setting 'ver-code))
    ; not-eq is used here because a fresh EEPROM can return nil for ver-code;
    ; the numeric = operator raises a type error when comparing nil to i32.
    (if (not-eq stored-version settings-version-legacy)
        (if (not-eq stored-version settings-version) (restore-settings))
        (write-setting 'ver-code settings-version))

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
        1.0))
    (var due (and
        (>= ah-cnt-soc 0.0)
        (or
            force
            (>= age soc-checkpoint-max-time-s)
            (and (>= age soc-checkpoint-min-time-s) (>= delta soc-checkpoint-delta)))))

    (if due {
        (write-setting 'ah-cnt-soc ah-cnt-soc)
        (setq soc-checkpoint-ah ah-cnt-soc)
        (setq soc-checkpoint-ts (systime))
        (if force (print (str-merge "SOC checkpoint: " reason)))
    })
    due
})

(defun status-append (base part)
    (if (> (str-len part) 0) (if (> (str-len base) 0) (str-merge base " | " part) part) base))

; LispBM's numeric comparisons alone do not reject NaN.
(defun number-in-range (value low high)
    (and (number? value) (not (is-nan value)) (not (is-inf value)) (>= value low) (<= value high)))

(defun temp-valid (temp) (number-in-range temp -40.0 120.0))

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

(defunret can-active () {
    (var devs (can-list-devs))
    (if (eq devs nil) (return false))
    (looprange i 1 7 {
        (var age (can-msg-age (first devs) i))
        (if (and age (< age 0.1)) (return true))
    })
    false
})

(defun sleep-duration-s () {
    (var dur (* (bms-get-param 'sleep) 3600.0))

    ; sleep-deep 0 means no timer wakeup, which is not useful for this master.
    (if (< dur 1.0) 1.0 dur)
})

(defun charger-status () {
    ; C supplies the latest raw ADC sample. The returned list is
    ; (valid detected voltage sample-age-ms).
    (trap-value '(master-charger-status) '(nil nil 0.0 999999))
})

(defun test-chg (samples) {
    (var status (charger-status))
    (var detected (and
        status
        (>= (length status) 3)
        (not (eq (ix status 0) nil))
        (> (ix status 2) 5.0)))

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
    (>= soc 0.0)))

;;;;;;;;;; Buzzer and output control ;;;;;;;;;;

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

(defun slave-lost-alarm (sid) {
    (beep 1 0.6)
    (sleep 0.4)
    (beep sid 0.2)
})

(defun set-chg (chg) {
    (var requested (if chg true false))
    (var was-charging is-charging)
    (var allowed true)

    ; Charging has priority. A charge request first performs the checked
    ; three-pass zero-mask handoff; CHG_EN cannot rise while STOPPING fails.
    (if (and requested (fast-oc-latched)) (setq allowed false))
    ; Charging remains locked until the one-second zero capture completes.
    ; Never let a direct caller bypass that pre-charge step.
    (if (and requested (not current-zero-ready)) (setq allowed false))
    (if (and requested (balance-in-progress)) (setq allowed (and allowed (stop-all-balancing))))

    (var ok (and allowed (trap-value (list 'master-set-chg (bool-int requested)) false)))

    (if (and requested ok) {
        (setq is-charging true)
        (if (not was-charging) {
            (setq charge-session-valid false)
            (setq charge-ts (systime))
            (if (not charge-enable-beeped) {
                (setq charge-enable-beeped true)
                (spawn charge-start-beep)
            })
        })
    } {
        (if requested (trap (master-set-chg 0)))
        ; As on JFBMS32, a real fault-free charge session triggers balancing
        ; when it ends for any reason. Voltage alone must not arm balancing.
        (if (and was-charging charge-session-valid pack-data-ok
                (not (assoc rtc-val 'charge-fault))
                (not (assoc rtc-val 'short-service))
                (not (fast-oc-latched))) {
            (setq trigger-bal-after-charge true)
            (setq bal-auto-retry-ts (systime))
        })
        (setq charge-session-valid false)
        (setq is-charging false)
    })

    ok
})

(defun send-slave-beep (code) (send-cached-balance-masks code))

; Informational slave beeps follow user-beeps-en, like the master's quiet beeps.
(defun user-slave-beep (code) (if user-beeps-en (send-slave-beep code)))

; Slave error code for a charge block, 0 = none (codes: jfbms_slave handle-beep).
(defun block-reason-slave-code (reason) (cond
    ((or (eq reason "CHG_CELL_HOT") (eq reason "CHG_MOS_HOT")) 0x10)
    ((eq reason "CHG_CELL_HIGH") 0x11)
    ((eq reason "CHG_CELL_LOW") 0x12)
    ((or (eq reason "FLT_SHORT_LOCK") (eq reason "FLT_FAST_OC_REV")
        (eq reason "FLT_FAST_OC_CHG") (eq reason "FLT_CHG_OC")) 0x13)
    (true 0)))

;;;;;;;;;; Slave data aggregation ;;;;;;;;;;

; Accumulate required IC/NTC temperatures from the same slave snapshot as cells.
; Slave order is IC1, cell1, IC2, cell2; status[4] enables the two cell NTCs.
(defun scan-slave-temperatures (sid status ic2-count) {
    (var flags (if (and status (>= (length status) 5)) (ix status 4) 3))
    (var temps (master-get-slave-temps sid))
    (var expected (if (> ic2-count 0) 4 2))
    (var valid (and temps (>= (length temps) expected)))
    (if valid (looprange i 0 expected {
        (var is-ic (= (mod i 2) 0))
        (var enabled (!= (bitwise-and flags (if (< i 2) 1 2)) 0))
        (var temp (ix temps i))
        (if (and (not is-ic) enabled) (setq s-cell-temp-mon-en true))
        (if (or is-ic enabled) {
            (if (temp-valid temp) {
                (if is-ic {
                    (if (> temp s-t-ic) (setq s-t-ic temp))
                } {
                    (if (< temp s-t-min) (setq s-t-min temp))
                    (if (> temp s-t-max) (setq s-t-max temp))
                })
            } (setq valid false))
        })
    }))
    valid
})

(defun scan-pack-from-slaves () {
    (var missing false)
    (var slave-fault false)
    (var bad-cell false)
    (var stale-slave false)
    (var temps-ok true)
    (setq s-cell-num 0)
    (setq s-vtot 0.0)
    (setq s-c-min 9.0)
    (setq s-c-max 0.0)
    (setq s-t-ic -300.0)
    (setq s-t-min 300.0)
    (setq s-t-max -300.0)
    (setq s-t-mos (trap-value '(master-get-temp-pcb) -300.0))
    (setq s-cell-temp-mon-en false)

    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (if (master-slave-active? sid) {
            (if (not (master-slave-fresh? sid)) (setq stale-slave true))
            (var status (master-get-slave-status sid))
            (var faults (if status (ix status 1) 0))
            (var ic2-count (master-get-cells-ic2 sid))
            (var count (+ (master-get-cells-ic1 sid) ic2-count))
            (var cells (master-get-slave-cells sid))
            (if (> (bitwise-and faults 0x0B) 0) (setq slave-fault true))
            (if (not (scan-slave-temperatures sid status ic2-count)) (setq temps-ok false))
            (if (and cells (> count 0) (= (length cells) count)) {
                (loopforeach v cells {
                    (if (number-in-range v 0.0 65.534) {
                        (setq s-cell-num (+ s-cell-num 1))
                        (setq s-vtot (+ s-vtot v))
                        (if (< v s-c-min) (setq s-c-min v))
                        (if (> v s-c-max) (setq s-c-max v))
                        (if (not (number-in-range v 1.0 5.0)) (setq bad-cell true))
                    } (setq bad-cell true))
                })
            } (setq bad-cell true))
        } (setq missing true))
    })

    (if (= s-cell-num 0) { (setq s-c-min 0.0) (setq s-c-max 0.0) })
    (if (not s-cell-temp-mon-en) { (setq s-t-min -300.0) (setq s-t-max -300.0) })
    ; Publish the scan atomically: other threads read these globals without
    ; the refresh mutex and must never see a half-accumulated pack.
    (setq cell-num s-cell-num)
    (setq vtot s-vtot)
    (setq c-min s-c-min)
    (setq c-max s-c-max)
    (setq t-ic s-t-ic)
    (setq t-min s-t-min)
    (setq t-max s-t-max)
    (setq t-mos s-t-mos)
    (setq cell-temp-mon-en s-cell-temp-mon-en)
    (setq temp-data-ok (and temps-ok (temp-valid s-t-ic) (temp-valid s-t-mos)
        (or (not s-cell-temp-mon-en) (and (temp-valid s-t-min) (temp-valid s-t-max)))))
    (setq slave-data-fresh (and (> s-cell-num 0) (not missing) (not slave-fault)
        (not bad-cell) (not stale-slave)))
    (setq pack-data-ok (and slave-data-fresh temp-data-ok))
    (setq pack-status (cond
        (missing "WAIT_SLAVE")
        (slave-fault "SLAVE_FAULT")
        (bad-cell "BAD_CELL")
        ((= s-cell-num 0) "NO_CELL")
        (stale-slave "STALE_SLAVE")
        ((not temp-data-ok) "TEMP_INVALID")
        (true "")))
    ; C owns VESC cell/temperature publication; these globals are for control.
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
        (setq iout (+ (master-get-current) (can-sum-current)))
        ; Publish the local ADC samples explicitly. VESC Tool reads these
        ; bms_values fields, not the Lisp globals used by charge control.
        ; Keep the normal VESC sign convention: discharge positive, charge
        ; negative.
        (set-bms-val 'bms-v-charge vt-vchg)
        (set-bms-val 'bms-i-in iout)
        (set-bms-val 'bms-i-in-ic iout)
        (scan-pack-from-slaves)
        true)))
    (mutex-unlock pack-refresh-mutex)
    (match result
        ((exit-ok _) true)
        (_ {
            (setq pack-data-ok false)
            (setq slave-data-fresh false)
            false
        }))
})

;;;;;;;;;; Shutdown and sleep ;;;;;;;;;;

(defun shutdown-reason-name (reason)
    (cond
        ((= reason shutdown-reason-timer) "timer")
        ((= reason shutdown-reason-low-soc-timer) "low-soc-timer")
        ((= reason shutdown-reason-app) "app")
        (true "unknown")))

(defun fail-close-outputs (clear-bal-trigger) {
    (var local-ok false)
    (match (trap (master-fail-close-local))
        ((exit-ok _) (setq local-ok true))
        (_ (match (trap (gpio-write 5 0)) ((exit-ok _) (setq local-ok true)) (_ nil))))

    (setq is-charging false)
    (setq charge-ok false)
    (if clear-bal-trigger (setq trigger-bal-after-charge false))
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
    ; Capture the local ADC zero independently of slave availability.
    (master-set-chg 0)
    (setq is-charging false)
    (setq charge-ok false)
    (sleep 1.0)
    (print "CAL: sampling current ADC")
    ; Firmware 7 commits synchronously; verify once instead of polling for 15s.
    (if (trap-value '(master-calibrate-current) false) {
        (var status (trap-value '(master-current-calibration) nil))
        (and status (>= (length status) 4) (ix status 0) (not (ix status 3)))
    } false)
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
        (if calibration-beep (spawn (fn () (user-beep 2 0.2))))
    } {
        (print "CAL: failed; charging remains off")
        (if calibration-beep (spawn (fn () (user-beep-low 1 0.6))))
    })
    (setq calibration-beep false)
})

(def calibration-beep false) ; true only for a manual VESC Tool zero request

(defun start-current-calibration () {
    (if (not calibration-running) {
        ; Latch before spawning so charge checks cannot queue multiple captures.
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
    (save-settings)
    (master-shutdown)

    ; Returning here means hardware power removal failed. Keep GPIO19 asserted
    ; and beep continuously. Give the power latch one second to react first.
    (sleep 1.0)
    (print "BMS hardware shutdown failed")
    (loopwhile t {
        (trap (master-shutdown))
        (trap (wdt-reset))
        (beep 5 0.2)
        (sleep 0.5)
    })
})

(defun bms-shutdown-low-soc-timer () (bms-shutdown-impl shutdown-reason-low-soc-timer))
(defun bms-shutdown-app () (bms-shutdown-impl shutdown-reason-app))
(defun low-soc-timer-wake () (and
    (valid-pack-reading)
    (< soc 0.05)
    (<= c-min (bms-get-param 'vc_empty))
    (not trigger-bal-after-charge)
    (not (test-chg 1))
    (external-wake-inactive)
    (not (is-connected))
    (not (can-active))))

(defun reset-sleep-total-time () {
    (setassoc rtc-val 'sleep-total-time-s 0)
    (save-rtc-val)
})

(defun process-sleep-time () {
    (var source (master-wakeup-source))
    ; The wall clock resets at boot: a timer wake adds the configured interval.
    (cond
        ((= source 1) (setassoc rtc-val 'sleep-total-time-s 0))
        ((= source 2) (setassoc rtc-val 'sleep-total-time-s
            (+ (rtc-number 'sleep-total-time-s 0) (sleep-duration-s)))))
    (setassoc rtc-val 'sleep-enter-time-s 0)
    (save-rtc-val)
    (update-sleep-shutdown-timer)
    source
})

(defun update-sleep-shutdown-timer () {
    (if (or (external-wake-active) (is-comm-connected) (can-active)) {
        (if (> (rtc-number 'sleep-total-time-s 0) 0) (reset-sleep-total-time))
    } {
        (if (and (charger-data-ok) (> (bms-get-param 'shutdown) 0)
                (>= (rtc-number 'sleep-total-time-s 0) (* (bms-get-param 'shutdown) 86400)))
            (bms-shutdown-impl shutdown-reason-timer))
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
    (not (can-active))))

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

            ; Suppress only the fast comparator before its 1.65 V reference is
            ; powered down. Keep the original COM_EN and wake-source ordering.
            (if (trap-value '(master-sleep-disarm-fast-oc) false) {
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
                    ; Let the current reference settle before callbacks can trip.
                    (sleep 0.15)
                    (if (not (trap-value '(master-sleep-rearm-fast-oc) false))
                        (print "Fast OC rearm failed after sleep cancellation"))
                    (setassoc rtc-val 'sleep-enter-time-s 0)
                    (save-rtc-val)
                    (print "Sleep cancelled by ENABLE")
                } {
                    (sleep-deep dur)
                })
            } {
                (setassoc rtc-val 'sleep-enter-time-s 0)
                (save-rtc-val)
                (print "Sleep deferred because fast OC disarm failed")
                (sleep-fail-alarm)
            })
        } {
            (print "Sleep deferred because fail-close did not complete")
            (sleep-fail-alarm)
        })
    })
})

;;;;;;;;;; Charge and counter control ;;;;;;;;;;

(defun pack-generation () (trap-value '(master-get-pack-generation) 0))

(defun qualify-dt-valid (dt) (and (> dt 0.0) (<= dt control-max-qualify-dt)))

(defun set-soc-value (new-soc source reason force-log) {
    (var bounded (truncate new-soc 0.0 1.0))
    (var previous soc)
    (var batt-ah (bms-get-param 'batt_ah))
    (var coulomb-candidate (if (> batt-ah 0.0) (truncate (/ ah-cnt-soc batt-ah) 0.0 1.0) 0.0))
    (var voltage-candidate (calc-soc c-min))

    (if (or force-log (< previous 0.0) (> (abs (- bounded previous)) 0.05))
        (print (str-merge
            "SOC " source "/" reason
            " prev=" (str-from-n previous "%.3f")
            " new=" (str-from-n bounded "%.3f")
            " ah=" (str-from-n coulomb-candidate "%.3f")
            " volt=" (str-from-n voltage-candidate "%.3f")
            " I=" (str-from-n iout "%.2f")
            " min=" (str-from-n c-min "%.3f")
            " max=" (str-from-n c-max "%.3f")
            " gen=" (str-from-n (pack-generation) "%d"))))

    (setq soc bounded)
    (set-bms-val 'bms-soc bounded)
})

(defun publish-counters () {
    (set-bms-val 'bms-ah-cnt ah-cnt)
    (set-bms-val 'bms-wh-cnt wh-cnt)
    (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
    (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
    (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
    (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)
})

(defun update-soc-and-counters (dt) {
    (var batt-ah (bms-get-param 'batt_ah))
    (var dt-ok (qualify-dt-valid dt))

    (if (and pack-data-ok (< ah-cnt-soc 0.0)) (setq ah-cnt-soc (* (calc-soc c-min) batt-ah)))

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
                    (lpf soc voltage-soc (truncate (* 100.0 (bms-get-param 'soc_filter_const)) 0.0 1.0))
                    "VOLTAGE" "TRACK" false)
                (set-soc-value voltage-soc "VOLTAGE" "INITIAL" true))
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
    (publish-counters)
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
            (spawn (fn () (user-beep-low 2 0.6))) ; charge fault
            (user-slave-beep 0x13)
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
        ; Master buzzer only, same 3 short high beeps as JFBMS32.
        (spawn (fn () (user-beep 3 0.2)))
        (user-slave-beep 0x03)
        (print (str-merge "CHG complete: " reason))
    })
})

(defun rearm-charge-hysteresis () {
    ; Keep the charge-complete latch set for the whole existing balance cycle:
    ; 30 seconds active, two seconds settled, and repeat until balancing is done.
    ; Only the clean, post-balance voltage can rearm charging.
    (if (and charge-complete pack-data-ok (not (balance-in-progress))
            (< c-max (bms-get-param 'vc_charge_start))) {
        (setq charge-complete false)
        ; The charger can remain connected throughout an overnight balance
        ; cycle. Start a new current-establishment window so CHG_EN can rise
        ; before min_charge_current is enforced for this top-up.
        (setq charge-ts (systime))
        (setassoc rtc-val 'charge-complete false)
        (save-rtc-val)
        (print "CHG rearmed below vc_charge_start")
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
    (if (and charge-complete (> (rtc-number 'short-count 0) 0) (not (assoc rtc-val 'short-service))) {
        (setassoc rtc-val 'short-count 0)
        (setq changed true)
    })
    (if changed (save-rtc-val))

    (setq charge-complete false)
    (setq charge-no-current false)
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

; Charge temperature limits are always enforced, as on JFBMS32.
(defun charge-block-reason (charger-detected) {
    (cond
        ((assoc rtc-val 'short-service) "FLT_SHORT_LOCK")
        ((fast-oc-latched) (if (> (fast-oc-direction) 0) "FLT_FAST_OC_REV" "FLT_FAST_OC_CHG"))
        ((not (fast-oc-armed)) "FLT_FAST_ADC")
        ((assoc rtc-val 'charge-fault) "FLT_CHG_OC")
        (charge-complete "CHG_COMPLETE")
        (charge-no-current "CHG_NO_CURRENT")
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
        ; Charging has priority over balancing; set-chg stops balancing first.
        ; Before starting, cells must be below vc_charge_start (restart hysteresis).
        ((>= c-max (bms-get-param (if is-charging 'vc_charge_end 'vc_charge_start))) "CHG_CELL_HIGH")
        ((<= c-min (bms-get-param 'vc_charge_min)) "CHG_CELL_LOW")
        ((not temp-data-ok) "TEMP_INVALID")
        ((and cell-temp-mon-en (>= t-max (bms-get-param 't_charge_max))) "CHG_CELL_HOT")
        ((and cell-temp-mon-en (<= t-min (bms-get-param 't_charge_min))) "CHG_CELL_COLD")
        ((>= t-mos (bms-get-param 't_charge_max_mos)) "CHG_MOS_HOT")
        ; As on JFBMS32: presence is a fixed 5 V in C, but CHG_EN may only
        ; rise once the unloaded charger is 0.7 V above the pack.
        ((and (not is-charging) (<= vt-vchg (+ vtot charge-start-margin-v))) "CHG_VOLTAGE_LOW")
        (true ""))
})

; C has the final say on CHG_EN and reports why it refused.
(defun c-charge-block-reason () {
    (var reason (trap-value '(master-chg-block-reason) ""))
    (if (> (str-len reason) 0) reason "CHG_REFUSED")
})

(defun charge-block-detail (reason)
    (str-merge reason
        " vchg=" (str-from-n vt-vchg "%.2f")
        " vpack=" (str-from-n vtot "%.2f")
        " cmin=" (str-from-n c-min "%.3f")
        " cmax=" (str-from-n c-max "%.3f")
        " I=" (str-from-n iout "%.2f")))

(defun update-charge-control (dt) {
    (var charger-detected (test-chg 1))
    (var raw-current (master-get-current-raw))
    (var charge-current (if (number? raw-current) (- raw-current) 0.0))
    ; End of charge uses the EMA-filtered shunt current so one noisy or
    ; pulsing-charger sample cannot stop charging. Over-current stays raw.
    (var low-current (<= (- (master-get-current)) (bms-get-param 'min_charge_current)))

    (record-fast-oc)

    ; The first plug-in captures and stores zero. Later sessions reuse it.
    (if (and charger-detected (not charger-detected-prev)) {
        (setq charge-block-beeped false)
        (setq charge-enable-beeped false)
        (setq charge-ts (systime))
        (print "CHG: charger detected")
    })
    (setq charger-detected-prev charger-detected)

    (if (and charger-detected (not current-zero-ready) (not calibration-running))
        (start-current-calibration))

    ; Same simple slow-current guard as JFBMS32. The C fast comparator remains
    ; an independent backstop.
    (if (and is-charging (> charge-current (bms-get-param 'max_charge_current))) {
        (setq charge-ok false)
        (if (not (assoc rtc-val 'charge-fault)) {
            (setassoc rtc-val 'charge-fault true)
            (save-rtc-val)
            (spawn (fn () (user-beep-low 2 0.6))) ; charge fault
            (user-slave-beep 0x13)
        })
        (set-chg false)
    })
    (if (and is-charging (> charge-current (bms-get-param 'min_charge_current)))
        (setq charge-session-valid true))

    ; Five seconds unplugged starts a completely new session.
    (if (and (charger-data-ok) (not charger-detected) (> (secs-since charge-dis-ts) 5.0)) {
        (set-chg false)
        (clear-session-after-disconnect)
    })

    ; The cell limit and low port voltage take effect without startup delays.
    (if (and is-charging pack-data-ok (>= c-max (bms-get-param 'vc_charge_end)))
        (finish-charge "CELL_LIMIT"))

    (rearm-charge-hysteresis)

    (var block-reason (charge-block-reason charger-detected))
    (setq charge-ok (and charger-detected (= (str-len block-reason) 0)))

    (if (and is-charging charger-detected charge-ok low-current
            (>= (secs-since charge-ts) charger-max-delay)) {
        (if (and charge-session-valid (>= c-max (bms-get-param 'vc_charge_start)))
            (finish-charge "CURRENT_TAPER")
            (setq charge-no-current true))
        (setq block-reason (if charge-complete "CHG_COMPLETE" "CHG_NO_CURRENT"))
        (setq charge-ok false)
    })
    (cond
        ((not charge-ok) (set-chg false))
        (true {
            (if (not (set-chg true)) {
                (setq charge-ok false)
                (setq block-reason (c-charge-block-reason))
            })
        }))

    (setq chg-status (if is-charging "CHARGING" block-reason))

    (if (and charger-detected (not is-charging) (> (str-len block-reason) 0)) {
        ; Only a block that outlasts plug-in transients (calibration, charger
        ; start-up); a completed charge is not a block.
        (if (and (not charge-block-beeped) (not-eq block-reason "CHG_COMPLETE")
                (> (secs-since charge-ts) 3.0)) {
            (setq charge-block-beeped true)
            (spawn charge-block-beep)
            (var code (block-reason-slave-code block-reason))
            (if (> code 0) (user-slave-beep code))
        })
        ; Log every reason change, not only the first, with the values behind it.
        (if (not-eq block-reason charge-block-printed)
            (print (str-merge "CHG blocked: " (charge-block-detail block-reason))))
    })
    (setq charge-block-printed (if (and charger-detected (not is-charging)) block-reason ""))

    (set-bms-val 'bms-chg-allowed (bool-int chg-allowed))
})

;;;;;;;;;; Balancing ;;;;;;;;;;

; Pick cells from one BQ76952, same rule as JFBMS32: highest first, never two
; adjacent cells on the same BQ, at most max_bal_ch channels per BQ.
(defun balance-ic-group (voltages c-min threshold max-ch) {
    (var order (sort (fn (a b) (> (cdr a) (cdr b)))
        (map (fn (i) (cons i (ix voltages i))) (range (length voltages)))))
    (var mask 0)
    (var count 0)
    (loopforeach cell order {
        (if (>= count max-ch) (break))
        (var i (car cell))
        (var neighbours (bitwise-or (shl 1 (+ i 1)) (if (> i 0) (shl 1 (- i 1)) 0)))
        (if (and (> (- (cdr cell) c-min) threshold) (= (bitwise-and mask neighbours) 0)) {
            (setq mask (bitwise-or mask (shl 1 i)))
            (setq count (+ count 1))
        })
    })
    mask
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

(defun any-cached-balancing () (> (+ (apply + slave-bal-mask-ic1) (apply + slave-bal-mask-ic2)) 0))

(defun mask-bit-count (mask) {
    (var count 0)
    (looprange i 0 16 {
        (if (!= (bitwise-and mask (shl 1 i)) 0) (setq count (+ count 1)))
    })
    count
})

; Map a pack cell into (slave, IC, bit, voltage), using the announced topology.
(defunret pack-cell-location (cell) {
    (var base 0)
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (var ic1-count (master-get-cells-ic1 sid))
        (var count (+ ic1-count (master-get-cells-ic2 sid)))
        (if (and (>= cell base) (< cell (+ base count))) {
            (var cells (master-get-slave-cells sid))
            (if (and cells (= (length cells) count)) {
                (var local (- cell base))
                (return (list sid (if (< local ic1-count) 1 2)
                    (shl 1 (if (< local ic1-count) local (- local ic1-count)))
                    (ix cells local)))
            })
        })
        (setq base (+ base count))
    })
    nil
})

(defunret manual-balance-cell (cell enable) {
    (var generation active-config-generation)
    (var target (pack-cell-location cell))
    (if (or (not target) (and (> enable 0) (not (balance-safe-now)))) {
        ; Disabling an unknown cell stops the entire pack to fail closed.
        (if (= enable 0) (stop-all-balancing))
        (print "BAL OVR blocked: invalid cell or unsafe pack state")
        (return false)
    })
    (if (not manual-bal-active) {
        (setq trigger-bal-after-charge false)
        (clear-cached-balancing)
    })
    (var index (- (ix target 0) 1))
    (var masks (if (= (ix target 1) 1) slave-bal-mask-ic1 slave-bal-mask-ic2))
    (var bit (ix target 2))
    (var old-mask (ix masks index))
    (var mask (if (> enable 0) (bitwise-or old-mask bit) (bitwise-and old-mask (bitwise-not bit))))
    (if (not (and (= (bitwise-and mask (shr mask 1)) 0)
            (<= (mask-bit-count mask) (bms-get-param 'max_bal_ch))
            (or (= enable 0) (>= (ix target 3) (bms-get-param 'vc_balance_min))))) {
        (print "BAL OVR blocked: voltage, adjacency, or channel limit")
        (return false)
    })
    (setq balance-cache-generation generation)
    (setix masks index mask)
    (if (not (any-cached-balancing)) {
        (stop-all-balancing)
        (return true)
    })
    (master-set-chg 0)
    (setq is-charging false)
    (setq manual-bal-active true)
    ; Enter ACTIVE before raising the C request. The balance thread treats an
    ; IDLE state with a raised request as stray and would stop the override.
    (setix bal-state 0 bal-state-active)
    (setq bal-status "BAL_OVR")
    (set-c-balance-request true)
    (if (not (send-cached-balance-masks 0)) {
        (stop-all-balancing)
        (return false)
    })
    (print (str-merge "BAL OVR cell " (str-from-n cell "%d") (if (> enable 0) " on" " off")))
    true
})

(defun send-cached-balance-masks (beep-code) {
    (var generation balance-cache-generation)
    (var all-ok true)
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (if (master-slave-active? sid) {
            (if (not (master-send-balance sid (ix slave-bal-mask-ic1 (- sid 1))
                    (ix slave-bal-mask-ic2 (- sid 1)) beep-code generation))
                (setq all-ok false))
        })
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

(defun balance-safe-now () (= (str-len (balance-block-reason)) 0))

; One safety gate serves control decisions, manual overrides and diagnostics.
(defun balance-block-reason ()
    (cond
        ((!= active-config-generation (master-config-generation)) "configuration changed")
        ((not pack-data-ok) (str-merge "pack data: " pack-status))
        ((not slave-data-fresh) "slave data stale")
        ((not temp-data-ok) "temperature data invalid")
        ((<= (bms-get-param 'max_bal_ch) 0) "max_bal_ch is 0")
        (is-charging "charging is active")
        (charge-ok "charging requested")
        ((> (* (abs iout) (if (balance-state-is bal-state-active) 0.8 1.0))
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
        ((not (all-configured-slaves-fresh)) "configured slave data stale")
        (true "")))

(defunret all-configured-slaves-fresh () {
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (if (not (and (master-slave-active? sid) (master-slave-fresh? sid))) (return false))
    })
    true
})

(defun slave-balance-masks (sid threshold max-ch) {
    (var cells (master-get-slave-cells sid))
    (var ic1-count (master-get-cells-ic1 sid))
    (var ic2-count (master-get-cells-ic2 sid))
    (var count (+ ic1-count ic2-count))
    (if (and cells (> count 0) (= (length cells) count))
        (list
            (balance-ic-group (take cells ic1-count) c-min threshold max-ch)
            (balance-ic-group (drop cells ic1-count) c-min threshold max-ch))
        nil)
})

(defun fresh-slave-balance-masks (sid threshold max-ch)
    (if (and (master-slave-active? sid) (master-slave-fresh? sid))
        (slave-balance-masks sid threshold max-ch) nil))

(defunret balance-needed-now () {
    (var max-ch (bms-get-param 'max_bal_ch))
    (var threshold (bms-get-param 'vc_balance_start))
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (var masks (fresh-slave-balance-masks sid threshold max-ch))
        (if (and masks (> (apply + masks) 0)) (return true))
    })
    false
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
    (var reason (balance-block-reason))
    (cond
        ((> (str-len reason) 0) { (print (str-merge "BAL CMD: blocked: " reason)) false })
        ((not (balance-needed-now)) { (print "BAL CMD: no cells above start threshold") false })
        (true (start-balance-request)))
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

(defun update-balance-masks (threshold generation) {
    (setq balance-cache-generation generation)
    (var max-ch (bms-get-param 'max_bal_ch))
    (clear-cached-balancing)
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (var masks (fresh-slave-balance-masks sid threshold max-ch))
        (if masks {
            (setix slave-bal-mask-ic1 (- sid 1) (ix masks 0))
            (setix slave-bal-mask-ic2 (- sid 1) (ix masks 1))
            (if (> (apply + masks) 0)
                (print (str-merge "BAL S" (str-from-n sid "%d")
                    " IC1:" (mask-to-bin (ix masks 0) (master-get-cells-ic1 sid))
                    " IC2:" (mask-to-bin (ix masks 1) (master-get-cells-ic2 sid))
                    " min=" (str-from-n c-min "%.3f"))))
        })
    })
    (any-cached-balancing)
})

(defun balance-cycle-failed (message) {
    (print message)
    ; JFBMS32 cancels the automatic post-charge request when discharge current
    ; makes balancing unsafe. Other temporary safety conditions keep retrying.
    (if (> iout (bms-get-param 'balance_max_current)) (setq trigger-bal-after-charge false))
    (fail-close-active-balance)
    (stop-all-balancing)
    (if trigger-bal-after-charge (setq bal-auto-retry-ts (systime)))
})

; Slaves publish their actual balance state and stop on keepalive timeout.
; Settle with zero masks, select high non-adjacent cells, then keep them alive.
; The captured generation prevents a settings save from reviving an old cycle.
(defunret begin-balance-cycle () {
    (var generation active-config-generation)
    (refresh-pack-data)
    (var reason (balance-block-reason))
    (if (> (str-len reason) 0) {
        (balance-cycle-failed (str-merge "BAL: blocked: " reason))
        (return false)
    })
    (if (not (zero-balancing-preserve-request)) {
        (balance-cycle-failed "BAL: could not send zero masks")
        (return false)
    })
    (setq bal-status "BAL_SETTLE")
    (sleep 2.0)
    (refresh-pack-data)
    (if (not (and (balance-safe-now) (balance-state-is bal-state-requested)
            (= generation (master-config-generation)))) {
        (balance-cycle-failed "BAL: unsafe after settle")
        (return false)
    })
    (if (not (update-balance-masks balance-cycle-threshold generation)) {
        (print "BAL: target reached")
        (clear-balance-request)
        (return false)
    })
    (if (not (send-cached-balance-masks 0)) {
        (balance-cycle-failed "BAL: transmit failed")
        (return false)
    })
    (setix bal-state 0 bal-state-active)
    (setq bal-status "BAL")
    (setq balance-active-start-ts (systime))
    true
})

(defun balance-thd () {
    (var keepalive-ts (systime))
    (loopwhile t {
        (if (and (balance-state-is bal-state-idle) (c-balance-inhibited)) {
            (setix bal-state 0 bal-state-stopping)
            (setq bal-status "BAL_STOP")
        })
        (if (balance-state-is bal-state-stopping) (stop-all-balancing))
        (if (and (balance-state-is bal-state-idle) trigger-bal-after-charge
                (not is-charging) (not charge-ok) (> (secs-since bal-auto-retry-ts) 5.0))
            (start-balance-request))
        (if (and (balance-state-is bal-state-requested) (begin-balance-cycle))
            (setq keepalive-ts (systime)))

        (if (balance-state-is bal-state-active) {
            (refresh-pack-data)
            (var reason (balance-block-reason))
            (cond
                ((> (str-len reason) 0) {
                    (balance-cycle-failed (str-merge "BAL: stopped: " reason))
                    (setq keepalive-ts (systime))
                })
                ((and (not manual-bal-active)
                        (>= (secs-since balance-active-start-ts) balance-active-time-s)) {
                    (setq balance-cycle-threshold (bms-get-param 'vc_balance_end))
                    (setix bal-state 0 bal-state-requested)
                })
                ((>= (secs-since keepalive-ts) balance-keepalive-period-s) {
                    (setq keepalive-ts (systime))
                    (if (not (send-cached-balance-masks 0))
                        (balance-cycle-failed (if manual-bal-active
                            "BAL OVR: keepalive transmit failed" "BAL: keepalive transmit failed")))
                }))
        } (setq keepalive-ts (systime)))
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
                        (print "BAL CMD: ignored"))
                } {
                    (print "BAL CMD: stop")
                    (setq trigger-bal-after-charge false)
                    (stop-all-balancing)
                })
            })
            ((event-bms-chg-allow (? allow)) {
                (setq chg-allowed (= allow 1))
                (if chg-allowed {
                    (setq charge-no-current false)
                    (setq charge-ts (systime))
                })
                (if (not chg-allowed) (set-chg nil))
                (if (and chg-allowed (or
                        (assoc rtc-val 'short-service)
                        (assoc rtc-val 'charge-fault)
                        (fast-oc-latched)))
                    (clear-service-faults))
                (set-bms-val 'bms-chg-allowed (bool-int chg-allowed))
                (print (str-merge "CHG: " (if chg-allowed "allowed" "blocked")))
            })
            ((event-bms-reset-cnt (? ah) (? wh)) {
                (if (= ah 1) (setq ah-cnt 0.0))
                (if (= wh 1) (setq wh-cnt 0.0))
                (if (or (= ah 1) (= wh 1)) {
                    (publish-counters)
                    (save-settings)
                    (print "BMS counters reset and stored")
                })
            })
            (event-bms-zero-ofs {
                (print "CAL: zero-current calibration requested")
                (setq calibration-beep true)
                (start-current-calibration)
            })
            ((event-data-rx ? data) (handle-app-data data))
            (_ nil))))

(defun handle-app-data (data)
    (match (trap (read data))
        ((exit-ok (bms-shutdown)) {
            (print "APPUI requested BMS shutdown")
            (spawn (fn () (bms-shutdown-app)))
        })
        (_ (print "Ignoring unsupported APPUI command"))))

(defun update-status () {
    (var s "")
    (var cfg-warning (master-config-warning))
    (if (!= (bitwise-and cfg-warning 1) 0) (setq s (status-append s "WARN_SLAVE_COUNT")))
    (if (!= (bitwise-and cfg-warning 2) 0) (setq s (status-append s "WARN_SLAVE_IDS")))

    (setq s (status-append s chg-status))
    (setq s (status-append s bal-status))
    (setq s (status-append s pack-status))
    (if calibration-running (setq s (status-append s "CALIBRATING")))
    ; An uncaptured current zero is an intentional 0 A startup state, not an
    ; ADC fault. Charger voltage and PCB temperature must still be valid.
    (if (or (not (charger-data-ok)) (not (pcb-temp-data-ok))) (setq s (status-append s "ADC_FAULT")))
    (if bal-off-failed (setq s (status-append s "BAL_OFF_FAIL")))
    (if fail-close-failed (setq s (status-append s "FAIL_CLOSE_FAIL")))

    (if (and chg-allowed (not charge-ok) (not is-charging) (test-chg 1) (= (str-len chg-status) 0))
        (setq s (status-append s "CHG_BLOCK")))

    (set-bms-val 'bms-status s)
})

(defun update-slave-presence () {
    (looprange sid 1 (+ (cfg-num-slaves) 1) {
        (var active (bool-int (master-slave-active? sid)))
        (var previous (ix prev-active (- sid 1)))
        (if (!= active previous) {
            (print (str-merge "Slave " (str-from-n sid "%d")
                (if (= active 1) " connected" " disconnected")))
            (if (= active 0) {
                (set-chg nil)
                (stop-all-balancing)
                (send-slave-beep 0x04)
                (spawn (fn () (slave-lost-alarm sid)))
            }
                ; Only the slave that came online plays power-on (0x01).
                (if user-beeps-en
                    (master-send-balance sid (ix slave-bal-mask-ic1 (- sid 1))
                        (ix slave-bal-mask-ic2 (- sid 1)) 0x01 balance-cache-generation)))
            (setix prev-active (- sid 1) active)
        })
    })
})

(defun supervise (worker message) {
    (loopwhile t {
        (match (trap (worker))
            ((exit-ok _) nil)
            (_ { (print message) (fail-close-outputs true) }))
        (sleep 0.2)
    })
})

(defun event-supervisor () (supervise event-handler "Event handler crashed, restarting"))

(defun balance-supervisor ()
    (supervise balance-thd "Balance controller crashed, restarting fail-closed"))

(defun fail-close-retry-thd () {
    (loopwhile t {
        (if (or fail-close-failed bal-off-failed) (fail-close-outputs true))
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
            sleep-unblock-en)))

        (var should-unblock true)
        (looprange i 0 60 {
            (if (not (sleep-unblock-ok)) (setq should-unblock false))
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

(defun sync-runtime-config () {
    (var generation (master-config-generation))
    (if (!= generation active-config-generation) {
        ; Native apply has already stopped the hardware and invalidated CAN
        ; snapshots. Clear matching Lisp state before acknowledging this save.
        (master-set-chg 0)
        (setq is-charging false)
        (setq charge-ok false)
        (setq trigger-bal-after-charge false)
        (setq manual-bal-active false)
        (clear-cached-balancing)
        (setq balance-cache-generation generation)
        (setix bal-state 0 bal-state-idle)
        (set-c-balance-request false)
        (setq bal-status "")
        (setq charge-no-current false)
        (setq charge-block-beeped false)
        (setq charge-enable-beeped false)
        (setq balance-cycle-threshold (bms-get-param 'vc_balance_start))
        (setq charge-ts (systime))
        (setq bal-auto-retry-ts (systime))
        (setq i-zero-time 0.0)
        (setq pack-data-ok false)
        (setq slave-data-fresh false)
        (setq temp-data-ok false)
        (looprange i 0 8 {
            (setix prev-active i 0)
        })
        ; Preserve SOC/counters and latched protection faults across a save.
        (var reconfigured (>= active-config-generation 0))
        (if (master-config-ack generation) {
            (setq active-config-generation generation)
            (if reconfigured (spawn (fn () (user-beep 2 0.2))))
            (print "BMS settings applied automatically; waiting for fresh pack data")
        })
    })
})

(defun main-control-step () {
    (sync-runtime-config)
    ; Drain CAN at 20 Hz.
    (master-can-read-all)

    ; 10 Hz control and display work.
    (if (= (mod loop-cnt 2) 0) {
        (var dt (secs-since t-last))
        (setq t-last (systime))

        (refresh-pack-data)

        (if pack-data-ok (update-soc-and-counters dt))

        (update-charge-control dt)
        (update-sleep-shutdown-timer)

        (update-status)
        (send-can-info)
        (update-slave-presence)

        ; Measure idle time for sleep and shutdown decisions.
        (if (or (not (current-data-ok)) (> (abs iout) (bms-get-param 'min_current_sleep)))
            (setq i-zero-time 0.0)
            (setq i-zero-time (+ i-zero-time dt)))

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

        ; Low SOC shutdown is evaluated once after a deep-sleep timer wake.
        ; A charger, external enable, connection or CAN activity makes this a
        ; normal wake and must not trigger the shutdown.
        (if (and low-soc-timer-wake-pending (valid-pack-reading)) {
            (var low-soc-shutdown (low-soc-timer-wake))
            (setq low-soc-timer-wake-pending false)
            (if low-soc-shutdown (bms-shutdown-low-soc-timer))
        })

        ; The master has no local BQ to put to sleep; slaves handle their own
        ; BQ state and the master only drops COM/ESP.
        (if (sleep-allowed-now) (enter-master-sleep))
    })

    (setq loop-cnt (+ loop-cnt 1))
})

(defun main () {
    (print "=== JFBMS Master ===")
    (loopwhile (!= (bms-fw-version) 7) {
        (master-set-chg 0)
        (print (if (< (bms-fw-version) 7)
            "Firmware too old; update master firmware"
            "Package too old; update master Lisp application"))
        (sleep 5.0)
    })
    (var boot-status (trap-value '(master-boot-status) '(0 0)))
    (if (and boot-status (>= (length boot-status) 2))
        (print (str-merge "Boot count=" (str-from-n (ix boot-status 0) "%d")
            " reset-reason=" (str-from-n (ix boot-status 1) "%d"))))

    ; Reuse live configuration initialization on every image boot.
    (setq active-config-generation -1)
    (sync-runtime-config)
    (setq balance-active-start-ts (systime))
    (setq charge-complete false)
    (setq charger-detected-prev false)
    (setq calibration-running false)
    (var current-cal (trap-value '(master-current-calibration) '(nil 1.65 0.0 nil)))
    (setq current-zero-ready (and current-cal (>= (length current-cal) 1) (ix current-cal 0)))
    (if current-zero-ready
        (print (str-merge "CAL: using stored zero " (str-from-n (ix current-cal 1) "%.4f") " V")))
    (setq last-fast-trip-count -1)
    (setq charge-dis-ts (systime))
    (setq t-last (systime))
    (setq loop-cnt 0)

    (if (> app-wdt-timeout 0) (wdt-configure true app-wdt-timeout) (wdt-disable))

    ; COM enable low (active), charge off.
    (gpio-hold-deepsleep 0)
    (gpio-hold 6 0)
    (gpio-write 6 0)
    (set-chg false)
    (set-bms-val 'bms-can-id (can-local-id))

    ; COM_EN powers the external CAN transceiver. After deep sleep it is held
    ; off until this point, so bring TWAI0 up after COM_EN is low.
    (sleep 0.05)
    (trap (can-start))
    (print "Primary CAN up; BMS status and frame 35 at 10 Hz")

    ; Buzzer on GPIO8.
    (pwm-start beep-freq-high 0.0 0 buzzer-pin)

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

    ; Seed the pack snapshot once; the control loop keeps refreshing it. Do not
    ; block boot waiting for slaves — charging self-gates on pack-data-ok, so the
    ; pack comes online as soon as the first broadcast arrives.
    (refresh-pack-data)

    (setq init-done true)
    (load-settings)
    (var wake-source (process-sleep-time))
    (setq low-soc-timer-wake-pending (= wake-source 2))

    (if pack-data-ok
        (print (str-merge "Pack ready: slaves=" (str-from-n (cfg-num-slaves) "%d")
            " cells=" (str-from-n cell-num "%d")))
        (print "Slave CAN pack data pending; control loop will adopt it")
    )

    ; 2 beeps = initialization complete.
    (user-beep 2 0.2)

    ; 5 long beeps (always on) = the ADC stalled and the watchdog reset the chip.
    (if (trap-value '(master-adc-stall-reset?) false) {
        (print "ADC stall: chip was reset by ADC watchdog")
        (sleep 0.5)
        (beep 5 0.4)
    })

    (spawn 200 balance-supervisor)
    (spawn 100 fail-close-retry-thd)
    (spawn 100 sleep-unblock-thd)

    ; As on JFBMS32, only a completed control step feeds the watchdog. A
    ; controller that keeps crashing stays fail-closed and then resets the chip.
    (loopwhile t {
        (match (trap (main-control-step))
            ((exit-ok _) (wdt-reset))
            (_ {
                (setq control-crash-count (+ control-crash-count 1))
                (print "Main controller crashed, retrying fail-closed")
                (fail-close-outputs true)
                (set-bms-val 'bms-status "CONTROL_FAULT")
                (sleep 0.5)
            }))
        (sleep 0.05)
    })
})

@const-end

(image-save)
(main)

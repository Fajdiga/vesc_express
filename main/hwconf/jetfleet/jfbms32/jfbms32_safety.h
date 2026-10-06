#ifndef JFBMS32_SAFETY_H_
#define JFBMS32_SAFETY_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Only a completed control scan may renew this lease, never an I2C access
// or a balancing worker. Time is unsigned milliseconds (wrap is intentional).
#define BMS_CONTROL_TIMEOUT_MS 5000U

typedef struct {
    bool inhibited;
    bool shutdown;
    bool armed;
    uint32_t last_update;
    uint32_t generation;
} bms_safety_state;

static inline void bms_safety_inhibit(bms_safety_state *s, bool shutdown) {
    if (!s->inhibited) { s->generation++; }
    s->inhibited = true;
    s->shutdown |= shutdown;
    s->armed = false;
}

// Call only after configuration readback and successful balance disable.
static inline void bms_safety_ready(bms_safety_state *s) {
    s->inhibited = s->shutdown;
    s->armed = false;
}

static inline bool bms_safety_expired(const bms_safety_state *s, uint32_t now) {
    return s->armed && (uint32_t)(now - s->last_update) >= BMS_CONTROL_TIMEOUT_MS;
}

static inline bool bms_safety_feed(bms_safety_state *s, uint32_t now, uint32_t generation) {
    if (bms_safety_expired(s, now)) {
        bms_safety_inhibit(s, false);
    }
    if (s->inhibited || s->shutdown || generation != s->generation) {
        return false;
    }
    s->last_update = now;
    s->armed = true;
    return true;
}

static inline bool bms_safety_allowed(const bms_safety_state *s, uint32_t now) {
    return !s->inhibited && !s->shutdown && s->armed && !bms_safety_expired(s, now);
}

static inline bool bms_temperature_valid(float temperature) {
    return isfinite(temperature) && temperature >= -50.0f && temperature <= 150.0f;
}

static inline float bms_ntc_temperature(float volts, float pullup, float nominal, float beta) {
    if (!isfinite(volts) || volts <= 0.0f || volts >= 1.79f || nominal <= 0 || beta <= 0) {
        return NAN;
    }
    float resistance = pullup / (1.8f / volts - 1.0f) - 500.0f;
    return resistance > 0.0f
        ? 1.0f / (logf(resistance / nominal) / beta + 1.0f / 298.15f) - 273.15f
        : NAN;
}

#endif

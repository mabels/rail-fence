#include "globals.h"
#include <soc/gpio_struct.h>

// ── Module globals (extern'd in globals.h) ────────────────────────────────────
volatile int32_t g_enc_delta   = 0;
volatile bool    g_enc_btn_raw = false;
uint32_t         g_enc_btn_ms  = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Full quadrature state machine — both CLK and DT on CHANGE.
// Reads both pins atomically from the GPIO register (single 32-bit read)
// so transient glitches between two digitalRead() calls can't cause phantom ticks.
// Draining uses g_enc_delta -= delta (not = 0) so counts arriving between
// the loop's read and clear are preserved rather than zeroed out.
// ─────────────────────────────────────────────────────────────────────────────

void IRAM_ATTR enc_isr() {
    static uint8_t state = 3;  // start at detent: CLK=HIGH, DT=HIGH → s=11=3
    static int8_t  acc   = 0;

    // Single 32-bit register read captures CLK and DT at exactly the same instant
    const uint32_t gpio_in = GPIO.in;
    const uint8_t  clk     = (gpio_in >> PIN_ENC_CLK) & 1;
    const uint8_t  dt      = (gpio_in >> PIN_ENC_DT)  & 1;
    const uint8_t  s       = (clk << 1) | dt;

    // CW  full detent cycle: 3→1→0→2→3  (acc sums to +4)
    // CCW full detent cycle: 3→2→0→1→3  (acc sums to -4)
    static const int8_t T[4][4] = {
        {  0, -1, +1,  0 },  // prev=00
        { +1,  0,  0, -1 },  // prev=01
        { -1,  0,  0, +1 },  // prev=10
        {  0, +1, -1,  0 },  // prev=11 (detent)
    };
    acc  += T[state][s];
    state = s;

    // Emit exactly one tick when encoder snaps back to detent
    if (s == 3) {
        if      (acc > 0) g_enc_delta += 1;
        else if (acc < 0) g_enc_delta -= 1;
        acc = 0;
    }
}

void IRAM_ATTR enc_sw_isr() {
    g_enc_btn_raw = true;
}

void encoder_setup() {
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    pinMode(PIN_ENC_SW,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), enc_isr,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  enc_isr,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_SW),  enc_sw_isr, FALLING);
}

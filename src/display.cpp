#include "globals.h"
#include <Wire.h>
#include <U8g2lib.h>

// SW_I2C (bit-bang) — avoids i2c-ng driver issues in Arduino ESP32 v3.x
static U8G2_SH1106_128X64_NONAME_F_SW_I2C g_display(
    U8G2_R0, PIN_SCL, PIN_SDA, U8X8_PIN_NONE);

static uint32_t     g_last_disp_ms = 0;
static CaliperReading g_last_cal{ 0, 0, false, CaliperUnit::MM };
static bool           g_cal_valid = false;

// ─────────────────────────────────────────────────────────────────────────────

static const char* sync_state_label() {
    switch (g_sync_state) {
        case SyncState::IDLE:         return "IDLE";
        case SyncState::MOVING:       return g_is_initiator ? "INIT" : "PEER";
        case SyncState::WAITING_PEER: return "WAIT";
        case SyncState::FAILED:       return "FAIL";
    }
    return "????";
}

void display_setup() {
    g_display.begin();
    g_display.setFont(u8g2_font_6x10_tf);
    g_display.clearBuffer();
    g_display.drawStr(0, 10, "Slider booting...");
    g_display.drawStr(0, 24, "peer protocol");
    g_display.sendBuffer();
}

// Self-throttled to 4 Hz — call every loop iteration.
void display_update() {
    const uint32_t now = millis();
    if (now - g_last_disp_ms < 250) return;
    g_last_disp_ms = now;

    g_display.clearBuffer();
    g_display.setFont(u8g2_font_6x10_tf);

    if (g_ui_state == UIState::MENU) {
        // ── Menu screen ──────────────────────────────────────────────────────
        g_display.drawStr(0, 10, "=== MENU ===");
        g_display.drawHLine(0, 12, 128);

        if (g_menu_scroll > 0)
            g_display.drawStr(116, 24, "^");

        for (int8_t v = 0; v < MENU_VISIBLE; v++) {
            const int8_t  i   = g_menu_scroll + v;
            if (i >= MENU_COUNT) break;
            const int16_t y   = 24 + v * 11;
            const bool    sel = (i == g_menu_sel);
            char buf[22];

            if (i < ENC_STEP_COUNT) {
                snprintf(buf, sizeof(buf), "%s%-6s%s",
                    sel ? ">" : " ",
                    ENC_STEP_LABELS[i],
                    (i == g_enc_step_idx) ? "*" : "");
            } else if (i == MENU_LOCK) {
                snprintf(buf, sizeof(buf), "%sLock: %s",
                    sel ? ">" : " ", g_locked ? "ON" : "OFF");
            } else if (i == MENU_RESET) {
                snprintf(buf, sizeof(buf), "%sReset to 0", sel ? ">" : " ");
            } else if (i == MENU_IP) {
                snprintf(buf, sizeof(buf), "%sIP:%-15s",
                    sel ? ">" : " ", WiFi.localIP().toString().c_str());
            } else if (i == MENU_MAC) {
                snprintf(buf, sizeof(buf), "%s%s",
                    sel ? ">" : " ", WiFi.macAddress().c_str());
            } else if (i == MENU_TRANSPORT) {
                snprintf(buf, sizeof(buf), "%sSync:%s",
                    sel ? ">" : " ",
                    g_transport == Transport::ESPNOW ? "ENOW*" : "UDP *");
            } else {
                snprintf(buf, sizeof(buf), "%sExit", sel ? ">" : " ");
            }
            g_display.drawStr(0, y, buf);
        }

        if (g_menu_scroll + MENU_VISIBLE < MENU_COUNT)
            g_display.drawStr(116, 64, "v");

    } else {
        // ── Normal screen ────────────────────────────────────────────────────
        const int32_t pos  = stepper ? stepper->getCurrentPosition() : 0;
        const bool    busy = stepper && stepper->isRunning();

        // Row 1: sync state + step size + lock indicator
        char row1[24];
        snprintf(row1, sizeof(row1), "%s [%s]%s",
            sync_state_label(),
            ENC_STEP_LABELS[g_enc_step_idx],
            g_locked ? " L" : " U");
        g_display.drawStr(0, 10, row1);

        // Row 2: P (actual) and T (target) on one line
        char pt_str[32];
        snprintf(pt_str, sizeof(pt_str), "P%+.1f T%+.1f%s",
            pos / STEPS_PER_MM,
            g_target_pos / STEPS_PER_MM,
            busy ? ">" : " ");
        g_display.drawStr(0, 22, pt_str);

        // Row 3: caliper reading (large) — update cache on fresh ISR value
        g_display.setFont(u8g2_font_7x13_tf);
        {
            CaliperReading fresh{0, 0, false, CaliperUnit::MM};
            if (g_caliper.read(fresh)) {
                new (&g_last_cal) CaliperReading(fresh);
                g_cal_valid = true;
            }
        }
        char cal_str[24];
        if (g_cal_valid) {
            const bool neg = g_last_cal.negative || g_last_cal.whole < 0;
            if (g_last_cal.unit == CaliperUnit::MM) {
                snprintf(cal_str, sizeof(cal_str), "%c%d.%02u mm",
                    neg ? '-' : '+',
                    abs((int)g_last_cal.whole),
                    (unsigned)g_last_cal.frac);
            } else {
                snprintf(cal_str, sizeof(cal_str), "%c%d.%04u\"",
                    neg ? '-' : '+',
                    abs((int)g_last_cal.whole),
                    (unsigned)g_last_cal.frac);
            }
        } else {
            snprintf(cal_str, sizeof(cal_str), "--- mm");
        }
        g_display.drawStr(0, 40, cal_str);

        // Row 4: peer position + transport indicator
        g_display.setFont(u8g2_font_6x10_tf);
        char peer[32];
        snprintf(peer, sizeof(peer), "Pr%+.1fmm %s",
            g_peer_pos / STEPS_PER_MM,
            g_transport == Transport::ESPNOW ? "EN" : "UD");
        g_display.drawStr(0, 54, peer);
    }

    g_display.sendBuffer();
}

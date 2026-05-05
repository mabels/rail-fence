#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <FastAccelStepper.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "MoveCmd.h"
#include "Config.h"

// ── Pin assignments ───────────────────────────────────────────────────────────
static constexpr uint8_t PIN_STEP    = 13;
static constexpr uint8_t PIN_DIR     = 14;
static constexpr uint8_t PIN_ENABLE  = 10;  // TMC2209 EN: LOW = enabled
static constexpr uint8_t PIN_UART_TX = 9;   // → 1kΩ → TMC2209 UART-PDN
static constexpr uint8_t PIN_UART_RX = 12;  // → TMC2209 UART-PDN (direct)
static constexpr uint8_t PIN_SPREAD  = 11;  // SPREAD_EN: HIGH = SpreadCycle
static constexpr uint8_t PIN_SDA     = 42;
static constexpr uint8_t PIN_SCL     = 41;

// ── Rotary encoder ────────────────────────────────────────────────────────────
static constexpr uint8_t PIN_ENC_CLK = 4;
static constexpr uint8_t PIN_ENC_DT  = 5;
static constexpr uint8_t PIN_ENC_SW  = 6;

// ── Motion calibration ────────────────────────────────────────────────────────
// 8000 steps = 10 mm  →  800 steps/mm
static constexpr float    STEPS_PER_MM  = 800.0f;
static constexpr float    MAX_POS_MM    = 190.0f;
static constexpr float    MIN_POS_MM    =   0.0f;
// 900 mm/min = 15 mm/s  →  12 000 steps/s
// accel: 31.25 mm/s²   →  25 000 steps/s²
static constexpr uint32_t SPEED_MAX_HZ  = 12000;
static constexpr uint32_t ACCEL_DEFAULT = 25000;

// ── Encoder step sizes (steps per click) ─────────────────────────────────────
static const int32_t ENC_STEP_SIZES[]  = { 80, 800, 8000 };  // 0.1 mm, 1 mm, 10 mm
static const char*   ENC_STEP_LABELS[] = { "0.1 mm", "1 mm", "10 mm" };
static constexpr int8_t ENC_STEP_COUNT = 3;

// ── NVS keys ─────────────────────────────────────────────────────────────────
static constexpr char NVS_NS[]      = "slider";
static constexpr char NVS_ROLE[]    = "role";     // uint8: 0=leader 1=follower
static constexpr char NVS_POS[]     = "pos";      // int32: steps from reference
static constexpr char NVS_FLW_MAC[] = "flw_mac";  // bytes[6]


// ── Role ─────────────────────────────────────────────────────────────────────
enum class Role : uint8_t { LEADER = 0, FOLLOWER = 1 };

// ─────────────────────────────────────────────────────────────────────────────
// Global state
// ─────────────────────────────────────────────────────────────────────────────

static Role    g_role            = Role::LEADER;
static char    g_hostname[24]    = {};
static uint8_t g_follower_mac[6] = {};
static char    g_i2c_scan[64]    = "not scanned";

// Rotary encoder — ISR writes, loop reads
static volatile int32_t g_enc_delta   = 0;    // accumulated ticks
static volatile bool    g_enc_btn_raw = false; // set by ISR, cleared by loop
static uint32_t         g_enc_btn_ms  = 0;    // debounce timestamp

// Encoder step size index (into ENC_STEP_SIZES / ENC_STEP_LABELS)
static int8_t g_enc_step_idx = 1;   // default: 1 mm per click

// Locked mode: clamps target to [0 .. MAX_POS_MM]
static bool g_locked = true;

// ── UI state machine ──────────────────────────────────────────────────────────
enum class UIState : uint8_t { NORMAL = 0, MENU = 1 };
static UIState g_ui_state = UIState::NORMAL;

// Menu layout:  0-2 step sizes | 3 lock | 4 reset | 5 IP | 6 MAC | 7 exit
static constexpr int8_t MENU_LOCK           = 3;
static constexpr int8_t MENU_RESET          = 4;
static constexpr int8_t MENU_IP             = 5;
static constexpr int8_t MENU_MAC            = 6;
static constexpr int8_t MENU_EXIT           = 7;
static constexpr int8_t MENU_COUNT          = 8;
static constexpr int8_t MENU_VISIBLE        = 4;  // rows visible at once (6×10 font fits)
static int8_t g_menu_sel    = 0;
static int8_t g_menu_scroll = 0;  // index of topmost visible item

// Stepper
static FastAccelStepperEngine engine  = FastAccelStepperEngine();
static FastAccelStepper*      stepper = nullptr;
static bool                   g_was_running = false;

// Target position — encoder and web both write here; loop drives motor to it
static volatile int32_t  g_target_pos      = 0;
static          int32_t  g_commanded_pos   = 0;  // last value sent to moveTo()

// ESP-NOW
static uint8_t          g_seq          = 0;
static volatile bool    g_follower_ok  = false;
static volatile int32_t g_follower_pos = 0;

// UDP sync transport — two sockets: RX binds to the port, TX sends from any port.
// Keeping them separate prevents beginPacket() from interfering with the RX buffer.
static WiFiUDP  g_udp_rx;   // receive only — bound to UDP_PORT
static WiFiUDP  g_udp_tx;   // send only   — unbound (ephemeral source port)
static constexpr uint16_t UDP_PORT = 4210;

// Restart scheduling (set by web config handler, checked in loop)
static uint32_t g_restart_at = 0;

// Display
// SW_I2C (bit-bang) — avoids i2c-ng driver issues in Arduino ESP32 v3.x
static U8G2_SH1106_128X64_NONAME_F_SW_I2C display(
    U8G2_R0, PIN_SCL, PIN_SDA, U8X8_PIN_NONE);
static uint32_t g_last_disp_ms = 0;

// Web server + network log stream
static AsyncWebServer   server(80);
static AsyncEventSource g_events("/events");
static Preferences      g_prefs;


// Telnet log server — pio device monitor --port socket://<ip>:23
static WiFiServer       g_telnet(23);
static WiFiClient       g_telnet_clients[2];

// net_log() — writes to Serial AND streams to every open /events browser tab.
// Use exactly like Serial.printf.
static void net_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void net_log(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    g_events.send(buf, "log", millis());
    for (auto& c : g_telnet_clients) {
        if (c && c.connected()) c.print(buf);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// NVS / Storage
// ─────────────────────────────────────────────────────────────────────────────

static void nvs_save_position(int32_t pos) {
    g_prefs.begin(NVS_NS, false);
    g_prefs.putInt(NVS_POS, pos);
    g_prefs.end();
}

static void nvs_save_config(Role role, const uint8_t* flw_mac) {
    g_prefs.begin(NVS_NS, false);
    g_prefs.putUChar(NVS_ROLE, static_cast<uint8_t>(role));
    g_prefs.putBytes(NVS_FLW_MAC, flw_mac, 6);
    g_prefs.end();
}

static void nvs_load(Role& role, int32_t& pos, uint8_t* flw_mac) {
    g_prefs.begin(NVS_NS, true);
    role = static_cast<Role>(g_prefs.getUChar(NVS_ROLE, 0));
    pos  = g_prefs.getInt(NVS_POS, 0);
    if (g_prefs.getBytes(NVS_FLW_MAC, flw_mac, 6) != 6)
        memset(flw_mac, 0, 6);
    g_prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
// MAC helpers
// ─────────────────────────────────────────────────────────────────────────────

static bool mac_is_zero(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i]) return false;
    return true;
}

static bool mac_parse(const char* str, uint8_t* mac) {
    return sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
        &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6;
}

static void mac_format(const uint8_t* mac, char* buf, size_t len) {
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW — leader side (send move, receive ack)
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// UDP sync transport
// ─────────────────────────────────────────────────────────────────────────────
// All packets on port 4210 share a 1-byte PktType prefix (see MoveCmd.h).
//
// Startup sync (Hello):
//   Both devices broadcast HelloPkt every 500 ms for HELLO_WINDOW_MS after boot.
//   The device with higher uptime_ms is the authoritative position source; the
//   other adopts that position so both start aligned.
//
// Move reliability:
//   Leader retransmits MoveCmd up to MOVE_RETRIES times (MOVE_ACK_TIMEOUT_MS)
//   if no ACK arrives.  Because positions are absolute, a later packet always
//   resyncs the follower even if earlier ones were dropped.
//
// Heartbeat:
//   Leader re-sends its current position every HEARTBEAT_MS even when idle,
//   keeping the follower in sync after any gap.
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t HELLO_WINDOW_MS    = 6000;   // ms to send hellos on boot
static constexpr uint32_t HELLO_INTERVAL_MS  =  500;   // ms between hello broadcasts
static constexpr uint32_t MOVE_ACK_TIMEOUT_MS=  300;   // ms to wait for ACK
static constexpr uint8_t  MOVE_RETRIES       =    3;   // max resend attempts
static constexpr uint32_t HEARTBEAT_MS       = 5000;   // leader idle resync interval

static IPAddress g_leader_ip;           // follower learns sender IP from first packet
static IPAddress g_follower_ip;         // leader learns follower IP from Hello/ACK
static IPAddress g_bcast_ip;           // subnet broadcast, set after WiFi connects
static uint8_t   g_last_seq     = 0xFF;

// Pending-move retry state (leader only)
static uint8_t   g_pend_seq     = 0xFF;
static int32_t   g_pend_target  = 0;
static uint32_t  g_pend_sent_ms = 0;
static uint8_t   g_pend_tries   = 0;

// Hello timing
static uint32_t  g_hello_until_ms   = 0;
static uint32_t  g_hello_last_ms    = 0;
static uint32_t  g_heartbeat_last_ms= 0;

// ── helpers ──────────────────────────────────────────────────────────────────

// Call once after WiFi connects
static void udp_init_bcast() {
    IPAddress local = WiFi.localIP();
    IPAddress mask  = WiFi.subnetMask();
    for (int i = 0; i < 4; i++)
        g_bcast_ip[i] = (local[i] & mask[i]) | (uint8_t)(~mask[i] & 0xFF);
    net_log("[udp] subnet broadcast: %s\n", g_bcast_ip.toString().c_str());
}

static void udp_broadcast(const void* data, size_t len) {
    // Use 255.255.255.255 as fallback if subnet broadcast not yet computed
    const IPAddress& dst = (g_bcast_ip != IPAddress(0,0,0,0)) ? g_bcast_ip : IPAddress(255,255,255,255);
    g_udp_tx.beginPacket(dst, UDP_PORT);
    g_udp_tx.write(reinterpret_cast<const uint8_t*>(data), len);
    g_udp_tx.endPacket();
}

static void udp_unicast(const IPAddress& dest, const void* data, size_t len) {
    g_udp_tx.beginPacket(dest, UDP_PORT);
    g_udp_tx.write(reinterpret_cast<const uint8_t*>(data), len);
    g_udp_tx.endPacket();
}

// ── hello / startup sync ─────────────────────────────────────────────────────

static void udp_send_hello() {
    const int32_t pos = stepper ? stepper->getCurrentPosition() : g_target_pos;
    HelloPkt h{};
    h.uptime_ms = millis();
    h.position  = pos;
    h.role      = (g_role == Role::LEADER) ? 0 : 1;
    udp_broadcast(&h, sizeof(h));
}

static void udp_apply_position(int32_t pos) {
    // Adopt a position from the peer — move motor and update all state
    if (stepper) stepper->setCurrentPosition(pos);
    g_target_pos    = pos;
    g_commanded_pos = pos;
    nvs_save_position(pos);
    net_log("[sync] adopted peer position: %ld steps (%.1f mm)\n",
        (long)pos, pos / STEPS_PER_MM);
}

static void udp_on_hello(const HelloPkt& h, const IPAddress& src) {
    const uint32_t my_uptime = millis();
    net_log("[sync] hello from %s uptime=%lums pos=%ld role=%s\n",
        src.toString().c_str(), (unsigned long)h.uptime_ms,
        (long)h.position, h.role == 0 ? "leader" : "follower");

    // Leader: record the follower's IP so we can unicast moves directly
    if (g_role == Role::LEADER && h.role == 1) {
        if (g_follower_ip != src) {
            g_follower_ip = src;
            net_log("[sync] follower IP: %s\n", g_follower_ip.toString().c_str());
        }
    }

    if (h.uptime_ms > my_uptime) {
        // Peer has been running longer — their position is authoritative
        const int32_t my_pos = stepper ? stepper->getCurrentPosition() : g_target_pos;
        if (h.position != my_pos) {
            net_log("[sync] peer uptime %lums > ours %lums — adopting pos %ld\n",
                (unsigned long)h.uptime_ms, (unsigned long)my_uptime, (long)h.position);
            udp_apply_position(h.position);
        }
    }
    // If we have higher uptime, keep sending our hellos — peer will adopt us
}

// ── move send / retry ────────────────────────────────────────────────────────

// Send to follower: unicast when IP is known, broadcast as fallback
static void udp_send_to_follower(const void* data, size_t len) {
    if (g_follower_ip != IPAddress(0,0,0,0)) {
        udp_unicast(g_follower_ip, data, len);
    } else {
        udp_broadcast(data, len);
    }
}

static void udp_send_move(int32_t target_steps, uint32_t speed_hz) {
    MoveCmd cmd{};
    cmd.steps     = target_steps;
    cmd.speed_hz  = speed_hz;
    cmd.seq       = ++g_seq;
    udp_send_to_follower(&cmd, sizeof(cmd));
    // Arm retry tracker
    g_pend_seq    = cmd.seq;
    g_pend_target = target_steps;
    g_pend_sent_ms= millis();
    g_pend_tries  = 1;
    // Optimistically assume follower received it — ACK will update if it arrives
    g_follower_ok  = true;
    g_follower_pos = target_steps;
    net_log("[udp] send tgt=%ld seq=%u (try 1) → %s\n",
        (long)target_steps, cmd.seq,
        g_follower_ip != IPAddress(0,0,0,0) ? g_follower_ip.toString().c_str() : "broadcast");
}

static void udp_send_ack(const IPAddress& /*dest*/, uint8_t seq, bool success) {
    MoveAck ack{};
    ack.seq      = seq;
    ack.position = stepper ? stepper->getCurrentPosition() : 0;
    ack.success  = success;
    net_log("[udp] ack→bcast seq=%u pos=%ld bcast=%s\n",
        seq, (long)ack.position, g_bcast_ip.toString().c_str());
    udp_broadcast(&ack, sizeof(ack));
}

// ── retry + heartbeat tick (call from loop) ──────────────────────────────────

static void udp_tick() {
    const uint32_t now = millis();

    // Hello window
    if (now < g_hello_until_ms && now - g_hello_last_ms >= HELLO_INTERVAL_MS) {
        g_hello_last_ms = now;
        udp_send_hello();
    }

    if (g_role != Role::LEADER) return;

    // Retry pending move if no ACK yet
    if (g_pend_tries > 0 && g_pend_tries < MOVE_RETRIES &&
        now - g_pend_sent_ms >= MOVE_ACK_TIMEOUT_MS) {
        g_pend_tries++;
        MoveCmd cmd{};
        cmd.steps    = g_pend_target;
        cmd.speed_hz = SPEED_MAX_HZ;
        cmd.seq      = g_pend_seq;
        udp_send_to_follower(&cmd, sizeof(cmd));
        g_pend_sent_ms = now;
        net_log("[udp] retry tgt=%ld seq=%u (try %u)\n",
            (long)g_pend_target, g_pend_seq, g_pend_tries);
    }

    // Heartbeat — re-send current position so follower stays aligned when idle
    if (now - g_heartbeat_last_ms >= HEARTBEAT_MS) {
        g_heartbeat_last_ms = now;
        {
            const int32_t pos = stepper ? stepper->getCurrentPosition() : g_commanded_pos;
            MoveCmd hb{};
            hb.steps    = pos;
            hb.speed_hz = SPEED_MAX_HZ;
            hb.seq      = g_pend_seq;   // same seq — follower will dedup if it already has it
            udp_send_to_follower(&hb, sizeof(hb));
            // Keep estimated follower position in sync with ours
            g_follower_ok  = true;
            g_follower_pos = pos;
            net_log("[udp] heartbeat pos=%ld\n", (long)pos);
        }
    }
}

// ── receive dispatch ─────────────────────────────────────────────────────────

static void udp_poll() {
    int pkt = g_udp_rx.parsePacket();
    if (pkt < 1) return;

    uint8_t buf[64];
    int len = g_udp_rx.read(buf, sizeof(buf));
    if (len < 1) return;
    const IPAddress src_ip = g_udp_rx.remoteIP();

    net_log("[udp] rx %d bytes from %s type=0x%02x own=%s\n",
        len, src_ip.toString().c_str(),
        (unsigned)buf[0], WiFi.localIP().toString().c_str());

    // Skip our own broadcasts
    if (src_ip == WiFi.localIP()) return;

    const PktType type = static_cast<PktType>(buf[0]);

    if (type == PktType::HELLO && len == static_cast<int>(sizeof(HelloPkt))) {
        HelloPkt h;
        memcpy(&h, buf, sizeof(h));
        udp_on_hello(h, src_ip);
        return;
    }

    if (type == PktType::MOVE_ACK && g_role == Role::LEADER) {
        if (len != static_cast<int>(sizeof(MoveAck))) return;
        MoveAck ack;
        memcpy(&ack, buf, sizeof(ack));
        if (ack.seq == g_pend_seq) {
            g_pend_tries  = 0;   // ACKed — stop retrying
        }
        g_follower_ok  = ack.success;
        g_follower_pos = ack.position;
        // Learn follower IP from ACK source (most reliable discovery path)
        if (g_follower_ip != src_ip) {
            g_follower_ip = src_ip;
            net_log("[udp] follower IP learned from ACK: %s\n", g_follower_ip.toString().c_str());
        }
        net_log("[udp] ack seq=%u ok=%d flw_pos=%ld\n",
            ack.seq, (int)ack.success, (long)ack.position);
        return;
    }

    if (type == PktType::MOVE_CMD && g_role == Role::FOLLOWER) {
        if (len != static_cast<int>(sizeof(MoveCmd))) return;
        MoveCmd cmd;
        memcpy(&cmd, buf, sizeof(cmd));
        g_leader_ip = src_ip;

        // Dedup
        if (cmd.seq == g_last_seq) {
            udp_send_ack(src_ip, cmd.seq, true);
            return;
        }
        g_last_seq = cmd.seq;

        if (!stepper) { udp_send_ack(src_ip, cmd.seq, false); return; }

        g_target_pos = cmd.steps;
        net_log("[udp] recv tgt=%ld seq=%u → target set\n",
            (long)cmd.steps, cmd.seq);
        udp_send_ack(src_ip, cmd.seq, true);
        return;
    }
}

static void udp_init() {
    const bool rx_ok = g_udp_rx.begin(UDP_PORT);
    g_udp_tx.begin(0);   // bind TX to ephemeral port so the socket is fully initialised
    net_log("[udp] rx bind port %u: %s   tx ephemeral: ok\n",
        UDP_PORT, rx_ok ? "ok" : "FAILED");
    // Start hello window — runs for HELLO_WINDOW_MS after boot
    g_hello_until_ms    = millis() + HELLO_WINDOW_MS;
    g_hello_last_ms     = 0;
    g_heartbeat_last_ms = millis();
    net_log("[udp] hello window %lus\n", (unsigned long)(HELLO_WINDOW_MS / 1000));
}


// ─────────────────────────────────────────────────────────────────────────────
// Display
// ─────────────────────────────────────────────────────────────────────────────

static void display_update() {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    if (g_ui_state == UIState::MENU) {
        // ── Menu screen — same 6×10 font as normal screen, 4 items visible ──
        // Header
        display.drawStr(0, 10, "=== MENU ===");
        display.drawHLine(0, 12, 128);

        // Scroll-up indicator (top-right)
        if (g_menu_scroll > 0)
            display.drawStr(116, 24, "^");

        // Visible items: y = 24, 35, 46, 57  (11 px spacing)
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
                    sel ? ">" : " ",
                    g_locked ? "ON" : "OFF");
            } else if (i == MENU_RESET) {
                snprintf(buf, sizeof(buf), "%sReset to 0", sel ? ">" : " ");
            } else if (i == MENU_IP) {
                snprintf(buf, sizeof(buf), "%sIP:%-15s",
                    sel ? ">" : " ",
                    WiFi.localIP().toString().c_str());
            } else if (i == MENU_MAC) {
                snprintf(buf, sizeof(buf), "%s%s",
                    sel ? ">" : " ",
                    WiFi.macAddress().c_str());
            } else {
                snprintf(buf, sizeof(buf), "%sExit", sel ? ">" : " ");
            }
            display.drawStr(0, y, buf);
        }

        // Scroll-down indicator (bottom-right)
        if (g_menu_scroll + MENU_VISIBLE < MENU_COUNT)
            display.drawStr(116, 64, "v");

    } else {
        // ── Normal screen ────────────────────────────────────────────────────
        const int32_t pos  = stepper ? stepper->getCurrentPosition() : 0;
        const bool    busy = stepper && stepper->isRunning();

        // Row 1: role + step size + lock indicator
        char row1[24];
        snprintf(row1, sizeof(row1), "%s [%s]%s",
            g_role == Role::LEADER ? "LEAD" : "FLWR",
            ENC_STEP_LABELS[g_enc_step_idx],
            g_locked ? " L" : " U");
        display.drawStr(0, 10, row1);

        // Row 2: target position (large) — what the encoder has dialled in
        display.setFont(u8g2_font_7x13_tf);
        char tgt_str[24];
        snprintf(tgt_str, sizeof(tgt_str), "T%+.1f mm", g_target_pos / STEPS_PER_MM);
        display.drawStr(0, 26, tgt_str);

        // Row 3: actual drive position (smaller)
        display.setFont(u8g2_font_6x10_tf);
        char pos_str[24];
        snprintf(pos_str, sizeof(pos_str), "P%+.1f mm%s",
            pos / STEPS_PER_MM, busy ? " >" : "  ");
        display.drawStr(0, 40, pos_str);

        // Row 4: follower info or hostname
        if (g_role == Role::LEADER) {
            char flw[32];
            snprintf(flw, sizeof(flw), "F:%s %+.1fmm",
                g_follower_ok ? "ok" : "--",
                g_follower_pos / STEPS_PER_MM);
            display.drawStr(0, 54, flw);
        } else {
            display.drawStr(0, 54, g_hostname);
        }
    }

    display.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Web UI — embedded HTML (single-page, role-aware)
// ─────────────────────────────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Slider</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font:14px monospace;background:#111;color:#ddd;padding:12px;max-width:460px}
h1{color:#4af;margin-bottom:14px;font-size:1.3em}
nav{display:flex;gap:8px;margin-bottom:14px}
nav button{flex:1;padding:8px;background:#1e1e1e;color:#888;border:1px solid #333;border-radius:4px;cursor:pointer;font:13px monospace}
nav button.on{background:#4af;color:#000;border-color:#4af}
section{display:none}.show{display:block!important}
.card{background:#181818;border:1px solid #2a2a2a;border-radius:6px;padding:12px;margin-bottom:12px}
.lbl{color:#666;font-size:.8em;text-transform:uppercase;letter-spacing:.06em;margin-bottom:4px}
.big{font-size:2em;color:#4fa;margin:2px 0;letter-spacing:-1px}
.big.neg{color:#f84}
.sub{font-size:1em;color:#888;margin:2px 0}
.badge{display:inline-block;padding:2px 10px;border-radius:3px;font-size:.8em;margin-top:6px}
.badge.idle{background:#0a2a0a;color:#4fa}
.badge.run{background:#2a1a00;color:#fa4;animation:blink 1s infinite}
@keyframes blink{50%{opacity:.5}}
.row{display:flex;gap:8px;margin-bottom:8px}
.step-btn{flex:1;padding:9px;background:#1e1e1e;color:#888;border:1px solid #333;border-radius:4px;cursor:pointer;font:13px monospace}
.step-btn.on{background:#4af;color:#000;border-color:#4af}
.jog-btn{flex:1;padding:16px;background:#1a2a1a;color:#8f8;border:none;border-radius:4px;cursor:pointer;font:bold 22px monospace}
.jog-btn:active{background:#2a4a2a}
input[type=number],input[type=text]{width:100%;padding:9px;background:#222;color:#eee;border:1px solid #333;border-radius:4px;margin:4px 0 8px;font:14px monospace}
.btn{width:100%;padding:11px;border:none;border-radius:4px;cursor:pointer;font:bold 14px monospace;margin-top:0}
.btn.blue{background:#1a3a5a;color:#4af}
.btn.blue:hover{background:#2a4a7a}
.btn.red{background:#4a1010;color:#f88}
.btn.red:hover{background:#6a1818}
.btn.amber{background:#4a3000;color:#fa8}
.btn.amber:hover{background:#6a4400}
.btn.green{background:#0a3a0a;color:#8f8}
.btn.green:hover{background:#1a5a1a}
.tag{font-size:.7em;padding:2px 7px;border-radius:3px;vertical-align:middle;margin-left:6px}
.tag.L{background:#4af;color:#000}.tag.F{background:#2a6a2a;color:#8f8}
.rad{display:flex;gap:16px;margin:6px 0 12px}
.rad label{display:flex;align-items:center;gap:5px;cursor:pointer;color:#bbb}
.info{color:#555;line-height:1.9em;font-size:.9em}
</style>
</head>
<body>
<h1>Slider <span id="role-tag" class="tag">…</span></h1>
<nav>
  <button class="on" onclick="tab('ctl',this)">Control</button>
  <button onclick="tab('cfg',this)">Settings</button>
</nav>

<!-- ── Control ── -->
<section id="ctl" class="show">

  <div class="card">
    <div class="lbl">Target</div>
    <div class="big" id="tgt">+0.0 mm</div>
    <div class="lbl" style="margin-top:8px">Position</div>
    <div class="sub" id="drv">+0.0 mm</div>
    <div id="flw-wrap" style="display:none;margin-top:8px">
      <div class="lbl">Follower</div>
      <div class="sub" id="flw-pos">—</div>
    </div>
    <span class="badge idle" id="badge">idle</span>
  </div>

  <div id="leader-ui">
    <div class="card">
      <div class="lbl">Step size</div>
      <div class="row">
        <button class="step-btn" id="s0" onclick="setStep(0)">0.1 mm</button>
        <button class="step-btn on" id="s1" onclick="setStep(1)">1 mm</button>
        <button class="step-btn" id="s2" onclick="setStep(2)">10 mm</button>
      </div>
      <div class="row" style="margin-top:4px">
        <button class="jog-btn" onclick="jog(-1)">&#9664;</button>
        <button class="jog-btn" onclick="jog(1)">&#9654;</button>
      </div>
    </div>

    <div class="card">
      <div class="lbl">Go to position (mm)</div>
      <input type="number" id="goto-mm" value="0" step="0.1" min="-9999" max="9999">
      <button class="btn blue" onclick="doGoto()">&#9654; Go</button>
    </div>

    <div class="card">
      <div class="row">
        <button class="btn amber" id="lock-btn" onclick="toggleLock()" style="margin:0;flex:1">Lock: ON</button>
        <div style="width:8px"></div>
        <button class="btn red" id="rst-btn" style="margin:0;flex:1;position:relative;overflow:hidden"
          onmousedown="holdStart(event)" ontouchstart="holdStart(event)"
          onmouseup="holdCancel()" onmouseleave="holdCancel()"
          ontouchend="holdCancel()" ontouchcancel="holdCancel()">
          <span id="rst-fill" style="position:absolute;left:0;top:0;height:100%;width:0%;background:rgba(255,80,80,.35);transition:none;pointer-events:none"></span>
          <span id="rst-lbl">Hold to reset</span>
        </button>
      </div>
    </div>
  </div>

  <div id="follower-ui" style="display:none">
    <div class="card" style="text-align:center;color:#48a;padding:16px">
      Follower — waiting for leader commands
    </div>
  </div>
</section>

<!-- ── Settings ── -->
<section id="cfg">
  <div class="card">
    <div class="lbl">Device role</div>
    <div class="rad">
      <label><input type="radio" name="role" value="leader" id="r-L"> Leader</label>
      <label><input type="radio" name="role" value="follower" id="r-F"> Follower</label>
    </div>
    <div id="mac-wrap">
      <div class="lbl">Follower MAC</div>
      <input type="text" id="flw-mac" placeholder="AA:BB:CC:DD:EE:FF" maxlength="17">
    </div>
    <button class="btn blue" onclick="saveCfg()">&#128190; Save &amp; Restart</button>
  </div>
  <div class="card">
    <div class="lbl">Device info</div>
    <div class="info" id="dev"></div>
  </div>
</section>

<script>
let isLeader=true,locked=true,stepIdx=1;
const STEP_STEPS=[80,800,8000];

function tab(id,b){
  document.querySelectorAll('section').forEach(s=>s.classList.remove('show'));
  document.querySelectorAll('nav button').forEach(x=>x.classList.remove('on'));
  document.getElementById(id).classList.add('show');b.classList.add('on');
}
function fmm(v){return(v>=0?'+':'')+v.toFixed(1)+' mm';}

async function poll(){
  try{
    const d=await(await fetch('/api/status')).json();
    isLeader=(d.role==='leader');
    locked=!!d.locked;
    stepIdx=d.step_idx||1;

    const t=document.getElementById('role-tag');
    t.textContent=d.role;t.className='tag '+(isLeader?'L':'F');

    const tEl=document.getElementById('tgt');
    tEl.textContent=fmm(d.target_mm||0);
    tEl.className='big'+((d.target_mm||0)<0?' neg':'');
    document.getElementById('drv').textContent=fmm(d.actual_mm||0);

    const fw=document.getElementById('flw-wrap');
    if(isLeader&&d.follower_enabled){
      fw.style.display='block';
      document.getElementById('flw-pos').textContent=
        (d.follower_ok?'✓ ':'✗ ')+fmm((d.follower_pos||0)/800);
    }else{fw.style.display='none';}

    const badge=document.getElementById('badge');
    badge.textContent=d.busy?'moving…':'idle';
    badge.className='badge '+(d.busy?'run':'idle');

    [0,1,2].forEach(i=>{
      const el=document.getElementById('s'+i);
      if(el)el.className='step-btn'+(i===stepIdx?' on':'');
    });

    const lb=document.getElementById('lock-btn');
    if(lb){
      lb.textContent='Lock: '+(locked?'ON':'OFF');
      lb.className='btn '+(locked?'amber':'green');
    }

    document.getElementById('leader-ui').style.display=isLeader?'block':'none';
    document.getElementById('follower-ui').style.display=isLeader?'none':'block';

    document.getElementById('dev').innerHTML=
      'Hostname: <b>'+d.hostname+'</b><br>IP: <b>'+d.ip+'</b><br>MAC: <b>'+d.mac+'</b><br>i2c: <b>'+d.i2c_scan+'</b>';
  }catch(e){}
}

async function setStep(idx){
  stepIdx=idx;
  [0,1,2].forEach(i=>document.getElementById('s'+i).className='step-btn'+(i===idx?' on':''));
  await fetch('/api/set_step',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({idx})});
}
async function jog(dir){
  await fetch('/api/move',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({steps:STEP_STEPS[stepIdx]*dir})});
}
async function doGoto(){
  const mm=parseFloat(document.getElementById('goto-mm').value);
  if(isNaN(mm))return;
  await fetch('/api/goto',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({mm})});
}
async function toggleLock(){
  await fetch('/api/set_lock',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({locked:!locked})});
}
let holdTimer=null,holdRaf=null,holdStart_t=0;
const HOLD_MS=1500;
function holdStart(e){
  e.preventDefault();
  holdStart_t=Date.now();
  const fill=document.getElementById('rst-fill');
  const lbl=document.getElementById('rst-lbl');
  fill.style.transition='none';fill.style.width='0%';
  lbl.textContent='Hold…';
  function frame(){
    const pct=Math.min(100,(Date.now()-holdStart_t)/HOLD_MS*100);
    fill.style.width=pct+'%';
    if(pct<100){holdRaf=requestAnimationFrame(frame);}
    else{holdRaf=null;doReset();}
  }
  holdRaf=requestAnimationFrame(frame);
}
function holdCancel(){
  if(holdRaf){cancelAnimationFrame(holdRaf);holdRaf=null;}
  const fill=document.getElementById('rst-fill');
  const lbl=document.getElementById('rst-lbl');
  fill.style.transition='width .3s ease';fill.style.width='0%';
  lbl.textContent='Hold to reset';
}
async function doReset(){
  document.getElementById('rst-lbl').textContent='✓ Reset!';
  await fetch('/api/reset_pos',{method:'POST'});
  setTimeout(()=>{holdCancel();},800);
}
async function loadCfg(){
  try{
    const c=await(await fetch('/api/config')).json();
    document.getElementById(c.role==='leader'?'r-L':'r-F').checked=true;
    document.getElementById('flw-mac').value=c.follower_mac||'';

    toggleMac(c.role==='leader');
    document.querySelectorAll('input[name=role]').forEach(r=>{
      r.addEventListener('change',()=>toggleMac(r.value==='leader'));
    });
  }catch(e){}
}
function toggleMac(show){document.getElementById('mac-wrap').style.display=show?'block':'none';}
async function saveCfg(){
  const role=document.querySelector('input[name=role]:checked').value;
  const mac=document.getElementById('flw-mac').value.trim();
  const r=await fetch('/api/config',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({role,follower_mac:mac})});
  if((await r.json()).ok)alert('Saved! Restarting…');
}

setInterval(poll,400);poll();loadCfg();
</script>
</body></html>
)html";

// ─────────────────────────────────────────────────────────────────────────────
// Web server
// ─────────────────────────────────────────────────────────────────────────────

static void webserver_init() {
    // GET / — main UI
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    // GET /api/status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["role"]             = (g_role == Role::LEADER) ? "leader" : "follower";
        doc["position"]         = stepper ? stepper->getCurrentPosition() : 0;
        doc["busy"]             = stepper ? stepper->isRunning() : false;
        doc["follower_enabled"] = (g_role == Role::LEADER);
        doc["follower_ok"]      = (bool)g_follower_ok;
        doc["follower_pos"]     = (int32_t)g_follower_pos;
        doc["target_mm"]        = g_target_pos / STEPS_PER_MM;
        doc["actual_mm"]        = (stepper ? stepper->getCurrentPosition() : 0) / STEPS_PER_MM;
        doc["step_idx"]         = g_enc_step_idx;
        doc["locked"]           = g_locked;
        doc["hostname"]         = g_hostname;
        doc["ip"]               = WiFi.localIP().toString();
        doc["mac"]              = WiFi.macAddress();
        doc["i2c_scan"]         = g_i2c_scan;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // GET /api/config
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req) {
        char mac_str[18] = {};
        mac_format(g_follower_mac, mac_str, sizeof(mac_str));
        JsonDocument doc;
        doc["role"]         = (g_role == Role::LEADER) ? "leader" : "follower";
        doc["follower_mac"] = mac_str;

        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /api/config  {"role":"leader","follower_mac":"AA:BB:CC:DD:EE:FF"}
    server.on("/api/config", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            req->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                req->send(400, "application/json", "{\"ok\":false}");
                return;
            }
            const char* role_str = doc["role"] | "leader";
            Role new_role = (strcmp(role_str, "follower") == 0)
                            ? Role::FOLLOWER : Role::LEADER;

            uint8_t new_mac[6] = {};
            const char* mac_str = doc["follower_mac"] | "";
            if (strlen(mac_str) == 17) mac_parse(mac_str, new_mac);

            nvs_save_config(new_role, new_mac);
            req->send(200, "application/json", "{\"ok\":true,\"restart\":true}");
            g_restart_at = millis() + 600;
        }
    );

    // POST /api/move  {"steps":800,"speed":12000}
    server.on("/api/move", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            req->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            if (g_role != Role::LEADER) return;
            if (stepper && stepper->isRunning()) return;
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            const int32_t steps = doc["steps"] | 0;
            if (steps == 0) return;
            g_target_pos += steps;
            if (g_locked) {
                const int32_t lo = (int32_t)(MIN_POS_MM * STEPS_PER_MM);
                const int32_t hi = (int32_t)(MAX_POS_MM * STEPS_PER_MM);
                g_target_pos = constrain(g_target_pos, lo, hi);
            }
        }
    );

    // POST /api/reset_pos
    server.on("/api/reset_pos", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (stepper) stepper->setCurrentPosition(0);
        g_target_pos    = 0;
        g_commanded_pos = 0;
        nvs_save_position(0);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/blink — blink LED to confirm neopixelWrite works after full init
    server.on("/api/blink", HTTP_POST, [](AsyncWebServerRequest* req) {
        for (int i = 0; i < 5; i++) {
            neopixelWrite(48, 0, 60, 0);   // GREEN
            delay(200);
            neopixelWrite(48, 0, 0, 0);
            delay(200);
        }
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/test_step — run a 200-step move via FastAccelStepper (RMT engine)
    server.on("/api/test_step", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!stepper) {
            req->send(503, "application/json", "{\"error\":\"no stepper\"}");
            return;
        }
        if (stepper->isRunning()) {
            req->send(409, "application/json", "{\"error\":\"busy\"}");
            return;
        }
        stepper->setSpeedInHz(1000);
        stepper->setAcceleration(2000);
        stepper->move(200);
        req->send(200, "application/json", "{\"ok\":true,\"steps\":200}");
    });

    // POST /api/set_step  { "idx": 0|1|2 }
    server.on("/api/set_step", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            const int8_t idx = doc["idx"] | 1;
            g_enc_step_idx = constrain(idx, (int8_t)0, (int8_t)(ENC_STEP_COUNT - 1));
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // POST /api/set_lock  { "locked": true|false }
    server.on("/api/set_lock", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            g_locked = doc["locked"] | false;
            if (g_locked) {
                const int32_t lo = (int32_t)(MIN_POS_MM * STEPS_PER_MM);
                const int32_t hi = (int32_t)(MAX_POS_MM * STEPS_PER_MM);
                g_target_pos = constrain(g_target_pos, lo, hi);
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // POST /api/goto  { "mm": 42.5 }
    server.on("/api/goto", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            float mm = doc["mm"] | 0.0f;
            int32_t target = (int32_t)(mm * STEPS_PER_MM);
            if (g_locked) {
                const int32_t lo = (int32_t)(MIN_POS_MM * STEPS_PER_MM);
                const int32_t hi = (int32_t)(MAX_POS_MM * STEPS_PER_MM);
                target = constrain(target, lo, hi);
            }
            g_target_pos = target;
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // SSE log stream — open http://<ip>/log in a browser tab
    g_events.onConnect([](AsyncEventSourceClient* c) {
        c->send("connected", "log", millis());
    });
    server.addHandler(&g_events);

    server.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html",
            "<!doctype html><html><head>"
            "<meta charset=utf-8>"
            "<title>Log</title>"
            "<style>"
            "body{background:#111;color:#0f0;font:13px/1.4 monospace;margin:0;padding:8px}"
            "#log{white-space:pre-wrap;word-break:break-all}"
            "</style></head><body>"
            "<b id=host></b> &mdash; live log "
            "(<a href=/ style=color:#08f>&#8592; back</a>)"
            "<hr><div id=log></div>"
            "<script>"
            "document.getElementById('host').textContent=location.host;"
            "const d=document.getElementById('log');"
            "const es=new EventSource('/events');"
            "es.addEventListener('log',e=>{"
            "  d.textContent+=e.data;"
            "  if(!d.textContent.endsWith('\n'))d.textContent+='\n';"
            "  window.scrollTo(0,document.body.scrollHeight);"
            "});"
            "</script></body></html>"
        );
    });

    server.begin();
    Serial.println("[web] server started on :80  log→ http://<ip>/log");
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// Rotary encoder ISRs
// ─────────────────────────────────────────────────────────────────────────────
// Full quadrature state machine — fires on CHANGE of both CLK and DT.
// Accumulates partial-step deltas and emits exactly ±1 per physical detent.
// Eliminates the timing race of reading DT at CLK-fall (which caused
// reliable CW but unreliable CCW with the old single-edge approach).

static void IRAM_ATTR enc_isr() {
    static uint8_t state = 3;   // start at rest: CLK=HIGH, DT=HIGH
    static int8_t  acc   = 0;   // partial-step accumulator

    const uint8_t s = (uint8_t)((digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT));

    // Transition delta table — rows = previous 2-bit state, cols = current
    // CW  full sequence: 3→1→0→2→3  (+4 accumulated)
    // CCW full sequence: 3→2→0→1→3  (-4 accumulated)
    static const int8_t T[4][4] = {
        {  0, -1, +1,  0 },   // prev=00
        { +1,  0,  0, -1 },   // prev=01
        { -1,  0,  0, +1 },   // prev=10
        {  0, +1, -1,  0 },   // prev=11 (detent)
    };

    acc  += T[state][s];
    state = s;

    // Emit exactly one tick when the encoder snaps back to the detent position
    if (s == 3) {
        if      (acc > 0) g_enc_delta += 1;
        else if (acc < 0) g_enc_delta -= 1;
        acc = 0;
    }
}

static void IRAM_ATTR enc_sw_isr() {
    g_enc_btn_raw = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// LED stage helper — brief coloured flash to show boot progress
// ─────────────────────────────────────────────────────────────────────────────
static void led_stage(uint8_t r, uint8_t g, uint8_t b, uint32_t ms = 300) {
    neopixelWrite(48, r, g, b);
    delay(ms);
    neopixelWrite(48, 0, 0, 0);
    delay(100);
}

void setup() {
    Serial.begin(115200);
    delay(300);

    // ── STAGE 1: Green flash — setup() started ────────────────────────────────
    led_stage(0, 40, 0);   // GREEN

    // Load config + stored position from NVS
    int32_t stored_pos = 0;
    nvs_load(g_role, stored_pos, g_follower_mac);
    Serial.printf("[nvs] role=%s  pos=%ld\n",
        g_role == Role::LEADER ? "leader" : "follower", (long)stored_pos);

    // ── STAGE 2: Yellow flash — NVS loaded ───────────────────────────────────
    led_stage(40, 40, 0);  // YELLOW

    // I2C scan — find what's on the display bus before U8g2 takes the pins
    {
        Wire.begin(PIN_SDA, PIN_SCL);
        char buf[64] = {};
        int  n = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                int len = strlen(buf);
                snprintf(buf + len, sizeof(buf) - len, "%s0x%02X", n ? "," : "", addr);
                n++;
            }
        }
        Wire.end();
        strncpy(g_i2c_scan, n ? buf : "none found", sizeof(g_i2c_scan) - 1);
        Serial.printf("[i2c] scan: %s\n", g_i2c_scan);
    }

    // Display — SW_I2C, no Wire.begin() needed
    bool disp_ok = display.begin();
    Serial.printf("[disp] begin=%s\n", disp_ok ? "ok" : "FAILED");
    if (disp_ok) {
        display.setFont(u8g2_font_6x10_tf);
        display.clearBuffer();
        display.drawStr(0, 10, "Slider booting...");
        display.drawStr(0, 24, g_role == Role::LEADER ? "role: LEADER" : "role: FOLLOWER");
        display.sendBuffer();
    }

    // ── STAGE 3: Cyan flash — display init done ───────────────────────────────
    led_stage(0, 40, 40);  // CYAN

    // WiFi
    WiFi.mode(WIFI_STA);
    // Disable WiFi power-save — without this the radio sleeps periodically and
    // ESP-NOW packets are dropped (TX FAILED status=1 / silent receive loss).
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[wifi] connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
    Serial.printf("\n[wifi] IP=%s  MAC=%s\n",
        WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());

    // ── STAGE 4: Blue flash — WiFi connected ─────────────────────────────────
    led_stage(0, 0, 40);   // BLUE

    // Derive unique hostname from last 3 MAC bytes
    {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        snprintf(g_hostname, sizeof(g_hostname),
            "slider-%02x%02x%02x", mac[3], mac[4], mac[5]);
    }
    Serial.printf("[ota] hostname: %s\n", g_hostname);
    Serial.printf(">>> This device MAC: %s <<<\n", WiFi.macAddress().c_str());

    // mDNS
    MDNS.begin(g_hostname);

    // OTA
    ArduinoOTA.setHostname(g_hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]()  { Serial.println("[ota] start"); });
    ArduinoOTA.onEnd([]()    { Serial.println("\n[ota] done"); });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        Serial.printf("[ota] %u%%\r", done * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[ota] error %u\n", e);
    });
    ArduinoOTA.begin();

    // Telnet log server
    g_telnet.begin();
    g_telnet.setNoDelay(true);
    net_log("[telnet] log server on port 23\n");
    net_log("[telnet] pio device monitor --port socket://%s:23\n",
        WiFi.localIP().toString().c_str());

    // UDP sync — init broadcast address and open socket
    udp_init_bcast();
    udp_init();

    // SPREAD pin — LOW = StealthChop (quiet), HIGH = SpreadCycle (more torque)
    pinMode(PIN_SPREAD, OUTPUT);
    digitalWrite(PIN_SPREAD, LOW);

    // ── STAGE 5: Magenta flash — starting stepper init ────────────────────────
    led_stage(40, 0, 40);  // MAGENTA

    // Stepper — RMT engine (same peripheral as FluidNC)
    engine.init();
    stepper = engine.stepperConnectToPin(PIN_STEP);
    if (!stepper) {
        Serial.println("[stepper] ERROR: could not bind to step pin");
        // RED = stepper init failed — stays on so you can see it
        neopixelWrite(48, 60, 0, 0);
    } else {
        stepper->setDirectionPin(PIN_DIR);
        stepper->setEnablePin(PIN_ENABLE, true);   // true = low_active: LOW enables TMC2209
        stepper->setAutoEnable(false);             // never auto-disable — hold position always
        stepper->enableOutputs();                  // drive EN LOW now and keep it there
        stepper->setSpeedInHz(SPEED_MAX_HZ);
        stepper->setAcceleration(ACCEL_DEFAULT);
        stepper->setCurrentPosition(stored_pos);
        g_target_pos    = stored_pos;   // target starts at restored position
        g_commanded_pos = stored_pos;
        Serial.printf("[stepper] RMT ready, position restored to %ld (%.1f mm)\n",
            (long)stored_pos, stored_pos / STEPS_PER_MM);

        // ── STAGE 6: White flash — stepper ready ─────────────────────────────
        led_stage(40, 40, 40, 500);  // WHITE (longer so it's easy to see)
    }

    // Rotary encoder
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    pinMode(PIN_ENC_SW,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), enc_isr,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  enc_isr,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_SW),  enc_sw_isr, FALLING);
    Serial.println("[enc] rotary encoder ready");

    // Web server
    webserver_init();

    display_update();
    Serial.printf("[boot] complete — http://%s.local\n", g_hostname);
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    ArduinoOTA.handle();  // always first
    udp_poll();   // receive
    udp_tick();   // retry + hello + heartbeat

    // Accept new telnet connections
    if (g_telnet.hasClient()) {
        WiFiClient nc = g_telnet.accept();
        bool placed = false;
        for (auto& c : g_telnet_clients) {
            if (!c || !c.connected()) { c = nc; placed = true; break; }
        }
        if (!placed) nc.stop();  // all slots busy
    }

    // Pending restart (triggered by /api/config save)
    if (g_restart_at && millis() >= g_restart_at) {
        g_restart_at = 0;
        Serial.println("[sys] restarting…");
        delay(100);
        ESP.restart();
    }

    // ── Rotary encoder state machine ─────────────────────────────────────────
    {
        const uint32_t now_ms = millis();

        // Consume encoder delta (ISR accumulates, we drain here)
        const int32_t delta = g_enc_delta;
        if (delta) g_enc_delta = 0;

        // Debounced button press
        bool btn_pressed = false;
        if (g_enc_btn_raw && (now_ms - g_enc_btn_ms > 250)) {
            g_enc_btn_raw = false;
            g_enc_btn_ms  = now_ms;
            btn_pressed   = true;
        }

        if (g_ui_state == UIState::MENU) {
            // ── Menu mode ──────────────────────────────────────────────────
            if (delta) {
                // Move selection — wraps: last→first and first→last
                g_menu_sel = (int8_t)((g_menu_sel + MENU_COUNT + (delta > 0 ? 1 : -1)) % MENU_COUNT);
                // Keep selection inside the visible window
                if (g_menu_sel < g_menu_scroll)
                    g_menu_scroll = g_menu_sel;
                else if (g_menu_sel >= g_menu_scroll + MENU_VISIBLE)
                    g_menu_scroll = g_menu_sel - MENU_VISIBLE + 1;
            }
            if (btn_pressed) {
                if (g_menu_sel < ENC_STEP_COUNT) {
                    // Step-size selector
                    g_enc_step_idx = g_menu_sel;
                    Serial.printf("[menu] step size: %s\n", ENC_STEP_LABELS[g_enc_step_idx]);
                } else if (g_menu_sel == MENU_LOCK) {
                    g_locked = !g_locked;
                    Serial.printf("[menu] lock: %s\n", g_locked ? "ON" : "OFF");
                    // Stay in menu so user can see the updated state — no exit
                } else if (g_menu_sel == MENU_RESET) {
                    if (stepper) stepper->setCurrentPosition(0);
                    g_target_pos    = 0;
                    g_commanded_pos = 0;
                    nvs_save_position(0);
                    Serial.println("[menu] position reset to 0");
                }
                // Return to normal for everything except toggle/info items
                if (g_menu_sel != MENU_LOCK &&
                    g_menu_sel != MENU_IP   &&
                    g_menu_sel != MENU_MAC) {
                    g_ui_state = UIState::NORMAL;
                    Serial.println("[menu] exit");
                }
            }
        } else {
            // ── Normal mode ────────────────────────────────────────────────
            if (btn_pressed) {
                // Open menu
                g_menu_sel    = 0;
                g_menu_scroll = 0;
                g_ui_state    = UIState::MENU;
                Serial.println("[menu] open");
            } else if (delta) {
                // Always accumulate target — motor chases it even while moving
                g_target_pos += delta * ENC_STEP_SIZES[g_enc_step_idx];
                if (g_locked) {
                    const int32_t lo = (int32_t)(MIN_POS_MM * STEPS_PER_MM);
                    const int32_t hi = (int32_t)(MAX_POS_MM * STEPS_PER_MM);
                    g_target_pos = constrain(g_target_pos, lo, hi);
                }
                net_log("[enc] target → %+ld steps (%.1f mm)\n",
                    (long)g_target_pos, g_target_pos / STEPS_PER_MM);
            }
        }
    }

    // Drive motor toward target whenever target changed
    if (stepper && g_target_pos != g_commanded_pos) {
        g_commanded_pos = g_target_pos;
        stepper->setSpeedInHz(SPEED_MAX_HZ);
        stepper->setAcceleration(ACCEL_DEFAULT);
        stepper->moveTo(g_commanded_pos);
        if (g_role == Role::LEADER) {
            // Send absolute target so follower is always in sync,
            // even if packets arrive while the motor is mid-move.
            udp_send_move(g_commanded_pos, SPEED_MAX_HZ);
        }
        net_log("[move] → %+ld steps (%.1f mm) stepper=%s running=%s\n",
            (long)g_commanded_pos, g_commanded_pos / STEPS_PER_MM,
            stepper ? "ok" : "NULL",
            stepper ? (stepper->isRunning() ? "yes" : "no") : "n/a");
    }

    // Detect end-of-move → persist position to NVS
    const bool running = stepper && stepper->isRunning();
    if (g_was_running && !running) {
        const int32_t pos = stepper->getCurrentPosition();
        nvs_save_position(pos);
        net_log("[nvs] position saved: %ld (%.1f mm)\n",
            (long)pos, pos / STEPS_PER_MM);
    }
    g_was_running = running;

    // Refresh display
    const uint32_t now = millis();
    if (now - g_last_disp_ms >= 250) {
        g_last_disp_ms = now;
        display_update();
    }
}

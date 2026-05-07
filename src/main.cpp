#include "globals.h"
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// ── Module-private ────────────────────────────────────────────────────────────
static FastAccelStepperEngine engine = FastAccelStepperEngine();

// ── Caliper ───────────────────────────────────────────────────────────────────
DigitalCaliper g_caliper(PIN_CALIPER_CLK, PIN_CALIPER_DATA);

// Telnet log server — pio device monitor --port socket://<ip>:23
static WiFiServer g_telnet(23);
static WiFiClient g_telnet_clients[2];

// ── Globals defined here (extern'd in globals.h) ──────────────────────────────
char              g_hostname[24]   = {};
char              g_i2c_scan[64]   = "not scanned";
FastAccelStepper* stepper          = nullptr;
bool              g_was_running    = false;
volatile int32_t  g_target_pos     = 0;
int32_t           g_commanded_pos  = 0;
int8_t            g_enc_step_idx   = 1;       // default: 1 mm per click
bool              g_locked         = true;
UIState           g_ui_state       = UIState::NORMAL;
int8_t            g_menu_sel       = 0;
int8_t            g_menu_scroll    = 0;
uint32_t          g_restart_at     = 0;
AsyncEventSource  g_events("/events");

// Encoder step sizes — arrays declared extern in globals.h
const int32_t ENC_STEP_SIZES[ENC_STEP_COUNT]  = { 80, 800, 8000 };  // 0.1 mm, 1 mm, 10 mm
const char*   ENC_STEP_LABELS[ENC_STEP_COUNT] = { "0.1 mm", "1 mm", "10 mm" };

// ── net_log — Serial + SSE + Telnet ──────────────────────────────────────────
void net_log(const char* fmt, ...) {
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

// ── LED stage helper — brief coloured flash to show boot progress ─────────────
static void led_stage(uint8_t r, uint8_t g, uint8_t b, uint32_t ms = 300) {
    neopixelWrite(48, r, g, b);
    delay(ms);
    neopixelWrite(48, 0, 0, 0);
    delay(100);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);

    // STAGE 1: Green flash — setup() started
    led_stage(0, 40, 0);

    // Load stored position and transport from NVS
    const int32_t stored_pos = nvs_load_position();
    const Transport stored_transport = nvs_load_transport();
    Serial.printf("[nvs] pos=%ld  transport=%s\n",
        (long)stored_pos,
        stored_transport == Transport::ESPNOW ? "ESPNOW" : "UDP");

    // STAGE 2: Yellow flash — NVS loaded
    led_stage(40, 40, 0);

    // I2C scan — find what's on the display bus before U8g2 takes the pins
    {
        Wire.begin(PIN_SDA, PIN_SCL);
        char buf[64] = {};
        int  n = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                const int len = strlen(buf);
                snprintf(buf + len, sizeof(buf) - len, "%s0x%02X", n ? "," : "", addr);
                n++;
            }
        }
        Wire.end();
        strncpy(g_i2c_scan, n ? buf : "none found", sizeof(g_i2c_scan) - 1);
        Serial.printf("[i2c] scan: %s\n", g_i2c_scan);
    }

    // Display (SW_I2C — no Wire.begin() needed)
    display_setup();

    // STAGE 3: Cyan flash — display init done
    led_stage(0, 40, 40);

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[wifi] connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
    Serial.printf("\n[wifi] IP=%s  MAC=%s\n",
        WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());

    // STAGE 4: Blue flash — WiFi connected
    led_stage(0, 0, 40);

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

    // UDP transport — always init UDP (hello window, broadcast subnet)
    udp_init_bcast();
    udp_init();

    // Restore transport; start ESP-NOW if it was selected
    g_transport = stored_transport;
    if (g_transport == Transport::ESPNOW) {
        espnow_init();
    }

    // SPREAD pin — LOW = StealthChop (quiet), HIGH = SpreadCycle (more torque)
    pinMode(PIN_SPREAD, OUTPUT);
    digitalWrite(PIN_SPREAD, LOW);

    // STAGE 5: Magenta flash — stepper init
    led_stage(40, 0, 40);

    // Stepper — RMT engine
    engine.init();
    stepper = engine.stepperConnectToPin(PIN_STEP);
    if (!stepper) {
        Serial.println("[stepper] ERROR: could not bind to step pin");
        neopixelWrite(48, 60, 0, 0);  // RED — stay on
    } else {
        stepper->setDirectionPin(PIN_DIR);
        stepper->setEnablePin(PIN_ENABLE, true);   // true = low_active
        stepper->setAutoEnable(false);
        stepper->enableOutputs();
        stepper->setSpeedInHz(SPEED_MAX_HZ);
        stepper->setAcceleration(ACCEL_DEFAULT);
        stepper->setCurrentPosition(stored_pos);
        g_target_pos    = stored_pos;
        g_commanded_pos = stored_pos;
        Serial.printf("[stepper] RMT ready, position restored to %ld (%.1f mm)\n",
            (long)stored_pos, stored_pos / STEPS_PER_MM);

        // STAGE 6: White flash — stepper ready
        led_stage(40, 40, 40, 500);
    }

    // Rotary encoder
    encoder_setup();
    Serial.println("[enc] rotary encoder ready");

    // Digital caliper
    g_caliper.begin();
    Serial.println("[caliper] ISR ready on pins 7/15");

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

    // Transport-gated receive poll
    if (g_transport == Transport::ESPNOW) {
        espnow_poll();
    } else {
        udp_poll();
    }
    udp_sync_tick();      // hello window, progress broadcast, timeout

    // Accept new telnet connections
    if (g_telnet.hasClient()) {
        WiFiClient nc = g_telnet.accept();
        bool placed = false;
        for (auto& c : g_telnet_clients) {
            if (!c || !c.connected()) { c = nc; placed = true; break; }
        }
        if (!placed) nc.stop();
    }

    // Pending restart
    if (g_restart_at && millis() >= g_restart_at) {
        g_restart_at = 0;
        Serial.println("[sys] restarting…");
        delay(100);
        ESP.restart();
    }

    // ── Rotary encoder state machine ──────────────────────────────────────────
    {
        const uint32_t now_ms = millis();

        // Drain: subtract what we read so any ISR tick between read and subtract
        // stays in g_enc_delta rather than being silently zeroed out.
        const int32_t delta = g_enc_delta;
        if (delta) g_enc_delta -= delta;

        // Debounced button press
        bool btn_pressed = false;
        if (g_enc_btn_raw && (now_ms - g_enc_btn_ms > 250)) {
            g_enc_btn_raw = false;
            g_enc_btn_ms  = now_ms;
            btn_pressed   = true;
        }

        if (g_ui_state == UIState::MENU) {
            // ── Menu mode ─────────────────────────────────────────────────────
            if (delta) {
                g_menu_sel = (int8_t)((g_menu_sel + MENU_COUNT + (delta > 0 ? 1 : -1)) % MENU_COUNT);
                if (g_menu_sel < g_menu_scroll)
                    g_menu_scroll = g_menu_sel;
                else if (g_menu_sel >= g_menu_scroll + MENU_VISIBLE)
                    g_menu_scroll = g_menu_sel - MENU_VISIBLE + 1;
            }
            if (btn_pressed) {
                if (g_menu_sel < ENC_STEP_COUNT) {
                    g_enc_step_idx = g_menu_sel;
                    Serial.printf("[menu] step size: %s\n", ENC_STEP_LABELS[g_enc_step_idx]);
                } else if (g_menu_sel == MENU_LOCK) {
                    g_locked = !g_locked;
                    Serial.printf("[menu] lock: %s\n", g_locked ? "ON" : "OFF");
                } else if (g_menu_sel == MENU_RESET) {
                    if (stepper) stepper->setCurrentPosition(0);
                    g_target_pos    = 0;
                    g_commanded_pos = 0;
                    nvs_save_position(0);
                    Serial.println("[menu] position reset to 0");
                } else if (g_menu_sel == MENU_TRANSPORT) {
                    // Toggle transport and persist
                    if (g_transport == Transport::UDP) {
                        g_transport = Transport::ESPNOW;
                        espnow_init();
                    } else {
                        g_transport = Transport::UDP;
                        espnow_deinit();
                    }
                    nvs_save_transport(g_transport);
                    net_log("[menu] transport → %s\n",
                        g_transport == Transport::ESPNOW ? "ESPNOW" : "UDP");
                }
                // Return to normal for everything except info-only items
                if (g_menu_sel != MENU_LOCK      &&
                    g_menu_sel != MENU_TRANSPORT  &&
                    g_menu_sel != MENU_IP         &&
                    g_menu_sel != MENU_MAC) {
                    g_ui_state = UIState::NORMAL;
                    Serial.println("[menu] exit");
                }
            }
        } else {
            // ── Normal mode ───────────────────────────────────────────────────
            if (btn_pressed) {
                g_menu_sel    = 0;
                g_menu_scroll = 0;
                g_ui_state    = UIState::MENU;
                Serial.println("[menu] open");
            } else if (delta) {
                // Always accumulate — motor drive section decides what to do
                // based on sync state (initiate, update in-place, or ignore as peer).
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

    // ── Motor drive: apply target changes ────────────────────────────────────
    if (stepper && g_target_pos != g_commanded_pos) {
        if (g_sync_state == SyncState::IDLE     ||
            g_sync_state == SyncState::WAITING_PEER ||
            g_sync_state == SyncState::FAILED) {
            // Start a new transaction
            g_commanded_pos    = g_target_pos;
            g_txid             = (uint16_t)esp_random();
            g_is_initiator     = true;
            g_sync_state       = SyncState::MOVING;
            g_progress_last_ms = millis();
            g_sync_timeout_ms  = 0;

            SyncMove pkt{};
            pkt.txid     = g_txid;
            pkt.target   = g_commanded_pos;
            pkt.speed_hz = SPEED_MAX_HZ;
            sync_broadcast(&pkt, sizeof(pkt));

            stepper->setSpeedInHz(SPEED_MAX_HZ);
            stepper->setAcceleration(ACCEL_DEFAULT);
            stepper->moveTo(g_commanded_pos);

            net_log("[sync] init txid=%u tgt=%ld (%.1f mm)\n",
                g_txid, (long)g_commanded_pos, g_commanded_pos / STEPS_PER_MM);

        } else if (g_sync_state == SyncState::MOVING && g_is_initiator) {
            // Already moving as initiator — update target in-place, re-broadcast
            g_commanded_pos = g_target_pos;

            SyncMove pkt{};
            pkt.txid     = g_txid;
            pkt.target   = g_commanded_pos;
            pkt.speed_hz = SPEED_MAX_HZ;
            sync_broadcast(&pkt, sizeof(pkt));

            stepper->moveTo(g_commanded_pos);

            net_log("[sync] update txid=%u tgt=%ld (%.1f mm)\n",
                g_txid, (long)g_commanded_pos, g_commanded_pos / STEPS_PER_MM);
        }
        // else: MOVING as peer — peer drives this move, ignore local encoder
    }

    // ── Detect end-of-move → send ARRIVED, persist position ──────────────────
    const bool running = stepper && stepper->isRunning();
    if (g_was_running && !running) {
        const int32_t pos = stepper->getCurrentPosition();
        nvs_save_position(pos);
        net_log("[nvs] position saved: %ld (%.1f mm)\n", (long)pos, pos / STEPS_PER_MM);

        if (g_sync_state == SyncState::MOVING) {
            SyncArrived pkt{};
            pkt.txid = g_txid;
            pkt.pos  = pos;
            sync_broadcast(&pkt, sizeof(pkt));
            net_log("[sync] ARRIVED txid=%u pos=%ld\n", g_txid, (long)pos);
            g_sync_state      = SyncState::WAITING_PEER;
            g_sync_timeout_ms = millis() + 5000;
        }
    }
    g_was_running = running;

    // Display — self-throttled to 4 Hz inside display_update()
    display_update();

    // ── Caliper diagnostic — log every 2 s ───────────────────────────────────
    {
        static uint32_t s_last_cal_log  = 0;
        static uint32_t s_last_pkt_seen = 0;
        const  uint32_t now_ms          = millis();
        if (now_ms - s_last_cal_log >= 2000) {
            s_last_cal_log = now_ms;
            const uint32_t pkts = g_caliper.packet_count;
            const uint32_t rate = pkts - s_last_pkt_seen;
            s_last_pkt_seen = pkts;
            net_log("[caliper] pkts=%lu (+%lu/2s) bit_idx=%u acc=0x%06lX\n",
                (unsigned long)pkts,
                (unsigned long)rate,
                (unsigned)g_caliper.bit_idx,
                (unsigned long)g_caliper.accumulator);
        }
    }
}

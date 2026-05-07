#pragma once
// ── Shared declarations — included by every translation unit ─────────────────
// Defines: constants, enums, extern globals, cross-module function prototypes.

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <ESPAsyncWebServer.h>
#include "MoveCmd.h"
#include "Config.h"
#include "DigitalCaliper.h"

// ── Pin assignments ───────────────────────────────────────────────────────────
inline constexpr uint8_t PIN_CALIPER_CLK  = 15;
inline constexpr uint8_t PIN_CALIPER_DATA =  7;
inline constexpr uint8_t PIN_STEP    = 13;
inline constexpr uint8_t PIN_DIR     = 14;
inline constexpr uint8_t PIN_ENABLE  = 10;
inline constexpr uint8_t PIN_UART_TX =  9;
inline constexpr uint8_t PIN_UART_RX = 12;
inline constexpr uint8_t PIN_SPREAD  = 11;
inline constexpr uint8_t PIN_SDA     = 42;
inline constexpr uint8_t PIN_SCL     = 41;
inline constexpr uint8_t PIN_ENC_CLK =  4;
inline constexpr uint8_t PIN_ENC_DT  =  5;
inline constexpr uint8_t PIN_ENC_SW  =  6;

// ── Motion calibration ────────────────────────────────────────────────────────
inline constexpr float    STEPS_PER_MM  = 800.0f;
inline constexpr float    MAX_POS_MM    = 190.0f;
inline constexpr float    MIN_POS_MM    =   0.0f;
inline constexpr uint32_t SPEED_MAX_HZ  = 12000;
inline constexpr uint32_t ACCEL_DEFAULT = 25000;

// ── Encoder step sizes ────────────────────────────────────────────────────────
inline constexpr int8_t ENC_STEP_COUNT = 3;
extern const int32_t    ENC_STEP_SIZES[ENC_STEP_COUNT];   // steps per click
extern const char*      ENC_STEP_LABELS[ENC_STEP_COUNT];  // display labels

// ── NVS keys ─────────────────────────────────────────────────────────────────
inline constexpr char NVS_NS[]        = "slider";
inline constexpr char NVS_POS[]       = "pos";
inline constexpr char NVS_TRANSPORT[] = "transport";

// ── UI enums ──────────────────────────────────────────────────────────────────
enum class UIState : uint8_t { NORMAL = 0, MENU = 1 };

// Menu layout:  0-2 step sizes | 3 lock | 4 reset | 5 IP | 6 MAC | 7 transport | 8 exit
inline constexpr int8_t MENU_LOCK      = 3;
inline constexpr int8_t MENU_RESET     = 4;
inline constexpr int8_t MENU_IP        = 5;
inline constexpr int8_t MENU_MAC       = 6;
inline constexpr int8_t MENU_TRANSPORT = 7;
inline constexpr int8_t MENU_EXIT      = 8;
inline constexpr int8_t MENU_COUNT     = 9;
inline constexpr int8_t MENU_VISIBLE   = 4;

// ── Sync protocol ─────────────────────────────────────────────────────────────
enum class SyncState : uint8_t {
    IDLE         = 0,
    MOVING       = 1,
    WAITING_PEER = 2,
    FAILED       = 3,
};
enum class Transport : uint8_t { UDP = 0, ESPNOW = 1 };

// ── Extern globals ────────────────────────────────────────────────────────────
// defined in main.cpp
extern DigitalCaliper     g_caliper;
extern char               g_hostname[24];
extern char               g_i2c_scan[64];
extern FastAccelStepper*  stepper;
extern bool               g_was_running;
extern volatile int32_t   g_target_pos;
extern int32_t            g_commanded_pos;
extern int8_t             g_enc_step_idx;
extern bool               g_locked;
extern UIState            g_ui_state;
extern int8_t             g_menu_sel;
extern int8_t             g_menu_scroll;
extern uint32_t           g_restart_at;
extern AsyncEventSource   g_events;

// defined in encoder.cpp
extern volatile int32_t   g_enc_delta;
extern volatile bool      g_enc_btn_raw;
extern uint32_t           g_enc_btn_ms;

// defined in sync.cpp
extern SyncState          g_sync_state;
extern uint16_t           g_txid;
extern bool               g_is_initiator;
extern int32_t            g_peer_pos;
extern uint32_t           g_sync_timeout_ms;
extern uint32_t           g_progress_last_ms;
extern Transport          g_transport;

// ── Cross-module function prototypes ─────────────────────────────────────────

// log.cpp (net_log is defined in main.cpp)
void net_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// nvs_store.cpp
void     nvs_save_position(int32_t pos);
int32_t  nvs_load_position();
void     nvs_save_transport(Transport t);
Transport nvs_load_transport();

// sync.cpp
void udp_init_bcast();
void udp_init();
void udp_poll();
void udp_sync_tick();
void sync_broadcast(const void* data, size_t len);
void espnow_init();
void espnow_deinit();
void espnow_poll();

// display.cpp
void display_setup();
void display_update();

// webui.cpp
void webserver_init();

// encoder.cpp
void encoder_setup();

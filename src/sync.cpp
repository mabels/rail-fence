#include "globals.h"
#include <WiFiUdp.h>
#include <esp_now.h>
#include <soc/gpio_struct.h>

// ── Module-private constants ──────────────────────────────────────────────────
static constexpr uint32_t HELLO_WINDOW_MS      = 6000;
static constexpr uint32_t HELLO_INTERVAL_MS    =  500;
static constexpr uint32_t PROGRESS_INTERVAL_MS =  200;
static constexpr uint32_t WAITING_PEER_TIMEOUT = 5000;
static constexpr uint16_t UDP_PORT             = 4210;

// ── Module globals (extern'd in globals.h) ────────────────────────────────────
SyncState g_sync_state       = SyncState::IDLE;
uint16_t  g_txid             = 0;
bool      g_is_initiator     = false;
int32_t   g_peer_pos         = 0;
uint32_t  g_sync_timeout_ms  = 0;
uint32_t  g_progress_last_ms = 0;
Transport g_transport        = Transport::UDP;

// ── Module-private state ──────────────────────────────────────────────────────
static IPAddress g_bcast_ip;
static uint32_t  g_hello_until_ms = 0;
static uint32_t  g_hello_last_ms  = 0;

// UDP sockets — split RX/TX to prevent beginPacket() clobbering receive buffer
static WiFiUDP g_udp_rx;
static WiFiUDP g_udp_tx;

// ESP-NOW broadcast address
static const uint8_t ESPNOW_BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW receive buffer — callback runs on WiFi task (core 0),
// loop runs on app core (core 1). Single-slot: drop packet if unconsumed.
static struct {
    uint8_t       data[32];
    uint8_t       len;
    volatile bool ready;
} g_espnow_rx = {};

// ── UDP helpers ───────────────────────────────────────────────────────────────

void udp_init_bcast() {
    IPAddress local = WiFi.localIP();
    IPAddress mask  = WiFi.subnetMask();
    for (int i = 0; i < 4; i++)
        g_bcast_ip[i] = (local[i] & mask[i]) | (uint8_t)(~mask[i] & 0xFF);
    net_log("[udp] subnet broadcast: %s\n", g_bcast_ip.toString().c_str());
}

static void udp_broadcast(const void* data, size_t len) {
    const IPAddress& dst = (g_bcast_ip != IPAddress(0,0,0,0))
                           ? g_bcast_ip : IPAddress(255,255,255,255);
    g_udp_tx.beginPacket(dst, UDP_PORT);
    g_udp_tx.write(reinterpret_cast<const uint8_t*>(data), len);
    g_udp_tx.endPacket();
}

// ── ESP-NOW helpers ───────────────────────────────────────────────────────────

// Receive callback — runs on WiFi task, NOT app core.
// Only safe operations: memcpy + volatile flag write.
static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len) {
    if (g_espnow_rx.ready) return;   // previous packet not consumed — drop
    if (len < 1 || len > (int)sizeof(g_espnow_rx.data)) return;
    memcpy(g_espnow_rx.data, data, len);
    g_espnow_rx.len   = (uint8_t)len;
    g_espnow_rx.ready = true;        // publish last — acts as release barrier
}

void espnow_init() {
    if (esp_now_init() != ESP_OK) {
        net_log("[espnow] init FAILED\n");
        return;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ESPNOW_BCAST, 6);
    peer.channel = 0;       // use current WiFi channel
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        net_log("[espnow] add_peer FAILED\n");
        return;
    }
    esp_now_register_recv_cb(espnow_recv_cb);
    net_log("[espnow] ready — channel %d  broadcast %02X:%02X:%02X:%02X:%02X:%02X\n",
        WiFi.channel(),
        ESPNOW_BCAST[0], ESPNOW_BCAST[1], ESPNOW_BCAST[2],
        ESPNOW_BCAST[3], ESPNOW_BCAST[4], ESPNOW_BCAST[5]);
}

void espnow_deinit() {
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    g_espnow_rx.ready = false;
    net_log("[espnow] stopped\n");
}

// ── Unified broadcast (dispatches to active transport) ────────────────────────
void sync_broadcast(const void* data, size_t len) {
    if (g_transport == Transport::ESPNOW) {
        esp_now_send(ESPNOW_BCAST, reinterpret_cast<const uint8_t*>(data), len);
    } else {
        udp_broadcast(data, len);
    }
}

// ── Hello / startup position sync ────────────────────────────────────────────

static void udp_send_hello() {
    HelloPkt h{};
    h.uptime_ms = millis();
    h.position  = stepper ? stepper->getCurrentPosition() : g_target_pos;
    sync_broadcast(&h, sizeof(h));
}

static void udp_apply_position(int32_t pos) {
    if (stepper) stepper->setCurrentPosition(pos);
    g_target_pos    = pos;
    g_commanded_pos = pos;
    nvs_save_position(pos);
    net_log("[sync] adopted peer position: %ld steps (%.1f mm)\n",
        (long)pos, pos / STEPS_PER_MM);
}

static void udp_on_hello(const HelloPkt& h, const IPAddress& src) {
    net_log("[sync] hello from %s uptime=%lums pos=%ld\n",
        src.toString().c_str(), (unsigned long)h.uptime_ms, (long)h.position);
    if (h.uptime_ms > millis()) {
        const int32_t my_pos = stepper ? stepper->getCurrentPosition() : g_target_pos;
        if (h.position != my_pos) {
            net_log("[sync] peer uptime higher — adopting pos %ld\n", (long)h.position);
            udp_apply_position(h.position);
        }
    }
}

// ── Sync packet handlers ──────────────────────────────────────────────────────

static void on_sync_move(const SyncMove& pkt) {
    if (g_sync_state == SyncState::MOVING && pkt.txid != g_txid) {
        net_log("[sync] CONFLICT our txid=%u vs rx txid=%u\n", g_txid, pkt.txid);
        SyncFail fail{};
        fail.txid   = g_txid;
        fail.reason = 1;
        sync_broadcast(&fail, sizeof(fail));
        if (stepper) stepper->stopMove();
        const int32_t pos = stepper ? stepper->getCurrentPosition() : g_commanded_pos;
        g_target_pos    = pos;
        g_commanded_pos = pos;
        g_sync_state    = SyncState::IDLE;
        return;
    }
    if (g_sync_state == SyncState::MOVING && pkt.txid == g_txid) return;

    g_txid             = pkt.txid;
    g_is_initiator     = false;
    g_sync_state       = SyncState::MOVING;
    g_progress_last_ms = millis();
    g_sync_timeout_ms  = 0;

    const uint32_t spd = pkt.speed_hz ? pkt.speed_hz : SPEED_MAX_HZ;
    if (stepper) {
        stepper->setSpeedInHz(spd);
        stepper->setAcceleration(ACCEL_DEFAULT);
        stepper->moveTo(pkt.target);
    }
    g_target_pos    = pkt.target;
    g_commanded_pos = pkt.target;
    net_log("[sync] rx MOVE txid=%u tgt=%ld (%.1f mm)\n",
        pkt.txid, (long)pkt.target, pkt.target / STEPS_PER_MM);
}

static void on_sync_progress(const SyncProgress& pkt) {
    g_peer_pos = pkt.pos;
}

static void on_sync_arrived(const SyncArrived& pkt) {
    g_peer_pos = pkt.pos;
    if (pkt.txid != g_txid) {
        net_log("[sync] ARRIVED txid=%u (stale, ours=%u)\n", pkt.txid, g_txid);
        return;
    }
    net_log("[sync] peer ARRIVED txid=%u pos=%ld\n", pkt.txid, (long)pkt.pos);
    if (g_sync_state == SyncState::WAITING_PEER || g_sync_state == SyncState::MOVING) {
        g_sync_state      = SyncState::IDLE;
        g_sync_timeout_ms = 0;
        net_log("[sync] txid=%u complete\n", g_txid);
    }
}

static void on_sync_fail(const SyncFail& pkt) {
    net_log("[sync] FAIL txid=%u reason=%u\n", pkt.txid, pkt.reason);
    if (stepper && stepper->isRunning()) stepper->stopMove();
    const int32_t pos = stepper ? stepper->getCurrentPosition() : g_commanded_pos;
    g_target_pos      = pos;
    g_commanded_pos   = pos;
    g_sync_state      = SyncState::IDLE;
    g_sync_timeout_ms = 0;
}

// ── Common packet dispatch (shared by udp_poll and espnow_poll) ───────────────
static void dispatch_packet(const uint8_t* buf, int len, const IPAddress& src) {
    if (len < 1) return;
    const PktType type = static_cast<PktType>(buf[0]);

    if (type == PktType::HELLO && len == (int)sizeof(HelloPkt)) {
        HelloPkt h; memcpy(&h, buf, sizeof(h)); udp_on_hello(h, src);
    } else if (type == PktType::SYNC_MOVE && len == (int)sizeof(SyncMove)) {
        SyncMove m; memcpy(&m, buf, sizeof(m)); on_sync_move(m);
    } else if (type == PktType::SYNC_PROGRESS && len == (int)sizeof(SyncProgress)) {
        SyncProgress p; memcpy(&p, buf, sizeof(p)); on_sync_progress(p);
    } else if (type == PktType::SYNC_ARRIVED && len == (int)sizeof(SyncArrived)) {
        SyncArrived a; memcpy(&a, buf, sizeof(a)); on_sync_arrived(a);
    } else if (type == PktType::SYNC_FAIL && len == (int)sizeof(SyncFail)) {
        SyncFail f; memcpy(&f, buf, sizeof(f)); on_sync_fail(f);
    }
}

// ── Poll functions (call every loop iteration) ────────────────────────────────

void udp_poll() {
    int pkt = g_udp_rx.parsePacket();
    if (pkt < 1) return;
    uint8_t buf[64];
    int len = g_udp_rx.read(buf, sizeof(buf));
    if (len < 1) return;
    const IPAddress src = g_udp_rx.remoteIP();
    net_log("[udp] rx %d bytes from %s type=0x%02x\n",
        len, src.toString().c_str(), (unsigned)buf[0]);
    if (src == WiFi.localIP()) return;   // skip own broadcasts
    dispatch_packet(buf, len, src);
}

void espnow_poll() {
    if (!g_espnow_rx.ready) return;
    uint8_t buf[32];
    const uint8_t len = g_espnow_rx.len;
    memcpy(buf, g_espnow_rx.data, len);
    g_espnow_rx.ready = false;   // consume before dispatch
    net_log("[enow] rx %u bytes type=0x%02x\n", len, (unsigned)buf[0]);
    dispatch_packet(buf, (int)len, IPAddress(0,0,0,0));
}

// ── Tick (call every loop iteration) ─────────────────────────────────────────

void udp_sync_tick() {
    const uint32_t now = millis();

    // Hello window — broadcast position on boot for startup sync
    if (now < g_hello_until_ms && now - g_hello_last_ms >= HELLO_INTERVAL_MS) {
        g_hello_last_ms = now;
        udp_send_hello();
    }

    // MOVING: broadcast progress periodically
    if (g_sync_state == SyncState::MOVING) {
        if (now - g_progress_last_ms >= PROGRESS_INTERVAL_MS) {
            g_progress_last_ms = now;
            SyncProgress pkt{};
            pkt.txid = g_txid;
            pkt.pos  = stepper ? stepper->getCurrentPosition() : g_commanded_pos;
            sync_broadcast(&pkt, sizeof(pkt));
        }
    }

    // WAITING_PEER: check timeout
    if (g_sync_state == SyncState::WAITING_PEER &&
        g_sync_timeout_ms && now >= g_sync_timeout_ms) {
        net_log("[sync] TIMEOUT txid=%u\n", g_txid);
        SyncFail fail{};
        fail.txid   = g_txid;
        fail.reason = 2;
        sync_broadcast(&fail, sizeof(fail));
        g_sync_state      = SyncState::IDLE;
        g_sync_timeout_ms = 0;
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────

void udp_init() {
    const bool rx_ok = g_udp_rx.begin(UDP_PORT);
    g_udp_tx.begin(0);
    net_log("[udp] rx bind port %u: %s   tx ephemeral: ok\n",
        UDP_PORT, rx_ok ? "ok" : "FAILED");
    g_hello_until_ms = millis() + HELLO_WINDOW_MS;
    g_hello_last_ms  = 0;
    net_log("[udp] hello window %lus\n", (unsigned long)(HELLO_WINDOW_MS / 1000));
}

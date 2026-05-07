#pragma once
#include <stdint.h>

// UDP sync wire protocol — all packets share a 1-byte PktType prefix.
// All packets are broadcast; every device ignores its own source IP.

enum class PktType : uint8_t {
    HELLO         = 0x01,   // startup: position + uptime for election
    SYNC_MOVE     = 0x10,   // initiator → peer: start moving to target
    SYNC_PROGRESS = 0x11,   // both → broadcast: position update during move
    SYNC_ARRIVED  = 0x12,   // both → broadcast: motor stopped at target
    SYNC_FAIL     = 0x13,   // either → broadcast: conflict / abort
};

// Broadcast on boot — device with higher uptime wins position authority
struct HelloPkt {
    PktType  type      = PktType::HELLO;
    uint32_t uptime_ms = 0;
    int32_t  position  = 0;
};

// Initiator → broadcast: "move to target, txid identifies this transaction"
struct SyncMove {
    PktType  type     = PktType::SYNC_MOVE;
    uint16_t txid     = 0;    // random transaction id
    int32_t  target   = 0;    // absolute steps
    uint32_t speed_hz = 0;
};

// Both → broadcast: periodic current position while moving
struct SyncProgress {
    PktType  type = PktType::SYNC_PROGRESS;
    uint16_t txid = 0;
    int32_t  pos  = 0;
};

// Both → broadcast: motor reached final position
struct SyncArrived {
    PktType  type = PktType::SYNC_ARRIVED;
    uint16_t txid = 0;
    int32_t  pos  = 0;
};

// Either → broadcast: conflict (two txids seen) or other abort
struct SyncFail {
    PktType  type   = PktType::SYNC_FAIL;
    uint16_t txid   = 0;
    uint8_t  reason = 0;   // 1 = conflict, 2 = timeout
};

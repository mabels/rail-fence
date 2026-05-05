#pragma once
#include <stdint.h>

// UDP sync wire protocol — all packets share a 1-byte type prefix.

enum class PktType : uint8_t {
    MOVE_CMD = 0x01,   // leader → follower: move to absolute position
    MOVE_ACK = 0x02,   // follower → leader: acknowledge
    HELLO    = 0x03,   // broadcast on startup: position + uptime for sync election
};

// Leader → follower: move motor to absolute step position
struct MoveCmd {
    PktType  type     = PktType::MOVE_CMD;
    int32_t  steps;      // absolute target in steps
    uint32_t speed_hz;   // steps/s
    uint8_t  seq;        // rolling sequence (dedup)
};

// Follower → leader: acknowledgement
struct MoveAck {
    PktType  type     = PktType::MOVE_ACK;
    uint8_t  seq;        // echoes MoveCmd.seq
    int32_t  position;   // follower current position in steps
    bool     success;
};

// Broadcast on boot and periodically: for startup position election.
// The device with the higher uptime_ms is the authoritative position source.
struct HelloPkt {
    PktType  type     = PktType::HELLO;
    uint32_t uptime_ms;
    int32_t  position;   // current position in steps
    uint8_t  role;       // 0 = leader, 1 = follower
};

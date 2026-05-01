#pragma once
#include <stdint.h>

// ESP-NOW wire protocol — leader → follower (MoveCmd), follower → leader (MoveAck)

struct MoveCmd {
    int32_t  steps;     // signed: + forward, − backward
    uint32_t speed_hz;  // steps per second
    uint8_t  seq;       // rolling sequence number (dedup on receiver)
};

struct MoveAck {
    uint8_t  seq;       // echoes MoveCmd.seq
    int32_t  position;  // follower current position in steps from its stored start
    bool     success;   // false = was busy or encountered an error
};

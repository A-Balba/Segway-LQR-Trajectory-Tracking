
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>   // for uint8_t, uint32_t, uint32_t

enum {
    PKT_TELEM       = 1,
    PKT_EVENTS      = 2,
    PKT_OBSERVED_DT = 3,
    PKT_CMD         = 4,
    PKT_DEBUG       = 5
};

#define PACKED __attribute__((packed))

typedef struct PACKED {
    uint32_t tick_count;
    uint32_t ts;   // in ms
    float    x1, x2, x3, x4;
    float    u;
} Telemetry;

typedef struct PACKED {
    uint32_t tick_count;
    uint32_t event_sensing;
    uint32_t event_control;
    uint32_t event_actuation;
    uint32_t event_telemetry;
} Event_Telemetry;

typedef struct PACKED {
    uint32_t observed_dt; // in us
} Observed_dt_telemetry;

typedef struct PACKED {
    uint32_t tick_count;
    float    u;
} Command;

typedef struct PACKED {
    uint32_t debug_code;
} Debug;

void init_binary_communication();
bool receive_command_for_tick(uint32_t expected_tick, float& out_u, uint32_t timeout_us);
void send_telem_packet(const Telemetry& t);
void send_events_packet(const Event_Telemetry& et);
void send_odt_packet(const Observed_dt_telemetry& odt);
void send_debug_packet(const Debug& debug_message);

#endif
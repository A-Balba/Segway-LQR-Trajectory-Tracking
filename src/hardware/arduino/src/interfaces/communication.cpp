
#include "Arduino.h"
#include "communication.h"
#include "SerialTransfer.h"


SerialTransfer link;

void init_binary_communication() {
    link.begin(Serial);
}

bool receive_command_for_tick(uint32_t expected_tick, float& out_u, uint32_t timeout_us) {
    uint32_t start = micros();

    while (micros() - start < timeout_us) {
        if (!link.available()) continue;

        uint16_t pos = 0;
        uint32_t tag = 0;

        link.rxObj(tag, pos); pos += sizeof(uint32_t);  // Read from pos 0, advance by 4
        
        if (tag == PKT_CMD) {
            uint32_t tick_count = 0;
            double u;
            link.rxObj(tick_count, pos); pos += sizeof(uint32_t);  // Read from pos 4, advance by 4
            link.rxObj(u, pos);                                     // Read from pos 8

            if (tick_count == expected_tick) { 
                out_u = u;
                return true;
            }
        } 
    }
    return false; // timeout
}

void send_telem_packet(const Telemetry& t) {
    uint16_t n = 0;
    n = link.txObj((uint8_t)PKT_TELEM, n);  // tag
    n = link.txObj(t, n);                   // payload
    link.sendData(n);
}

void send_events_packet(const Event_Telemetry& et) {
    uint16_t n = 0;
    n = link.txObj((uint8_t)PKT_EVENTS, n); // tag
    n = link.txObj(et, n);                  // payload
    link.sendData(n);
}

void send_odt_packet(const Observed_dt_telemetry& odt) {
    uint16_t n = 0;
    n = link.txObj((uint8_t)PKT_OBSERVED_DT, n); // tag
    n = link.txObj(odt, n);                  // payload
    link.sendData(n);
}

void send_debug_packet(const Debug& debug_message) {
    uint16_t n = 0;
    n = link.txObj((uint8_t)PKT_DEBUG, n); // tag
    n = link.txObj(debug_message, n);        // payload
    link.sendData(n);
}

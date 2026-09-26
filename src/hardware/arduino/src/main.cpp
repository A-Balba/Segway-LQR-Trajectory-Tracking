
/******************************************************************************
* Includes
*******************************************************************************/

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Wire.h>
#include <TimerOne.h>

// Modules
#include <globals.h>
#include <miniseg_pins.h>
#include <interfaces/mpu.h>
#include <interfaces/encoder.h>
#include <interfaces/motor.h>
#include <interfaces/communication.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

#define SERIAL_BAUD_RATE 2000000 
#define SOLVER_TIME_STEP_MS 10
#define COMMUNICATION_TIMEOUT_US 7000 
#define MAX_ANGLE 30 * DEG_TO_RAD

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

const double Ts = ((double) SOLVER_TIME_STEP_MS / (double) 1000);
double current_alpha = 0;

uint32_t starting_time = micros();
uint32_t starting_time_ms = millis();
uint32_t prev_time = micros();

volatile bool control_flag = false;
volatile uint32_t tick_count = 0;

double u = 0;
uint32_t event_telemetry = 0;

double prev_u_command = 0.0f;

static uint32_t prev_tick_us = 0;

/******************************************************************************
* Function Definitions
*******************************************************************************/

void control_timer_ISR() {
    control_flag = true;
    tick_count++; 
}

void setup() {
    // Set PB7 (Digital Pin 13) as output -> LED
    DDRB |= (1 << PB7);

    // Set Up Timer
    Timer1.initialize(SOLVER_TIME_STEP_MS * 1000); 
    Timer1.attachInterrupt(control_timer_ISR); 

    // Init UART for communication with laptop
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.write("\nHey! The Segway is initializing ...\n");

    // Initialize Interfaces
    mpu_init();
        // MPU Communicates via I2C, for which the default pin config is used

    Serial.write("Starting calibration ...\n");
    Serial.print("Calibration: Hold segway vertical and steady.");
    mpu_calibrate();

    encoder_init();

    // Configure motor pins
    pinMode(M2A_PIN, OUTPUT);
    pinMode(M2B_PIN, OUTPUT);

    Serial.write("Entering the control loop ...\n");

    init_binary_communication();

    starting_time_ms = millis();
}


void loop() {
    // Toggle PB7 (LED)
    PORTB ^= (1 << PB7);

    if (control_flag) {
        noInterrupts();
        control_flag = false;
        interrupts();

        uint32_t event_loopStart = micros();
        uint32_t tick_us = micros();

        // Measure states
        mpu_get_raw_measurements();
            // encoder measurements are taken via ISR
        double x_4 = mpu_calculate_angular_velocity();
        double x_2 = mpu_calculate_angle(x_4, Ts);
        double x_1 = encoder_calculate_position();
        double x_3 = encoder_calculate_linear_velocity(Ts);
        uint32_t event_sensing = micros() - event_loopStart;

        // Send state information
        uint32_t event_now = millis() - starting_time_ms;
        Telemetry t{tick_count, event_now, x_1,x_2,x_3,x_4,u };
        send_telem_packet(t);

        if (x_2 > MAX_ANGLE || x_2 < -MAX_ANGLE) {
            motor_apply_input(0);
            send_debug_packet({777});
            while (1) {
                // do nothing
            }
        }

        // Receive control action
        uint32_t time_left = (SOLVER_TIME_STEP_MS * 1000) - (micros() - event_loopStart);
        uint32_t time_safety_margin_us = 500;
        uint32_t dynamic_timeout = time_left - time_safety_margin_us;
        float new_u = prev_u_command;  // default to hold last value
        if (receive_command_for_tick(tick_count, new_u, dynamic_timeout)) {
            prev_u_command = new_u;
            send_debug_packet({888});
        } else {
            // timeout
            send_debug_packet({999});
        }
        u = prev_u_command;
        uint32_t event_control = micros() - event_loopStart;

        // Apply control action
        motor_apply_input(u);
        uint32_t event_actuation = micros() - event_loopStart;

        // Send further telemetry, not relevant for control
        uint32_t observed_dt = (prev_tick_us == 0) ? 0 : (tick_us - prev_tick_us);
        Observed_dt_telemetry odt{observed_dt};
        send_odt_packet(odt);
        prev_tick_us = tick_us;

        Event_Telemetry et{tick_count,event_sensing,event_control,event_actuation,event_telemetry};
        send_events_packet(et);

        event_telemetry = micros() - event_loopStart;
    }    
}

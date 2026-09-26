
#include <interfaces/motor.h>
#include <miniseg_pins.h>

#include <Arduino.h>

void motor_apply_input(float input_voltage) {

    // Clamp to ±Vs
    float Vs = 7.2; 
    if (input_voltage > Vs) input_voltage = Vs;
    if (input_voltage < -Vs) input_voltage = -Vs;

    // Compute duty cycle
    float duty_cycle = input_voltage * (255 / Vs);
    uint8_t pwm = (uint8_t)duty_cycle;

    // Set direction pin
    if (input_voltage < 0) {
        digitalWrite(M2A_PIN, HIGH); 
    } else {
        digitalWrite(M2A_PIN, LOW);
    }

    // Set PWM on the speed pin
    analogWrite(M2B_PIN, pwm);
}
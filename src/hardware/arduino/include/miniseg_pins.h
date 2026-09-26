#ifndef MINISEG_PINS_H
#define MINISEG_PINS_H

// Motor 1 connections
#define M1A_PIN     2    // Amplified signal from D2
#define M1B_PIN     5    // Amplified signal from D5 through J8
#define M1E1_PIN    15   // D15 - Encoder interrupt
#define M1E2_PIN    A8   // Analog 8 - Encoder PCINT

// Motor 2 connections
#define M2A_PIN     6    // Amplified signal from D6
#define M2B_PIN     8    // Amplified signal from D8 through J7
#define M2E1_PIN    19   // D19 - Encoder interrupt
#define M2E2_PIN    18   // D18 - Encoder interrupt

// Potentiometer and Button
#define POT_PIN     A0   // Potentiometer
#define BUTTON_PIN  A3   // User input button

// Sensor connections (S1, S2, S3)
#define S1_READ_PIN     A0
#define S1_LED_PIN      A11
#define S1_PULLUP_J     "J1"

#define S2_READ_PIN     A1
#define S2_LED_PIN      A2
#define S2_PULLUP_J     "J2"

#define S3_READ_PIN     A9
#define S3_PULLUP_J     "J3"
#define S3_LED_J        "J4"
#define S3_I2C_SCL_J    "J5"
#define S3_I2C_SDA_J    "J6"

// Optional headers
#define BLUETOOTH_COMM_PORT  2
#define BLUETOOTH_AT_PORT    0
#define ULTRASONIC_HEADER    "H3"
#define I2C_HEADER_SCL_SDA   "H4"

#endif // MINISEG_PINS_H
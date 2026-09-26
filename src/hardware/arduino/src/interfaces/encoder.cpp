

/******************************************************************************
* Includes
*******************************************************************************/

#include <interfaces/encoder.h>

#include <Arduino.h>
#include <miniseg_pins.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

#define WHEEL_RADIUS 0.02 // in m
#define ENCODER_CPR 720   // Counts per revolution (NXT motors)

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

const double C2DEG = ((double) 360 / (double) ENCODER_CPR);
const double ES = (-C2DEG * DEG_TO_RAD);

volatile long encoder_A_count = 0;

double current_theta = 0;
double prev_theta = 0;

const uint8_t pinA = M2E2_PIN /* your A pin */;
const uint8_t pinB = M2E1_PIN /* your B pin */;

volatile int32_t encCount = 0;
volatile uint8_t prevAB = 0;

/******************************************************************************
* Function Definitions
*******************************************************************************/

inline void handleEncoder() {
  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  uint8_t ab = (a << 1) | b;

  static const int8_t LUT[4][4] = {
    /*prev 00*/ {  0, +1, -1,  0 },
    /*prev 01*/ { -1,  0,  0, +1 },
    /*prev 10*/ { +1,  0,  0, -1 },
    /*prev 11*/ {  0, -1, +1,  0 }
  };
  encoder_A_count += LUT[prevAB][ab];
  prevAB = ab;
}

void ISR_A() { handleEncoder(); }
void ISR_B() { handleEncoder(); }

void encoder_init() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  prevAB = (digitalRead(pinA) << 1) | digitalRead(pinB);
  attachInterrupt(digitalPinToInterrupt(pinA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), ISR_B, CHANGE);
}

double encoder_calculate_position() {
  double encoder_raw = (double) encoder_A_count;

  double theta = encoder_raw * (-ES);
  current_theta = theta;

  double s = - theta * WHEEL_RADIUS;

  return s;
}

double encoder_calculate_linear_velocity(double Ts) {

  if (Ts <= 0) return 0;

  double delta_theta = current_theta - prev_theta;
  double theta_dot = delta_theta / (Ts);
  prev_theta = current_theta;

  double s_dot = - theta_dot * WHEEL_RADIUS;

  return s_dot;
}
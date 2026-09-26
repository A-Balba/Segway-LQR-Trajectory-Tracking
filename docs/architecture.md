# Segway Control Architecture

## 1. Plant model

The Segway is represented by the four-state vector

```text
x = [s, α, ṡ, α̇]ᵀ
```

where `s` is longitudinal position and `α` is the pendulum pitch angle.

The nonlinear model is defined in `src/model/segway_model.py` using the physical parameters in `params.json`.

## 2. Linear control model

The nonlinear dynamics are differentiated around the upright equilibrium to obtain continuous-time matrices `Ac` and `Bc`. The model is discretized with zero-order hold to obtain `Ad` and `Bd`.

## 3. CSTD II

```text
Nonlinear model
      ↓
Linearization
      ↓
Discretization
      ↓
Controllability
      ↓
LQR design
      ↓
State feedback
      ↓
Segway hardware
```

Infinite-horizon LQR uses the discrete algebraic Riccati equation. Finite-horizon LQR uses a backward Riccati recursion to obtain a time-varying gain sequence.

## 4. CSTD III

```text
Position reference s_ref(t)
          ↓
Flatness-based state reference
          ↓
Model-inversion feedforward u_ff
          +
LQR feedback u_fb
          ↓
u = u_ff + u_fb
          ↓
Segway hardware
```

## 5. Hardware state estimation

```text
MPU6050 ──→ gyro + accelerometer ──→ complementary filter ──→ α, α̇
Encoder ──→ wheel position/velocity ───────────────────────→ s, ṡ
                                                               │
                                                               ▼
                                                        state vector x
                                                               │
                                                               ▼
                                                        Python controller
                                                               │
                                                               ▼
                                                         control input u
                                                               │
                                                               ▼
                                                          Arduino
                                                               │
                                                               ▼
                                                            motors
```

The Arduino firmware also records timing information and transmits telemetry and status packets to the Python side.

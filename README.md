# Dual‑Encoder Pendulum + Motor Control (Arduino Mega + BasicEncoder)

An Arduino **Mega 2560** sketch that reads two quadrature encoders using **pin‑change interrupts** (via the excellent [BasicEncoder](https://github.com/PaulStoffregen/Encoder)‑style API variant) and drives a DC motor through an H‑bridge to stabilize a pendulum angle. It logs angles and filtered velocities over Serial and implements a simple PI controller for both the **pendulum** and the **motor**, combined into a single command.

> Includes interrupt‑safe encoder servicing, exponential velocity filtering, soft output limiting, and a safety stop outside ±60°.

---

## Table of Contents

* [Hardware](#hardware)
* [Pin Map](#pin-map)
* [Libraries](#libraries)
* [Build & Upload](#build--upload)
* [How It Works](#how-it-works)
* [Parameters to Tune](#parameters-to-tune)
* [Serial Output](#serial-output)
* [Safety & Limits](#safety--limits)
* [License & Attribution](#license--attribution)

---

## Hardware

* **MCU:** Arduino **Mega 2560** (uses PCINT2\_vect)
* **Encoders:**

  * `encoder1` — pendulum (counts → angle)
  * `encoder2` — motor shaft (counts → angle)
* **Motor driver:** H‑bridge (e.g., L298N, BTN7971, or similar)
* **Motor:** Brushed DC motor
* **Power:** Separate motor supply recommended; share **GND** with the Mega

---

## Pin Map

```
Encoders (Mega):
  encoder1 A/B  -> 69 / 68   (pendulum)
  encoder2 A/B  -> 67 / 66   (motor)

Motor driver (H‑bridge control):
  pinM1 -> 52 (DIR1)
  pinM2 -> 53 (DIR2)
  pinMS ->  2 (PWM / speed)

---

## Libraries

* **BasicEncoder** (Peter Harrison / Helicron). Install via Library Manager or clone the repo that provides `BasicEncoder.h`.
* **Arduino core** for Mega 2560.

The sketch uses direct **pin‑change interrupts** (PCINT) and calls `encoder.service()` from the ISR.

---

## Build & Upload

1. Open the sketch (e.g., `dual_encoder_control.ino`) in **Arduino IDE**.
2. Select **Board: Arduino Mega 2560** and correct **Port**.
3. Install the **BasicEncoder** library.
4. Upload and open **Serial Monitor @ 115200 baud**.

---

## How It Works

### Encoder servicing

* `pciSetup(pin)` enables PCINT on each encoder pin.
* The single ISR `ISR(PCINT2_vect)` calls `encoder1.service()` and `encoder2.service()`.
* Use of a shared vector keeps both channels responsive without polling.

### Angle and velocity

* Angles are derived via fixed counts‑per‑rev scalings:

  * Pendulum: `thetap = encoder1.count * (360 / 2000)` (2000 CPR example)
  * Motor:    `thetam = encoder2.count * (360 / 220)`   (220 CPR example)
* Velocities are estimated from finite differences every microsecond tick and **exponentially smoothed**:

  ```c++
  velocity1 = (Δpos1 / Δt) * 180000.0;          // pendulum
  velocity2 = (Δpos2 / Δt) * (18000000.0/11.0); // motor
  filteredV = alpha * v + (1 - alpha) * filteredV;
  ```

  `alpha1=0.08` (faster) and `alpha2=0.02` (slower) balance noise vs. responsiveness.

### Control (dual PI combined)

* Pendulum PI: `motorsp = -(kpp*thetap + kip*∫thetap dt)`
* Motor PI:    `motorsm = -(kpm*thetam + kim*∫thetam dt)`
* Combined command: `motors = motorsp + motorsm`, then saturated to `[-255, 255]` and sent to the H‑bridge via:

  ```c++
  motor_clockwise(mspeed) / motor_cclockwise(mspeed) / motor_stop();
  ```

### Safety window

* If `thetam` leaves **±60°**, the loop **stops** and the motor is commanded to **brake** (both DIR high, PWM 0).

---

## Parameters to Tune

```c++
// Velocity filters
const float alpha1 = 0.08; // pendulum velocity smoothing
const float alpha2 = 0.02; // motor velocity smoothing

// PI gains (pendulum)
float kpp = 200.0;  // proportional
float kip = 0.0;    // integral

// PI gains (motor)
float kpm = 40.0;   // proportional
float kim = 0.0;    // integral

---

## Serial Output

Each time either encoder changes, the sketch prints **tab‑separated** values:

```
<thetap>	<filteredVelocity1>	<thetam>	<filteredVelocity2>
```

Example:

```
-2.15	-15.7	3.64	92.1
```

You can log this stream to a CSV for offline analysis (e.g., Python/Matlab).

---

## Safety & Limits

* **±60° limit:** outside this window, the loop stops and the motor brakes.
* Ensure your driver supports **brake** behavior (both highs) or adapt the code.
* Always test on **low voltage** and with a physical **catch** or tether.

---

## License & Attribution

This project includes code derived from/compatible with **Peter Harrison – Helicron** examples and is licensed under the **Apache License 2.0**:

```
Copyright 2021 Peter Harrison - Helicron
Licensed under the Apache License, Version 2.0
http://www.apache.org/licenses/LICENSE-2.0
```

# nRF52840 Motion-to-Haptic Feedback System

A real-time system that captures hand movement and turns it into physical haptic feedback. Two independent nRF52840 boards talk over BLE: a **motion-sensing unit** reads an IMU, and a **haptic feedback unit** drives a vibration motor from the incoming data.

| Motion-sensing unit | Haptic feedback unit |
|---|---|
| ![Motion-sensing unit PCB](docs/images/motion-unit-pcb.jpg) | ![Haptic feedback unit PCB](docs/images/haptic-unit-pcb.jpg) |

> Built by [DigitalMonk](https://digitalmonk.biz/) as part of our [embedded development services](https://digitalmonk.biz/electronics-embedded-software-development-services/).

---

## Table of contents

- [The challenge](#the-challenge)
- [How it works](#how-it-works)
- [Components used](#components-used)
- [Why the nRF52840](#why-the-nrf52840)
- [Motion unit firmware](#motion-unit-firmware)
- [BLE link](#ble-link)
- [Haptic unit firmware](#haptic-unit-firmware)
- [PCB design](#pcb-design)
- [Repository structure](#repository-structure)
- [Getting started](#getting-started)
- [Results](#results)
- [Limitations and notes](#limitations-and-notes)
- [Work with us](#work-with-us)

---

## The challenge

A haptic response only feels right if it is fast, stable and repeatable. That put four engineering problems at the center of this project:

1. **Low-latency BLE communication** between the two devices
2. **Accurate, noise-filtered IMU processing** without adding delay
3. **Drift-free mapping** of orientation and movement to haptic output
4. **Stable, repeatable motor control** under varying load and power states

Most of the design decisions below come from those four points.

## How it works

```
 ┌──────────────────────┐        BLE notify        ┌──────────────────────┐
 │  Motion unit         │  ───────────────────▶   │  Haptic unit         │
 │  nRF52840 + IMU      │   8-byte packet @100 Hz  │  nRF52840 + driver   │
 │  (BLE peripheral)    │                          │  (BLE central)       │
 └──────────────────────┘                          └──────────┬───────────┘
        IMU → filter → packet                                  │ PWM
                                                         vibration motor
```

1. The motion unit reads the IMU at a fixed rate and filters the noise.
2. Roll and pitch are sent to the haptic unit as a compact BLE notification.
3. The haptic unit maps the tilt to a motor intensity and drives the motor.
4. If data stops or the link drops, the motor switches off.

## Components used

| Component | Qty | Notes |
|---|---|---|
| nRF52840 SoC (AQFN-73) | 2 | One per unit |
| IMU (MPU6050 in the reference firmware) | 1 | Motion unit, I2C |
| Vibration (haptic) motor | 1 | Haptic unit |
| Motor driver circuitry (MOSFET / driver IC) | 1 | Haptic unit, custom design |
| Custom PCBs | 2 | Motion unit and haptic unit |
| USB-C connector | 2 | Power and programming |
| SWD debug header | 2 | Flashing and debugging |
| Crystals, passives, push buttons | – | See the schematic / BOM |

## Why the nRF52840

- Built-in Bluetooth 5 radio, so no external BLE module is needed
- Cortex-M4F with a hardware FPU, which suits the floating-point sensor filtering
- Hardware PWM that runs without CPU load
- Low-power design that fits a battery-powered motion unit
- Supported by the nRF Connect SDK (Zephyr), with a full debug toolchain over SWD

## Motion unit firmware

The motion unit is a BLE peripheral. Its main loop runs at a fixed 100 Hz.

- **Gyro calibration at boot:** about 200 samples are averaged while the unit is still, and the result is subtracted as a bias.
- **Complementary filter:** the gyro gives fast response and the accelerometer pulls the angle back, so roll and pitch do not drift.
- **Fixed-rate loop:** the loop uses absolute-time sleeps, so timing jitter does not build up.
- **No blocking:** if the BLE stack is out of buffers, the sample is dropped instead of stalling the loop.

Yaw is only sent as a rate. A true yaw angle needs a magnetometer.

## BLE link

The link uses one custom GATT service with a single notify characteristic.

| Field | Type | Meaning |
|---|---|---|
| `seq` | `uint16` | Packet counter, used to count lost packets |
| `roll_cdeg` | `int16` | Roll in centi-degrees |
| `pitch_cdeg` | `int16` | Pitch in centi-degrees |
| `gyro_z_cdps` | `int16` | Yaw rate in centi-degrees per second |

- 8 bytes per packet, little-endian
- Connection interval requested: 7.5 – 15 ms, no slave latency, 4 s supervision timeout
- The haptic unit finds the motion unit by its service UUID, so no addresses are hard-coded
- The packet definition lives in one shared header, [`common/motion_protocol.h`](common/motion_protocol.h)

## Haptic unit firmware

The haptic unit is a BLE central. It scans, connects, discovers the service and subscribes to notifications.

| Setting | Value |
|---|---|
| Deadband | 3° (smaller tilts are ignored) |
| Full scale | 45° tilt = 100 % duty |
| Minimum start duty | 25 % (many ERM motors will not spin below this) |
| Smoothing | One-pole low-pass on the motor duty |
| PWM frequency | 20 kHz, above the audible range |
| Failsafe | Motor off after 200 ms without data, and on disconnect |

Lost packets are counted from the sequence numbers and logged.

## PCB design

Two custom boards were designed, one for each unit. Both use an nRF52840 in the AQFN-73 package, USB-C for power and programming, and an SWD header for flashing and debugging.

The motor driver stage supplies the motor current. The nRF52840 GPIO only provides the control signal.

## Repository structure

```
.
├── common/
│   └── motion_protocol.h      # shared UUIDs and packet format
├── motion_unit/               # BLE peripheral: IMU -> filter -> notify
│   ├── boards/
│   │   └── nrf52840dk_nrf52840.overlay
│   ├── src/main.c
│   ├── prj.conf
│   └── CMakeLists.txt
├── haptic_unit/               # BLE central: subscribe -> PWM motor
│   ├── src/main.c
│   ├── prj.conf
│   └── CMakeLists.txt
└── docs/images/               # PCB renders
```

## Getting started

### Requirements

- nRF Connect SDK v2.7+ (Zephyr 3.7+). For older SDKs, the board name is `nrf52840dk_nrf52840`.
- Two nRF52840 boards (nRF52840 DK or the custom PCBs)
- An MPU6050 breakout for the motion unit

### Build and flash

```bash
# Motion unit
west build -b nrf52840dk/nrf52840 motion_unit -d build_motion
west flash -d build_motion

# Haptic unit (flash the second board)
west build -b nrf52840dk/nrf52840 haptic_unit -d build_haptic
west flash -d build_haptic
```

### Wiring (nRF52840 DK)

| Unit | Signal | Pin |
|---|---|---|
| Motion | MPU6050 SDA | P0.26 |
| Motion | MPU6050 SCL | P0.27 |
| Motion | MPU6050 VCC / GND | 3.3 V / GND |
| Haptic | Motor PWM | P0.13 (LED1, `pwm_led0` alias) |

For a real motor, drive a MOSFET or driver IC with a flyback diode from the PWM pin. Never connect a motor directly to the GPIO. On a custom board, remap the alias in a board overlay and use normal (non-inverted) PWM polarity.

### Using it

1. Power the haptic unit, then the motion unit.
2. Keep the motion unit still for about 2 seconds while the gyro calibrates.
3. The haptic unit connects automatically and subscribes.
4. Tilt the motion unit. The motor gets stronger as the tilt increases.

## Results

The system gives a low-latency response with no perceptible lag and synchronized feedback, and it suits applications that need precise interaction. No measured latency, sample-rate or battery figures are published yet.

## Limitations and notes

- This is reference firmware. It has not yet been compiled or tested on hardware in this repository, so check the API names against your SDK version.
- Yaw is a rate only (no magnetometer).
- Range is limited by BLE.
- The motion-to-haptic mapping is fixed and needs tuning for each use case.

## Work with us

If you need firmware, PCB design and wireless connectivity that work together, this is the kind of work we do. Learn more about our [embedded development services](https://digitalmonk.biz/electronics-embedded-software-development-services/) or get in touch at [digitalmonk.biz](https://digitalmonk.biz/).

# nRF52840 Motion-to-Haptic (reference firmware)

Two Zephyr / nRF Connect SDK apps:

| App | Role | Does |
|---|---|---|
| `motion_unit/` | BLE peripheral | MPU6050 @100 Hz -> gyro-bias cal -> complementary filter -> 8-byte BLE notify |
| `haptic_unit/` | BLE central | Scan -> connect -> subscribe -> tilt-to-PWM motor drive, failsafe |

Shared packet/UUID definition: `common/motion_protocol.h`.

## Build

nRF Connect SDK v2.7+ / Zephyr 3.7+ (older SDKs: board name `nrf52840dk_nrf52840`):

    west build -b nrf52840dk/nrf52840 motion_unit -d build_motion
    west build -b nrf52840dk/nrf52840 haptic_unit -d build_haptic
    west flash -d build_motion     # then the other board with build_haptic

## Wiring

Motion unit: MPU6050 SDA -> P0.26, SCL -> P0.27, VCC 3.3 V, GND, AD0 low (addr 0x68).
Haptic unit: PWM on `pwm_led0` (P0.13 / LED1 on the DK). For a real motor use a MOSFET or
driver IC with a flyback diode; for a custom board remap the alias in a
`boards/<board>.overlay` and use normal (non-inverted) PWM polarity.

## Behaviour

- Keep the motion unit still for ~2 s after power-up (gyro calibration).
- Tilt the motion unit: motor duty rises from 25 % (3 deg deadband) to 100 % (45 deg).
- Packet loss is counted from the sequence number and logged on the haptic unit.
- Motor shuts off after 200 ms without data and on disconnect.

## Notes

- Roll/pitch are drift-free (accelerometer corrects the gyro). Yaw is only sent as a
  rate (`gyro_z_cdps`), because true yaw needs a magnetometer.
- Not compiled or run on hardware yet. Check API names against your SDK version.

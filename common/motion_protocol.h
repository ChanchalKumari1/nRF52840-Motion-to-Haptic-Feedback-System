/*
 * Shared BLE protocol between the Motion Unit (peripheral) and the
 * Haptic Unit (central). Include from both projects.
 */
#ifndef MOTION_PROTOCOL_H_
#define MOTION_PROTOCOL_H_

#include <stdint.h>
#include <zephyr/toolchain.h>
#include <zephyr/bluetooth/uuid.h>

/* Custom 128-bit UUIDs (change if you want your own) */
#define BT_UUID_MOTION_SVC_VAL  \
	BT_UUID_128_ENCODE(0x4d4f5449, 0x4f4e, 0x4841, 0x5054, 0x494300000001)
#define BT_UUID_MOTION_DATA_VAL \
	BT_UUID_128_ENCODE(0x4d4f5449, 0x4f4e, 0x4841, 0x5054, 0x494300000002)

/* IMU sample rate = BLE notification rate */
#define MOTION_SAMPLE_PERIOD_MS 10

/* Notification payload: 8 bytes, little-endian */
struct motion_pkt {
	uint16_t seq;         /* wraps; used by the receiver to count lost packets */
	int16_t  roll_cdeg;   /* roll,  centi-degrees (accel+gyro fused, drift-free) */
	int16_t  pitch_cdeg;  /* pitch, centi-degrees (accel+gyro fused, drift-free) */
	int16_t  gyro_z_cdps; /* yaw RATE, centi-deg/s (bias-corrected, not an angle) */
} __packed;

#endif /* MOTION_PROTOCOL_H_ */

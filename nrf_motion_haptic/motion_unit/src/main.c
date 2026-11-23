/*
 * Motion Unit (BLE peripheral, nRF52840)
 *
 * - Reads MPU6050 at 100 Hz
 * - Gyro bias calibration at boot, complementary filter -> roll / pitch
 * - Streams a compact 8-byte packet over a custom GATT notification
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "motion_protocol.h"

LOG_MODULE_REGISTER(motion_unit, LOG_LEVEL_INF);

#define ALPHA        0.98f      /* complementary filter: gyro weight */
#define RAD2DEG      57.2957795f
#define CAL_SAMPLES  200        /* ~2 s of gyro bias averaging */

static const struct device *const imu = DEVICE_DT_GET_ONE(invensense_mpu6050);

/* ---------------------------------------------------------------- BLE --- */

static struct bt_uuid_128 svc_uuid  = BT_UUID_INIT_128(BT_UUID_MOTION_SVC_VAL);
static struct bt_uuid_128 data_uuid = BT_UUID_INIT_128(BT_UUID_MOTION_DATA_VAL);

static struct bt_conn *active_conn;
static volatile bool notify_enabled;

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notifications %s", notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(motion_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	BT_GATT_CHARACTERISTIC(&data_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MOTION_SVC_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void start_adv(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed (%d)", err);
	} else {
		LOG_INF("Advertising");
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (0x%02x)", err);
		return;
	}
	active_conn = bt_conn_ref(conn);
	LOG_INF("Connected");

	/* 7.5 - 15 ms interval, no latency, 4 s supervision timeout */
	const struct bt_le_conn_param param = BT_LE_CONN_PARAM_INIT(6, 12, 0, 400);

	bt_conn_le_param_update(conn, &param);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (0x%02x)", reason);
	notify_enabled = false;
	if (active_conn) {
		bt_conn_unref(active_conn);
		active_conn = NULL;
	}
}

static void recycled(void)
{
	start_adv();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected    = connected,
	.disconnected = disconnected,
	.recycled     = recycled,
};

/* ---------------------------------------------------------------- IMU --- */

static int read_imu(float a[3], float g[3])
{
	struct sensor_value acc[3], gyr[3];
	int rc = sensor_sample_fetch(imu);

	if (rc) {
		return rc;
	}
	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr);

	for (int i = 0; i < 3; i++) {
		a[i] = (float)sensor_value_to_double(&acc[i]); /* m/s^2 */
		g[i] = (float)sensor_value_to_double(&gyr[i]); /* rad/s */
	}
	return 0;
}

static int16_t to_cdeg_i16(float rad_or_rps)
{
	float v = rad_or_rps * RAD2DEG * 100.0f;

	return (int16_t)CLAMP(v, -32767.0f, 32767.0f);
}

int main(void)
{
	float a[3], g[3], gbias[3] = {0};
	float roll, pitch;
	uint16_t seq = 0;
	uint32_t drops = 0;
	int err;

	if (!device_is_ready(imu)) {
		LOG_ERR("MPU6050 not ready - check wiring / overlay");
		return -ENODEV;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}
	start_adv();

	/* Gyro bias calibration - keep the unit still for ~2 s after boot */
	LOG_INF("Calibrating gyro, keep still...");
	int good = 0;

	for (int i = 0; i < CAL_SAMPLES; i++) {
		if (read_imu(a, g) == 0) {
			for (int k = 0; k < 3; k++) {
				gbias[k] += g[k];
			}
			good++;
		}
		k_msleep(MOTION_SAMPLE_PERIOD_MS);
	}
	if (good == 0) {
		LOG_ERR("No IMU samples during calibration");
		return -EIO;
	}
	for (int k = 0; k < 3; k++) {
		gbias[k] /= good;
	}

	/* Seed the filter from the accelerometer */
	read_imu(a, g);
	roll  = atan2f(a[1], a[2]);
	pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
	LOG_INF("Running at %d Hz", 1000 / MOTION_SAMPLE_PERIOD_MS);

	const float dt = MOTION_SAMPLE_PERIOD_MS / 1000.0f;
	int64_t next = k_uptime_get();

	for (;;) {
		next += MOTION_SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next)); /* fixed-rate, no accumulated jitter */

		if (read_imu(a, g)) {
			continue;
		}

		/* Complementary filter: gyro for fast response, accel to cancel drift */
		float roll_acc  = atan2f(a[1], a[2]);
		float pitch_acc = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));

		roll  = ALPHA * (roll  + (g[0] - gbias[0]) * dt) + (1.0f - ALPHA) * roll_acc;
		pitch = ALPHA * (pitch + (g[1] - gbias[1]) * dt) + (1.0f - ALPHA) * pitch_acc;

		if (!notify_enabled || !active_conn) {
			continue;
		}

		struct motion_pkt pkt = {
			.seq         = seq++,
			.roll_cdeg   = to_cdeg_i16(roll),
			.pitch_cdeg  = to_cdeg_i16(pitch),
			.gyro_z_cdps = to_cdeg_i16(g[2] - gbias[2]),
		};

		err = bt_gatt_notify(active_conn, &motion_svc.attrs[1], &pkt, sizeof(pkt));
		if (err) {
			/* Out of buffers: drop the sample, never block the 100 Hz loop */
			if (++drops % 100 == 1) {
				LOG_WRN("notify failed (%d), drops=%u", err, drops);
			}
		}
	}
	return 0;
}

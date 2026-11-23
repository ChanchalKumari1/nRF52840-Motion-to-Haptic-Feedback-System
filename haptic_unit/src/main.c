/*
 * Haptic Unit (BLE central, nRF52840)
 *
 * - Scans for the Motion Unit service, connects, discovers and subscribes
 * - Maps tilt magnitude -> motor PWM duty (deadband, min-start duty, smoothing)
 * - Failsafe: motor off if no packet arrives for FAILSAFE_MS or on disconnect
 *
 * Motor output uses the board's pwm_led0 alias (LED1 / P0.13 on the nRF52840 DK).
 * For a real motor, drive a MOSFET / driver IC from that pin (or remap the alias
 * in an overlay) - never connect a motor directly to the GPIO.
 */
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "motion_protocol.h"

LOG_MODULE_REGISTER(haptic_unit, LOG_LEVEL_INF);

/* ---- Tuning ------------------------------------------------------------ */
#define PWM_PERIOD_NS   PWM_USEC(50)  /* 20 kHz, above audible range */
#define DEADBAND_DEG    3.0f          /* ignore small tilts */
#define FULL_SCALE_DEG  45.0f         /* tilt at which output reaches 100 % */
#define MIN_DUTY_PCT    25.0f         /* many ERM motors won't spin below this */
#define SMOOTHING       0.30f         /* 0..1, higher = faster response */
#define FAILSAFE_MS     200

static const struct pwm_dt_spec motor = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

static float smoothed_pct;

static void motor_set(float pct)
{
	pct = CLAMP(pct, 0.0f, 100.0f);
	uint32_t pulse = (uint32_t)((float)PWM_PERIOD_NS * pct / 100.0f);

	pwm_set(motor.dev, motor.channel, PWM_PERIOD_NS, pulse, motor.flags);
}

static void failsafe_handler(struct k_work *work)
{
	smoothed_pct = 0.0f;
	motor_set(0.0f);
	LOG_WRN("No data - motor off");
}
static K_WORK_DELAYABLE_DEFINE(failsafe_work, failsafe_handler);

static void update_haptics(const struct motion_pkt *p)
{
	float roll  = p->roll_cdeg / 100.0f;
	float pitch = p->pitch_cdeg / 100.0f;
	float mag   = sqrtf(roll * roll + pitch * pitch);

	float norm = (mag - DEADBAND_DEG) / (FULL_SCALE_DEG - DEADBAND_DEG);
	norm = CLAMP(norm, 0.0f, 1.0f);

	float target = (norm > 0.0f) ? MIN_DUTY_PCT + norm * (100.0f - MIN_DUTY_PCT) : 0.0f;

	/* One-pole low-pass keeps the motor drive stable between packets */
	smoothed_pct += SMOOTHING * (target - smoothed_pct);
	motor_set(smoothed_pct < 1.0f ? 0.0f : smoothed_pct);
}

/* ---------------------------------------------------------------- BLE --- */

static struct bt_uuid_128 svc_uuid  = BT_UUID_INIT_128(BT_UUID_MOTION_SVC_VAL);
static struct bt_uuid_128 data_uuid = BT_UUID_INIT_128(BT_UUID_MOTION_DATA_VAL);
static struct bt_uuid_16  ccc_uuid  = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static const uint8_t svc_bytes[16] = { BT_UUID_MOTION_SVC_VAL };

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static void start_scan(void);

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	static uint16_t last_seq;
	static bool have_seq;
	static uint32_t rx_count, lost;

	if (!data) {
		LOG_INF("Unsubscribed");
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}
	if (length != sizeof(struct motion_pkt)) {
		return BT_GATT_ITER_CONTINUE;
	}

	struct motion_pkt p;

	memcpy(&p, data, sizeof(p));

	if (have_seq) {
		lost += (uint16_t)(p.seq - last_seq - 1);
	}
	last_seq = p.seq;
	have_seq = true;

	if (++rx_count % 500 == 0) {
		LOG_INF("rx=%u lost=%u", rx_count, lost);
	}

	k_work_reschedule(&failsafe_work, K_MSEC(FAILSAFE_MS));
	update_haptics(&p);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		LOG_ERR("Discovery finished without finding the motion service");
		memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(params->uuid, &svc_uuid.uuid)) {
		/* Service found -> look for the data characteristic */
		params->uuid         = &data_uuid.uuid;
		params->start_handle = attr->handle + 1;
		params->type         = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, params);
		if (err) {
			LOG_ERR("Characteristic discovery failed (%d)", err);
		}
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(params->uuid, &data_uuid.uuid)) {
		/* Characteristic found -> look for its CCC descriptor */
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
		params->uuid         = &ccc_uuid.uuid;
		params->start_handle = attr->handle + 2;
		params->type         = BT_GATT_DISCOVER_DESCRIPTOR;
		err = bt_gatt_discover(conn, params);
		if (err) {
			LOG_ERR("Descriptor discovery failed (%d)", err);
		}
		return BT_GATT_ITER_STOP;
	}

	/* CCC found -> subscribe */
	subscribe_params.notify     = notify_func;
	subscribe_params.value      = BT_GATT_CCC_NOTIFY;
	subscribe_params.ccc_handle = attr->handle;

	err = bt_gatt_subscribe(conn, &subscribe_params);
	if (err && err != -EALREADY) {
		LOG_ERR("Subscribe failed (%d)", err);
	} else {
		LOG_INF("Subscribed to motion data");
	}
	return BT_GATT_ITER_STOP;
}

static bool ad_has_service(struct bt_data *data, void *user_data)
{
	bool *found = user_data;

	if ((data->type == BT_DATA_UUID128_ALL || data->type == BT_DATA_UUID128_SOME) &&
	    data->data_len == sizeof(svc_bytes) &&
	    !memcmp(data->data, svc_bytes, sizeof(svc_bytes))) {
		*found = true;
		return false; /* stop parsing */
	}
	return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		    struct net_buf_simple *ad)
{
	bool found = false;

	if (default_conn) {
		return;
	}
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}
	bt_data_parse(ad, ad_has_service, &found);
	if (!found) {
		return;
	}

	LOG_INF("Motion Unit found (RSSI %d), connecting", rssi);
	if (bt_le_scan_stop()) {
		return;
	}

	/* 7.5 - 15 ms interval, no latency, 4 s supervision timeout */
	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				    BT_LE_CONN_PARAM(6, 12, 0, 400), &default_conn);
	if (err) {
		LOG_ERR("Create connection failed (%d)", err);
		start_scan();
	}
}

static void start_scan(void)
{
	/* Passive, continuous: the service UUID is in the primary advertising data */
	struct bt_le_scan_param param = BT_LE_SCAN_PARAM_INIT(
		BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_NONE,
		BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);

	int err = bt_le_scan_start(&param, scan_cb);

	if (err) {
		LOG_ERR("Scan start failed (%d)", err);
	} else {
		LOG_INF("Scanning...");
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (conn != default_conn) {
		return;
	}
	if (err) {
		LOG_ERR("Connection failed (0x%02x)", err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		start_scan();
		return;
	}
	LOG_INF("Connected");

	memset(&discover_params, 0, sizeof(discover_params));
	discover_params.uuid         = &svc_uuid.uuid;
	discover_params.func         = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

	int rc = bt_gatt_discover(conn, &discover_params);

	if (rc) {
		LOG_ERR("Discovery start failed (%d)", rc);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != default_conn) {
		return;
	}
	LOG_INF("Disconnected (0x%02x)", reason);

	k_work_cancel_delayable(&failsafe_work);
	smoothed_pct = 0.0f;
	motor_set(0.0f);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	start_scan();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected    = connected,
	.disconnected = disconnected,
};

int main(void)
{
	if (!pwm_is_ready_dt(&motor)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}
	motor_set(0.0f);

	int err = bt_enable(NULL);

	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}
	start_scan();
	return 0; /* everything else is event driven */
}

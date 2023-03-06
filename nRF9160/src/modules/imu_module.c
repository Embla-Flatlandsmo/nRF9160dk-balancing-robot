/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <assert.h>

#include <zsl/zsl.h>
#include <zsl/orientation/orientation.h>
#include <zsl/instrumentation.h>

#include <stdio.h>

#include "imu_module_event.h"

#include "modules_common.h"

#include "app_mpu.h"


#define FUSION_ALGO_MADGWICK		0
#define FUSION_ALGO_AQUA		1
#define FUSION_ALGO_EKF			0

#define ACCEL_ALPHA			(0.7f)
#define GYRO_ALPHA			(0.7f)
#define MAGN_ALPHA			(0.05f)
#define ANGLE_ALPHA			(0.7f)

#define IMU_THREAD_SLEEP_USEC 		2000
#define CONTROL_LOOP_INTERVAL_MS	10

#define MODULE imu_module

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_module, CONFIG_IMU_MODULE_LOG_LEVEL);

#if FUSION_ALGO_MADGWICK

/* Config settings for the Madgwick filter. */
static struct zsl_fus_madg_cfg madg_cfg = {
	.beta = 0.22,
};

static struct zsl_fus_drv madgwick_drv = {
	.init_handler = zsl_fus_madg_init,
	.feed_handler = zsl_fus_madg_feed,
	.error_handler = zsl_fus_madg_error,
	.config = &madg_cfg,
};

#elif FUSION_ALGO_AQUA

/* Config settings for the AQUA filter. */
static struct zsl_fus_aqua_cfg aqua_cfg = {
	.alpha = 0.05,
	.beta = 0.7,
	.e_a = 0.9,
	.e_m = 0.9,
};

static struct zsl_fus_drv aqua_drv = {
	.init_handler = zsl_fus_aqua_init,
	.feed_handler = zsl_fus_aqua_feed,
	.error_handler = zsl_fus_aqua_error,
	.config = &aqua_cfg,
};

#elif FUSION_ALGO_EKF

static zsl_real_t _kalm_P[16] = {
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 1.0
};

static struct zsl_fus_kalm_cfg kalm_cfg = {
	.var_g = 0.00174533,
	.var_a = 0.0784,
	.var_m = 0.21,
	.P = {
		.sz_rows = 4,
		.sz_cols = 4,
		.data = _kalm_P,
	},
};

static struct zsl_fus_drv kalm_drv = {
	.init_handler = zsl_fus_kalm_init,
	.feed_handler = zsl_fus_kalm_feed,
	.error_handler = zsl_fus_kalm_error,
	.config = &kalm_cfg,
};

#else
#error "Select a fusion algorithm"
#endif


struct zsl_quat q = { .r = 1.0, .i = 0.0, .j = 0.0, .k = 0.0 };

static struct zsl_euler movement_data_to_angles(
	struct sensor_value accel_meas[3],
	struct sensor_value gyro_meas[3],
	struct sensor_value magn_meas[3])
{
	struct zsl_euler e = { 0 };
	float tmp_accel[3];
	float tmp_gyro[3];
	float tmp_magnetometer[3];
	static bool first_done;
	static float prev_accel_meas[3];
	static float prev_gyro_meas[3];
	static float prev_magn_meas[3];

	ZSL_VECTOR_DEF(av, 3);
	ZSL_VECTOR_DEF(gv, 3);
	// ZSL_VECTOR_DEF(mv, 3);
	zsl_vec_init(&av);
	zsl_vec_init(&gv);
	// zsl_vec_init(&mv);

	for (int i = 0; i < 3; i++) {
		tmp_accel[i] = sensor_value_to_double(&accel_meas[i]);
		tmp_gyro[i] = sensor_value_to_double(&gyro_meas[i]);
		// tmp_magnetometer[i] = sensor_value_to_double(&magn_meas[i]);

		if (first_done) {
			av.data[i] = ACCEL_ALPHA * tmp_accel[i] + (1.0f - ACCEL_ALPHA) * prev_accel_meas[i];
			gv.data[i] = GYRO_ALPHA * tmp_gyro[i] + (1.0f - GYRO_ALPHA) * prev_gyro_meas[i];
			// mv.data[i] = MAGN_ALPHA * tmp_magnetometer[i] + (1.0f - MAGN_ALPHA) * prev_magn_meas[i];
		} else {
			av.data[i] = tmp_accel[i];
			gv.data[i] = tmp_gyro[i];
			// mv.data[i] = tmp_magnetometer[i];
		}

		prev_accel_meas[i] = av.data[i];
		prev_gyro_meas[i] = gv.data[i];
		// prev_magn_meas[i] = mv.data[i];
	}

	LOG_DBG("Accel (X,Y,Z) - (%.2f, %.2f, %.2f)", (float)av.data[0], (float)av.data[1], (float)av.data[2]);
	LOG_DBG("Gyro (X,Y,Z) - (%.2f, %.2f, %.2f)", (float)gv.data[0], (float)gv.data[1], (float)gv.data[2]);
	// LOG_DBG("Magnetometer (X,Y,Z) - (%.2f, %.2f, %.2f)", (float)mv.data[0], (float)mv.data[1], (float)mv.data[2]);

#if FUSION_ALGO_MADGWICK
	madgwick_drv.feed_handler(&av, &mv, &gv, NULL, &q, madgwick_drv.config);
#elif FUSION_ALGO_AQUA
	aqua_drv.feed_handler(&av, NULL, &gv, NULL, &q, aqua_drv.config);
#elif FUSION_ALGO_EKF
	kalm_drv.feed_handler(&av, &mv, &gv, NULL, &q, kalm_drv.config);
#endif

	zsl_quat_to_euler(&q, &e);

	static struct zsl_euler prev_e;

	e.x *= 180. / ZSL_PI;
	e.y *= 180. / ZSL_PI;
	e.z *= 180. / ZSL_PI;

	if (first_done) {
		e.x = e.x * ANGLE_ALPHA + (1.0f - ANGLE_ALPHA) * prev_e.x;
		e.y = e.y * ANGLE_ALPHA + (1.0f - ANGLE_ALPHA) * prev_e.y;
		e.z = e.z * ANGLE_ALPHA + (1.0f - ANGLE_ALPHA) * prev_e.z;
	}

	prev_e.x = e.x;
	prev_e.y = e.y;
	prev_e.z = e.z;

	first_done = true;

	/*
		Pitch = around x-axis
		Roll = around y-axis
		Yaw = around z-axis
	*/

	return e;
}

static void angle_data_send(const struct zsl_euler angles)
{
	struct imu_module_event *imu_module_event =
		new_imu_module_event();
	/* Somehow the angle output swaps roll and pitch*/
	imu_module_event->angles.pitch = (float)angles.x;
	imu_module_event->angles.roll = (float)angles.y;

	imu_module_event->angles.timestamp = k_uptime_get_32();
	imu_module_event->type = IMU_EVT_MOVEMENT_DATA_READY;

	APP_EVENT_SUBMIT(imu_module_event);
}

static int process_mpu9250(const struct device *dev, bool send_msg)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value magnetometer[3];
	int err = sensor_sample_fetch(dev);

	if (err != 0) {
		LOG_ERR("Sample fetch failed: %d\n", err);
		return err;
	}

	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

	if (err != 0) {
		LOG_ERR("sensor acc channel get failed: %d\n", err);
		return err;
	}

	err = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err != 0) {
		LOG_ERR("sensor gyro channel get failed: %d\n", err);
		return err;
	}

	err = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magnetometer);
	if (err != 0) {
		LOG_ERR("sensor magnetometer channel get failed: %d\n", err);
		return err;
	}

	struct zsl_euler angles = movement_data_to_angles(accel, gyro, magnetometer);

	// float angles_raw[3];
	// float accel[3];
	// int err = app_mpu_get_angles((float *)&angles_raw, accel);


	// struct zsl_euler angles = {
	// 	.x = angles_raw[0],
	// 	.y = angles_raw[1],
	// 	.z = angles_raw[2],
	// };


	// if (err) {
	// 	return err;
	// }

	if (send_msg) {
		angle_data_send(angles);
	}

	return err;
}

// Is this really needed? Dunno...
#ifdef CONFIG_MPU9250_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu9250_drdy(const struct device *dev,
								const struct sensor_trigger *trig)
{
	int rc = process_mpu9250(dev);

	if (rc != 0)
	{
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif /* CONFIG_MPU9250_TRIGGER */

static void module_thread_fn(void)
{
	k_sleep(K_SECONDS(2)); // Wait for MPU9250 to initialize

#if FUSION_ALGO_MADGWICK
	madgwick_drv.init_handler((float)CONFIG_IMU_MESSAGE_FREQUENCY, madgwick_drv.config);
#elif FUSION_ALGO_AQUA
	aqua_drv.init_handler((float)CONFIG_IMU_MESSAGE_FREQUENCY, aqua_drv.config);
#elif FUSION_ALGO_EKF
	kalm_drv.init_handler((float)CONFIG_IMU_MESSAGE_FREQUENCY, kalm_drv.config);
#endif

	// app_mpu_init();

	const struct device *mpu9250 = device_get_binding("MPU9250");

	if (mpu9250 == NULL)
	{
		LOG_ERR("MPU9250 not found");
		return;
	}

	LOG_DBG("IMU thread initialized. ms between IMU messages: %d", IMU_THREAD_SLEEP_USEC);

	uint32_t prev_time = 0;

	while (true) {
		uint32_t current_time = k_uptime_get_32();

		if ((current_time - prev_time) >= (CONTROL_LOOP_INTERVAL_MS)) {
			process_mpu9250(mpu9250, true);
			prev_time = current_time;
		} else {
			process_mpu9250(mpu9250, false);
		}

		k_usleep(IMU_THREAD_SLEEP_USEC);
	}
}

static bool event_handler(const struct app_event_header *eh)
{
	// Does nothing with incoming messages right now

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

K_THREAD_DEFINE(imu_module_thread, 8192,
				module_thread_fn, NULL, NULL, NULL,
				1, 0, 0);

APP_EVENT_LISTENER(MODULE, event_handler);
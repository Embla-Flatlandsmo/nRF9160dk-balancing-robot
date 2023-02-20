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

#define IMU_THREAD_SLEEP_MSEC 1000.0 / (float)CONFIG_IMU_MESSAGE_FREQUENCY

#define MODULE imu_module

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_module, CONFIG_IMU_MODULE_LOG_LEVEL);

/* Config settings for the AQUA filter. */
static struct zsl_fus_aqua_cfg aqua_cfg = {
	.alpha = 0.7,
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

/* Config settings for the Madgwick filter. */
// static struct zsl_fus_madg_cfg madg_cfg = {
// 	.beta = 0.174,
// };

// static struct zsl_fus_drv madgwick_drv = {
// 	.init_handler = zsl_fus_madg_init,
// 	.feed_handler = zsl_fus_madg_feed,
// 	.error_handler = zsl_fus_madg_error,
// 	.config = &madg_cfg,
// };


struct zsl_quat q = { .r = 1.0, .i = 0.0, .j = 0.0, .k = 0.0 };

static struct zsl_euler movement_data_to_angles(struct sensor_value accel_meas[3], struct sensor_value gyro_meas[3])
{
	struct zsl_euler e = { 0 };
	ZSL_VECTOR_DEF(av, 3);
	ZSL_VECTOR_DEF(gv, 3);
	zsl_vec_init(&av);
	zsl_vec_init(&gv);
	for (int i = 0; i < 3; i++)
	{
		av.data[i] = sensor_value_to_double(&accel_meas[i]);
		gv.data[i] = sensor_value_to_double(&gyro_meas[i]);
	}

	LOG_DBG("Accel (X,Y,Z) - (%.2f, %.2f, %.2f)", (float)av.data[0], (float)av.data[1], (float)av.data[2]);
	LOG_DBG("Gyro (X,Y,Z) - (%.2f, %.2f, %.2f)", (float)gv.data[0], (float)gv.data[1], (float)gv.data[2]);
	// madgwick_drv.feed_handler(&av, NULL, &gv, NULL, &q, madgwick_drv.config);
	aqua_drv.feed_handler(&av, NULL, &gv, NULL, &q, aqua_drv.config);

	zsl_quat_to_euler(&q, &e);
	e.x *= 180. / ZSL_PI;
	e.y *= 180. / ZSL_PI;
	e.z *= 180. / ZSL_PI;

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

	imu_module_event->angles.timestamp = k_uptime_get();
	imu_module_event->type = IMU_EVT_MOVEMENT_DATA_READY;

	APP_EVENT_SUBMIT(imu_module_event);
}

static int process_mpu9250(const struct device *dev)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];

	int rc = sensor_sample_fetch(dev);
	if (rc != 0)
	{
		LOG_ERR("Sample fetch failed: %d\n", rc);
		return rc;
	}
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
							accel);

	if (rc != 0)
	{
		LOG_ERR("sensor acc channel get failed: %d\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
							gyro);
	if (rc != 0)
	{
		LOG_ERR("sensor gyro channel get failed: %d\n", rc);
		return rc;
	}

	if (rc == 0)
	{
		struct zsl_euler angles = movement_data_to_angles(&accel[0], &gyro[0]);
		angle_data_send(angles);
	}
	else
	{
		LOG_ERR("sample fetch/get failed: %d\n", rc);
	}

	return rc;
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
	/* Init filter at 100 Hz. */

	// aqua_drv.init_handler(1000.0, aqua_drv.config);
	// aqua_drv.init_handler(10.0, aqua_drv.config);
	// madgwick_drv.init_handler(100.0, madgwick_drv.config);
	aqua_drv.init_handler((float)CONFIG_IMU_MESSAGE_FREQUENCY, aqua_drv.config);

	k_sleep(K_SECONDS(2)); // Wait for MPU9250 to initialize
	const struct device *mpu9250 = device_get_binding("MPU9250");
	if (mpu9250 == NULL)
	{
		LOG_ERR("MPU9250 not found");
		return;
	}
	LOG_DBG("IMU thread initialized. ms between IMU messages: %f", IMU_THREAD_SLEEP_MSEC);
	while (true)
	{
		process_mpu9250(mpu9250);
		k_sleep(K_MSEC(IMU_THREAD_SLEEP_MSEC));
	}
}

static bool event_handler(const struct app_event_header *eh)
{
	// Does nothing with incoming messages right now

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

K_THREAD_DEFINE(imu_module_thread, CONFIG_IMU_THREAD_STACK_SIZE,
				module_thread_fn, NULL, NULL, NULL,
				1, 0, 0);

APP_EVENT_LISTENER(MODULE, event_handler);
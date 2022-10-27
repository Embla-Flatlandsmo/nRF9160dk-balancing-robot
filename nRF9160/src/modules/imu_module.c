/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <assert.h>

#include <zsl/zsl.h>
#include <zsl/orientation/orientation.h>
#include <zsl/instrumentation.h>

#include <stdio.h>

#include "imu_module_event.h"

#include "modules_common.h"
#include "util/dmp/sensor_processing_lib.h"
#include "util/dmp/quaternion.h"
#define IMU_THREAD_PRIORITY -1
#define IMU_THREAD_SLEEP 1.0 / (float)CONFIG_IMU_MESSAGE_FREQUENCY

#define MODULE imu_module

#include <logging/log.h>
LOG_MODULE_REGISTER(imu_module, CONFIG_IMU_MODULE_LOG_LEVEL);

static int64_t update_time_delta;
vector_ijk fused_vector;

static struct euler_angles movement_data_to_angles(struct sensor_value accel_meas[3], struct sensor_value gyro_meas[3], struct sensor_value magn_meas[3])
{
	assert((ACCELEROMETER_CHANNELS == 3) && (GYROSCOPE_CHANNELS == 3));
	float accel[3], gyro[3], magn[3];
	int64_t dt = k_uptime_delta(&update_time_delta);
	float delta = dt/1000.0;
	for (int i = 0; i < 3; i++)
	{
		accel[i] = (float)sensor_value_to_double(&accel_meas[i]);
		gyro[i] = (float)sensor_value_to_double(&gyro_meas[i]);
		magn[i] = (float)sensor_value_to_double(&magn_meas[i]);
	}
	LOG_DBG("Accel (X,Y,Z) - (%.2f, %.2f, %.2f)", accel[0], accel[1], accel[2]);
	LOG_DBG("Gyro (X,Y,Z) - (%.2f, %.2f, %.2f)", gyro[0], gyro[1], gyro[2]);
	LOG_DBG("Magn (X,Y,Z) - (%.2f, %.2f, %.2f)", magn[0], magn[1], magn[2]);
	/*
		Pitch = around x-axis
		Roll = around y-axis
		Yaw = around z-axis
	*/
	fused_vector = update_fused_vector(fused_vector, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], delta);
	Quaternion q_acc = quaternion_from_accelerometer(fused_vector.a, fused_vector.b, fused_vector.c);
	return quaternion_to_euler_angles(q_acc);
}

static void angle_data_send(const struct euler_angles angles)
{
	struct imu_module_event *imu_module_event =
		new_imu_module_event();
	/* Somehow the angle output swaps roll and pitch*/
	imu_module_event->angles.values[0] = angles.roll;
	imu_module_event->angles.values[1] = angles.pitch;
	imu_module_event->angles.values[2] = angles.yaw;
	
	imu_module_event->angles.timestamp = k_uptime_get();
	imu_module_event->type = IMU_EVT_MOVEMENT_DATA_READY;
	// LOG_DBG("Submitted IMU event.");
	APP_EVENT_SUBMIT(imu_module_event);
}

// static void movement_data_send(struct sensor_value accel[3], struct sensor_value gyro[3])
// {
// 	struct imu_module_event *imu_module_event =
// 		new_imu_module_event();
	
// 	for (int i = 0; i < ACCELEROMETER_CHANNELS; i++)
// 	{
// 		imu_module_event->accel.values[i] = sensor_value_to_double(&accel[i]);
// 	}
// 	for (int i = 0; i < GYROSCOPE_CHANNELS; i++)
// 	{
// 		imu_module_event->gyro.values[i] = sensor_value_to_double(&gyro[i]);
// 	}
// 	imu_module_event->accel.timestamp = k_uptime_get();
// 	imu_module_event->gyro.timestamp = k_uptime_get();
// 	imu_module_event->type = IMU_EVT_MOVEMENT_DATA_READY;
// 	LOG_DBG("Submitted IMU event.");
// 	APP_EVENT_SUBMIT(imu_module_event);
// }

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
			 h, min, s, ms);
	return buf;
}

static int process_mpu9250(const struct device *dev)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value magn[3];
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

	rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ,
							magn);
	if (rc != 0)
	{
		LOG_ERR("sensor gyro channel get failed: %d\n", rc);
		return rc;
	}

	if (rc == 0)
	{
		euler_angles angles = movement_data_to_angles(&accel[0], &gyro[0], &magn[0]);
		angle_data_send(angles);
		// movement_data_send(&accel[0], &gyro[0]);
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
	fused_vector = vector_3d_initialize(0.0, 0.0, 1.0);
	k_sleep(K_SECONDS(2)); // Wait for MPU9250 to initialize
	const struct device *mpu9250 = device_get_binding("MPU9250");
	if (mpu9250 == NULL)
	{
		LOG_ERR("MPU9250 not found");
		return;
	}
	LOG_DBG("IMU thread initialized");
	update_time_delta = k_uptime_get();
	while (true)
	{
		process_mpu9250(mpu9250);
		k_sleep(K_SECONDS(IMU_THREAD_SLEEP));
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
				K_HIGHEST_APPLICATION_THREAD_PRIO, 0, 0);

APP_EVENT_LISTENER(MODULE, event_handler);
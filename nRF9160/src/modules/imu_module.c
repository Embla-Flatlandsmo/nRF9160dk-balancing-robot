/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#include <stdio.h>

#include "imu_module_event.h"

// #include "measurement_event.h"
#include "control_event.h"
#include "ack_event.h"
#include "config_event.h"

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY -1
#define IMU_THREAD_SLEEP 2000

#define MODULE imu_module

#include <logging/log.h>
LOG_MODULE_REGISTER(imu_module, CONFIG_IMU_MODULE_LOG_LEVEL);

static int8_t value1 = 1;
static K_THREAD_STACK_DEFINE(imu_thread_stack,
							 IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread;

static void movement_data_send(struct sensor_value accel[3], struct sensor_value gyro[3])
{
	struct imu_module_event *imu_module_event =
		new_imu_module_event();
	
	// imu_module_event->accel.values[0] = 3.14;
	for (int i = 0; i < ACCELEROMETER_CHANNELS; i++)
	{
		// imu_module_event->accel.values[i] = 3.14;
		imu_module_event->accel.values[i] = sensor_value_to_double(&accel[i]);
	}
	for (int i = 0; i < GYROSCOPE_CHANNELS; i++)
	{
		imu_module_event->gyro.values[i] = sensor_value_to_double(&gyro[i]);
	}
	imu_module_event->accel.timestamp = k_uptime_get();
	imu_module_event->gyro.timestamp = k_uptime_get();
	imu_module_event->type = IMU_EVT_MOVEMENT_DATA_READY;

	LOG_INF("Accel value 0: %f", imu_module_event->accel.values[0]);

	// accelerometer_callback_set(false);
	EVENT_SUBMIT(imu_module_event);
}

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
	// struct sensor_value 
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

	// rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ,
	// 						magn);
	// if (rc != 0)
	// {
	// 	printf("sensor magn channel get failed: %d\n", rc);
	// 	return rc;
	// }
	// int32_t accel_test = accel[0].val1;
	// LOG_INF("Accel_test: %d", accel_test);
	// k_spinlock_key_t key = k_spin_lock(&data_lock);
	// double accel_value = sensor_value_to_double(&accel[0]);
	// k_spin_unlock(&data_lock, key);
	if (rc == 0)
	{
		// printk("Accel0: %f", accel_value);
		// LOG_INF("Accel0: %f", accel_value);
		// LOG_INF("Accel0: %f", sensor_value_to_double(&accel[0]));
		// LOG_INF("Accel1: %f", sensor_value_to_double(&accel[1]));
		// LOG_INF("Accel2: %f", sensor_value_to_double(&accel[2]));
		LOG_INF("Accel values: %f %f %f",
			sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
		// printf("[%s]:\n"
		// 	   " accel %f %f %f m/s/s\n"
		// 	   " gyro  %f %f %f rad/s\n",
		// 	   now_str(),
		// 	   sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
		// 	   sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
		// 	   sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
		movement_data_send(&accel[0], &gyro[0]);
	}
	else
	{
		LOG_ERR("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

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

static void imu_thread_fn(void)
{
	// const char *const label = DT_LABEL(DT_NODELABEL(mpu9250));
	// const char *const label = DT_LABEL((accelerometer));

	// if (label == NULL) {
	// 	LOG_WRN("label cannot be null");
	// 	return;
	// }

	// const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050)); // From old sample
	k_sleep(K_SECONDS(5)); // Wait for MPU9250 to initialize
	const struct device *mpu9250 = device_get_binding("MPU9250");
	if (mpu9250 == NULL)
	{
		LOG_INF("mpu9250 is null >:(");
		return;
	}
	LOG_INF("Initialized imu thread :)");

	while (true)
	{
		process_mpu9250(mpu9250);
		k_sleep(K_MSEC(IMU_THREAD_SLEEP));
	}
}

static void init(void)
{
	k_thread_create(&imu_thread,
					imu_thread_stack,
					IMU_THREAD_STACK_SIZE,
					(k_thread_entry_t)imu_thread_fn,
					NULL, NULL, NULL,
					IMU_THREAD_PRIORITY,
					0, K_NO_WAIT);
}

static bool event_handler(const struct event_header *eh)
{
	if (is_control_event(eh))
	{
		value1 = -value1;
		struct ack_event *ack = new_ack_event();

		EVENT_SUBMIT(ack);
		return false;
	}

	if (is_config_event(eh))
	{
		struct config_event *ce = cast_config_event(eh);

		value1 = ce->init_value1;
		init();
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, control_event);
EVENT_SUBSCRIBE(MODULE, config_event);

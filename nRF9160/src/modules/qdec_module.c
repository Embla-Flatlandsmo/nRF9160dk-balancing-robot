/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <float.h>

#define MODULE qdec_module
#include <caf/events/module_state_event.h>
#include <app_event_manager.h>
#include "modules_common.h"
#include "events/qdec_module_event.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_QDEC_MODULE_LOG_LEVEL);

// const struct device *qdec_dev = DEVICE_DT_GET(DT_ALIAS(motora));
static float encoder_travels[2];

#define DT_MSEC 1000.0/(float)CONFIG_QDEC_MESSAGE_FREQUENCY

// void process_shield(const struct device *dev)
// {
// 	struct sensor_value rot[2];

// 	int rc = sensor_sample_fetch(dev);
// 	if (rc != 0)
// 	{
// 		LOG_ERR("Sample fetch failed: %d\n", rc);
// 	}
// 	if (rc == 0)
// 	{
// 		rc = sensor_channel_get(dev, SENSOR_CHAN_ROTATION,
// 								rot);
// 	}
// 	if (rc == 0)
// 	{
// 		float rot_a = (float)(sensor_value_to_double(&rot[0]));
// 		float rot_b = (float)(sensor_value_to_double(&rot[1]));
// 		encoder_travels[0] = rot_a;
// 		encoder_travels[1] = rot_b;
// 		LOG_DBG("qdec A: %f, qdec B: %f", rot_a, rot_b);
// 	}
// }

static void send_data_evt(float travel)
{
	struct qdec_module_event *qdec_module_event = new_qdec_module_event();
	qdec_module_event->type = QDEC_EVT_DATA_READY;
	qdec_module_event->travel = travel;
	APP_EVENT_SUBMIT(qdec_module_event);
}

void process_qdec_data(const struct device *dev)
{
	struct sensor_value rot[2];
	int err;
	err = sensor_sample_fetch(dev);
	if (err != 0)
	{
		LOG_ERR("Qdec sensor_sample_fetch error: %d\n", err);
		return;
	}
	err = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, rot);
	if (err != 0)
	{
		LOG_ERR("Qdec A sensor_channel_get error: %d\n", err);
		return;
	}

	float rot_a = (float)(sensor_value_to_double(&rot[0]));
	float rot_b = (float)(sensor_value_to_double(&rot[1]));
	encoder_travels[0] = rot_a;
	encoder_travels[1] = rot_b;
	send_data_evt(rot_a);
}

static void module_thread_fn(void)
{
	const char *const label = DT_LABEL(DT_NODELABEL(shield_qdec));
	const struct device *qdec_dev = device_get_binding(label);
	if (!qdec_dev)
	{
		LOG_ERR("Failed to find shield");
	}
	LOG_DBG("IMU thread initialized. ms between QDEC messages: %f", DT_MSEC);
	while (true)
	{
		process_qdec_data(qdec_dev);
		k_sleep(K_MSEC(DT_MSEC));
	}
}

static bool event_handler(const struct app_event_header *eh)
{
	// Does nothing with incoming messages right now

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

K_THREAD_DEFINE(qdec_module_thread, CONFIG_QDEC_THREAD_STACK_SIZE,
				module_thread_fn, NULL, NULL, NULL,
				1, 0, 0);

APP_EVENT_LISTENER(MODULE, event_handler);

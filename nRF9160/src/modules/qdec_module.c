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

const struct device *qdec_dev = DEVICE_DT_GET(DT_ALIAS(motora));
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

static void data_evt_timeout_work_handler(struct k_work *work);
K_WORK_DEFINE(data_evt_timeout_work, data_evt_timeout_work_handler);

void data_evt_timeout_handler(struct k_timer *dummy)
{
    k_work_submit(&data_evt_timeout_work);
}

K_TIMER_DEFINE(data_evt_timeout, data_evt_timeout_handler, NULL);

void data_evt_timeout_work_handler(struct k_work *work)
{
	struct sensor_value rot[2];
	int err;
	err = sensor_sample_fetch(qdec_dev);
	if (err != 0)
	{
		LOG_ERR("Qdec sensor_sample_fetch error: %d\n", err);
		return;
	}
	err = sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &rot);
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

static int module_init(void)
{
	const char *const label = DT_LABEL(DT_NODELABEL(shield_qdec));
	qdec_dev = device_get_binding(label);
	if (!qdec_dev)
	{
		LOG_ERR("Failed to find shield");
	}

	k_timer_start(&data_evt_timeout, K_NO_WAIT, K_MSEC(DT_MSEC));
	return 0;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_module_state_event(aeh)) {
		const struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			
            if (module_init())
            {
                LOG_ERR("QDEC module init failed");

                return false;
            }
            LOG_INF("QDEC module initialized");
            module_set_state(MODULE_STATE_READY);
		}

		return false;
	}
	/* Event not handled but subscribed. */
	__ASSERT_NO_MSG(false);
	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
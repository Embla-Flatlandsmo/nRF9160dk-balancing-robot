/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <zephyr/kernel.h>
#include <float.h>

#define MODULE qdec_module
#include <caf/events/module_state_event.h>
#include <app_event_manager.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include "modules_common.h"
#include "events/qdec_module_event.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_QDEC_MODULE_LOG_LEVEL);

struct device* qdec_dev;

#define DT_MSEC 1000.0/(float)CONFIG_QDEC_MESSAGE_FREQUENCY

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
	struct sensor_value rot;
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

	float qdec_travel = (float)sensor_value_to_double(&rot);
	send_data_evt(qdec_travel);
}

static int module_init(void)
{
	// if (!IS_ENABLED(CONFIG_QDEC_SIMULATE_INPUT))
	// {
	// 	qdec_dev = device_get_binding(DT_LABEL(DT_NODELABEL(qdec)));
	// 	if (qdec_dev == NULL)
	// 	{
	// 		LOG_ERR("Failed to get bindings for qdec devices");
	// 		return -ENODEV;
	// 	}
	// } else {
	// 	LOG_DBG("Using simulated qdec inputs");
	// }

	// k_timer_start(&data_evt_timeout, K_NO_WAIT, K_MSEC(DT_MSEC));
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
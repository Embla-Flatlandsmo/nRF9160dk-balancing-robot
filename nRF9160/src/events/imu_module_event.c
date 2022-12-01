/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>

#include "imu_module_event.h"

static char *get_evt_type_str(enum imu_module_event_type type)
{
    switch (type)
    {
    case IMU_EVT_MOVEMENT_DATA_READY:
        return "IMU_EVT_MOVEMENT_DATA_READY";
    case IMU_EVT_ACCELEROMETER_ERROR:
        return "IMU_EVT_ACCELEROMETER_ERROR";
    case IMU_EVT_GYROSCOPE_ERROR:
        return "IMU_EVT_GYROSCOPE_ERROR";
    default:
        return "Unknown event";
    }
}

static void profile_imu_module_event(struct log_event_buf *buf,
                                      const struct app_event_header *aeh)
{
}

APP_EVENT_INFO_DEFINE(imu_module_event,
                      ENCODE(),
                      ENCODE(),
                      profile_imu_module_event);

static void log_imu_event(const struct app_event_header *eh)
{
    const struct imu_module_event *event = cast_imu_module_event(eh);

    if (event->type == IMU_EVT_ACCELEROMETER_ERROR ||
        event->type == IMU_EVT_GYROSCOPE_ERROR)
    {
        APP_EVENT_MANAGER_LOG(eh, "%s - Error code %d",
            get_evt_type_str(event->type), event->err);
    }
    else if (event->type == IMU_EVT_MOVEMENT_DATA_READY)
    {
        APP_EVENT_MANAGER_LOG(eh, "%s - Angles (P, R, Y): (%.2f, %.2f, %.2f)",
                          get_evt_type_str(event->type),
                          event->angles.pitch,
                          event->angles.roll,
                          0.0);
    }
    else
    {
        APP_EVENT_MANAGER_LOG(eh, "%s", get_evt_type_str(event->type));
    }
}

APP_EVENT_TYPE_DEFINE(imu_module_event,
                    log_imu_event,
                    &imu_module_event_info,
                    APP_EVENT_FLAGS_CREATE(
                        IF_ENABLED(CONFIG_IMU_EVENTS_LOG,
                        (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));
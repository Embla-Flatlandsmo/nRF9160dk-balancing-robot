/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

static int log_event(const struct event_header *eh, char *buf,
                     size_t buf_len)
{
    const struct imu_module_event *event = cast_imu_module_event(eh);

    if (event->type == IMU_EVT_ACCELEROMETER_ERROR ||
        event->type == IMU_EVT_GYROSCOPE_ERROR)
    {
        EVENT_MANAGER_LOG(eh, "%s - Error code %d",
            get_evt_type_str(event->type), event->err);
    }
    else if (event->type == IMU_EVT_MOVEMENT_DATA_READY)
    {
        EVENT_MANAGER_LOG(eh, "%s - Acc (X, Y, Z): (%.2f, %.2f, %.2f)",
                          get_evt_type_str(event->type),
                          event->accel.values[0],
                          event->accel.values[1],
                          event->accel.values[2]);
        EVENT_MANAGER_LOG(eh, "%s - Gyr (X, Y, Z): (%.2f, %.2f, %.2f)",
                          get_evt_type_str(event->type),
                          event->gyro.values[0],
                          event->gyro.values[1],
                          event->gyro.values[2]);
    }
    else
    {
        EVENT_MANAGER_LOG(eh, "%s", get_evt_type_str(event->type));
    }
    return 0;
}

#if defined(CONFIG_PROFILER)

static void profile_event(struct log_event_buf *buf,
                          const struct event_header *eh)
{
    const struct imu_module_event *event = cast_imu_module_event(eh);

#if defined(CONFIG_PROFILER_EVENT_TYPE_STRING)
    profiler_log_encode_string(buf, get_evt_type_str(event->type));
#else
    profiler_log_encode_uint8(buf, event->type);
#endif
}

EVENT_INFO_DEFINE(imu_module_event,
#if defined(CONFIG_PROFILER_EVENT_TYPE_STRING)
                  ENCODE(PROFILER_ARG_STRING),
#else
                  ENCODE(PROFILER_ARG_U8),
#endif
                  ENCODE("type"),
                  profile_event);

#endif /* CONFIG_PROFILER */

EVENT_TYPE_DEFINE(imu_module_event,
                  CONFIG_IMU_EVENTS_LOG,
                  log_event,
#if defined(CONFIG_PROFILER)
                  &imu_module_event_info);
#else
                  NULL);
#endif

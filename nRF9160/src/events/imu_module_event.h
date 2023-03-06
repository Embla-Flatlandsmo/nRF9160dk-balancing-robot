/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _IMU_MODULE_EVENT_H_
#define _IMU_MODULE_EVENT_H_

/**
 * @brief IMU Event
 * @defgroup imu_event IMU Event
 * @{
 */

#include <app_event_manager.h>
#include <app_event_manager_profiler_tracer.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Enum containing types of imu events emitted. */
    enum imu_module_event_type
    {
        /** Motion detected. Acceleration of the device has exceeded the configured threshold.
         *  Payload is of type @ref imu_module_accel_data (accel). The associated acceleration
         *  values contains the motion that caused the device to exceed the configured threshold.
         */
        IMU_EVT_MOVEMENT_DATA_READY,

        /** Events propagated when an error associated with the sensor device occurs. */
        IMU_EVT_ACCELEROMETER_ERROR,
        IMU_EVT_GYROSCOPE_ERROR
    };

    struct imu_module_angles
    {
        uint32_t timestamp;
        float pitch;
        float roll;
    };

    struct imu_module_event
    {
        struct app_event_header header;
        /** Sensor module event type. */
        enum imu_module_event_type type;
        int err;

        struct imu_module_angles angles;

    };

    APP_EVENT_TYPE_DECLARE(imu_module_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _IMU_MODULE_EVENT_H_ */

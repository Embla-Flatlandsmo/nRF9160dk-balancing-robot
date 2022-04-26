/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
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

#include <event_manager.h>
#include <event_manager_profiler_tracer.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Number of accelerometer channels. */
#define ACCELEROMETER_CHANNELS 3
#define GYROSCOPE_CHANNELS 3

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

    /** @brief Structure used to provide acceleration data. */
    struct imu_module_accel_data
    {
        /** Uptime when the data was sampled. */
        int64_t timestamp;
        /** Acceleration in X, Y and Z planes in m/s2. */
        double values[ACCELEROMETER_CHANNELS];
    };

    /** @brief Structure used to provide angular velocity data. */
    struct imu_module_gyro_data
    {
        /** Uptime when the data was sampled. */
        int64_t timestamp;
        /** Angular velocity in X, Y and Z axis in deg/sec. */
        double values[GYROSCOPE_CHANNELS];
    };

    struct imu_module_event
    {
        struct event_header header;
        /** Sensor module event type. */
        enum imu_module_event_type type;
        int err;

        struct imu_module_accel_data accel;
        struct imu_module_gyro_data gyro;

    };

    EVENT_TYPE_DECLARE(imu_module_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _IMU_MODULE_EVENT_H_ */

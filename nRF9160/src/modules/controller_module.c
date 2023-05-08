/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <stdio.h>
#define MODULE controller_module
#include "modules_common.h"
#include "events/imu_module_event.h"
#include "events/qdec_module_event.h"
// #include <caf/events/click_event.h>
#include "../../drivers/motors/motor.h"

// CMSIS PID controller
// #include <arm_math.h>

#include <math.h>
#include "util/pid.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(controller_module, CONFIG_CONTROLLER_MODULE_LOG_LEVEL);

// #define KP_PITCH 14.5f
// #define KI_PITCH 20.0f
// #define KD_PITCH 0.3f
#define KP_PITCH 20.0f
#define KI_PITCH 5.0f
#define KD_PITCH 0.0f

// #define KP_SPEED            1.0f
// #define KI_SPEED            0.0f
// #define KD_SPEED            0.1f

#define KP_SPEED            0.001f
#define KI_SPEED            0.0f
#define KD_SPEED            0.0f

// #define PITCH_OUTPUT_ALPHA  0.7f
#define PITCH_OUTPUT_ALPHA 0.7f

#define PITCH_SATURATION_VALUE 100.0f

#define ANGLE_NORMALIZED_CONSTANT 86.5f
// #define ANGLE_NORMALIZED_CONSTANT 0.0f
#define STATIC_SET_POINT            -ANGLE_NORMALIZED_CONSTANT

#define MOTOR_SPEED_MAX     128

//========================================================================================
/*                                                                                      *
 *                                     Structs etc                                      *
 *                                                                                      */
//========================================================================================

PID_t PID_pitch;
PID_t PID_speed;


float prev_pitch = -87.0;
float prev_position = 0.0;

struct controller_msg_data
{
    union
    {
        struct imu_module_event imu;
        struct qdec_module_event qdec;
    } module;
};


int64_t speed_dt;
int64_t pitch_dt;

float prev_pitch_output = 0.0;

/* Controller module message queue. */
#define CONTROLLER_QUEUE_ENTRY_COUNT 30
#define CONTROLLER_QUEUE_BYTE_ALIGNMENT 4

K_MSGQ_DEFINE(msgq_controller, sizeof(struct controller_msg_data),
              CONTROLLER_QUEUE_ENTRY_COUNT, CONTROLLER_QUEUE_BYTE_ALIGNMENT);

static struct module_data self = {
    .name = "controller",
    .msg_q = &msgq_controller,
    .supports_shutdown = true,
};

const struct device *device_motor_a = DEVICE_DT_GET(DT_ALIAS(motora));
const struct device *device_motor_b = DEVICE_DT_GET(DT_ALIAS(motorb));

//========================================================================================
/*                                                                                      *
 *                                      Motor                                           *
 *                                                                                      */
//========================================================================================
static int init_motors()
{
    LOG_DBG("Initializing motor drivers");
    int err;
    err = !device_is_ready(device_motor_a);
    if (err)
    {
        LOG_ERR("Motor a not ready: Error %d", err);
        return err;
    }

    err = !device_is_ready(device_motor_b);
    if (err)
    {
        LOG_ERR("Motor b not ready: Error %d", err);
        return err;
    }

    return 0;
}

static int set_motor_speed(float speed)
{
    uint32_t motor_speed;

    if (speed < 0.0)
    {
        motor_speed = (uint32_t)CLAMP((-1.0 * speed), 0, MOTOR_SPEED_MAX);
        motor_drive_continous(device_motor_a, motor_speed, MOTOR_SPEED_MAX, 1);
        motor_drive_continous(device_motor_b, motor_speed, MOTOR_SPEED_MAX, 0);
    } else if (speed > 0.0)
    {
        motor_speed = (uint32_t)CLAMP(speed, 0, MOTOR_SPEED_MAX);
        motor_drive_continous(device_motor_a, motor_speed, MOTOR_SPEED_MAX, 0);
        motor_drive_continous(device_motor_b, motor_speed, MOTOR_SPEED_MAX, 1);
    } else {
        motor_drive_continous(device_motor_a, 0, MOTOR_SPEED_MAX, 0);
        motor_drive_continous(device_motor_b, 0, MOTOR_SPEED_MAX, 0);
    }

    return 0;
}

//========================================================================================
/*                                                                                      *
 *                                     Controller                                       *
 *                                                                                      */
//========================================================================================
void update_speed_controller(float position)
{
    static uint32_t prev_elapsed_time;
    uint32_t current_time = k_uptime_get_32();
    uint32_t delta_time = current_time - prev_elapsed_time;
    prev_elapsed_time = current_time;
    float delta_time_s = (float)delta_time / 1000.0f;
    float speed = (position - prev_position) / delta_time_s;
    float speed_output = update_PID(&PID_speed, -speed, delta_time_s);
    speed_output = CLAMP(speed_output, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

    printk("speed_output: %f\n", speed_output);

    prev_position = position;
    PID_pitch.set_point = speed_output;
    return;
}


void update_controller(float pitch)
{
    float pitch_angle = pitch + ANGLE_NORMALIZED_CONSTANT; // Adding constant to center around zero degrees at upright position
    // LOG_DBG("Input pitch angle: %f", pitch);
    LOG_DBG("Normalized Pitch angle: %f", pitch_angle);

    if (pitch_angle * pitch_angle > CONFIG_ANGLE_FAILSAFE_LIMIT * CONFIG_ANGLE_FAILSAFE_LIMIT) {
        set_motor_speed(0.0);

        return;
    }

    static uint32_t prev_elapsed_time;
    static float prev_pitch_output;
    uint32_t current_time = k_uptime_get_32();
    uint32_t delta_time = current_time - prev_elapsed_time;
    prev_elapsed_time = current_time;
    float delta_time_s = (float)delta_time / 1000.0f;
    float pitch_output = update_PID(&PID_pitch, pitch_angle, delta_time_s);

    static uint32_t prev_time;
    prev_time = current_time;

    static bool first = true;

    if (first) {
        prev_pitch_output = pitch_output;
        first = false;
    }

    float controller_output = pitch_output * PITCH_OUTPUT_ALPHA + (1 - PITCH_OUTPUT_ALPHA) * prev_pitch_output;
    prev_pitch_output = pitch_output;

    set_motor_speed(controller_output);
}

//========================================================================================
/*                                                                                      *
 *                             Setup and other utils                                    *
 *                                                                                      */
//========================================================================================

int setup()
{
    PID_pitch.set_point = (float)(STATIC_SET_POINT + ANGLE_NORMALIZED_CONSTANT);
    PID_pitch.error = 0.0f;
    PID_pitch.last_error = 0.0f;
    PID_pitch.kp = (float)KP_PITCH;
    PID_pitch.ki = (float)KI_PITCH;
    PID_pitch.kd = (float)KD_PITCH;
    PID_pitch.i_lb = -(float)PITCH_SATURATION_VALUE;
    PID_pitch.i_ub = (float)PITCH_SATURATION_VALUE;

    // PID_pitch.Kp = CONFIG_KP_PITCH / 1000.0;
    // PID_pitch.Ki = CONFIG_KI_PITCH / 1000.0;
    // PID_pitch.Kd = CONFIG_KD_PITCH / 1000.0;

    PID_speed.kp = (float)KP_SPEED;
    PID_speed.ki = (float)KI_SPEED;
    PID_speed.kd = (float)KD_SPEED;
    PID_speed.i_lb = -(float)CONFIG_SPEED_INTEGRATION_LIMITS;
    PID_speed.i_ub = (float)CONFIG_SPEED_INTEGRATION_LIMITS;

    LOG_DBG("Pitch PID initialized with Kp = %.3f, Ki = %.3f, Kd = %.3f, i_lb = %.3f, i_ub = %.3f",
            PID_pitch.kp, PID_pitch.ki, PID_pitch.kd,
            PID_pitch.i_lb, PID_pitch.i_ub);

    LOG_DBG("Speed PID initialized with Kp = %.3f, Ki = %.3f, Kd = %.3f, i_lb = %.3f, i_ub = %.3f",
            PID_speed.kp, PID_speed.ki, PID_speed.kd,
            PID_speed.i_lb, PID_speed.i_ub);

    init_motors();
    pitch_dt = k_uptime_get();
    speed_dt = k_uptime_get();
    return 0;
}

//========================================================================================
/*                                                                                      *
 *                                    Event handlers                                    *
 *                                                                                      */
//========================================================================================

static bool app_event_handler(const struct app_event_header *eh)
{
    struct controller_msg_data msg = {0};
    bool enqueue_msg = false;


    if (is_qdec_module_event(eh))
    {
        struct qdec_module_event *event = cast_qdec_module_event(eh);
        msg.module.qdec = *event;

        enqueue_msg = true;
    }

    if (is_imu_module_event(eh))
    {
        struct imu_module_event *event = cast_imu_module_event(eh);
        msg.module.imu = *event;

        enqueue_msg = true;
    }

    if (enqueue_msg)
    {
        int err = module_enqueue_msg(&self, &msg);
        if (err)
        {
            LOG_ERR("Message could not be queued");
        }
    }
    return false;
}

//========================================================================================
/*                                                                                      *
 *                                     Module thread                                    *
 *                                                                                      */
//========================================================================================

static void module_thread_fn(void)
{
    LOG_DBG("Controller thread started");
    int err;
    struct controller_msg_data msg;
    self.thread_id = k_current_get();

    err = module_start(&self);
    if (err)
    {
        LOG_ERR("Failed starting module, error: %d", err);
    }

    err = setup();
    if (err)
    {
        LOG_ERR("setup, error %d", err);
    }

    while (true)
    {
        err = module_get_next_msg(&self, &msg, K_FOREVER);
        if (err == 0)
        {
            if (IS_EVENT((&msg), imu, IMU_EVT_MOVEMENT_DATA_READY))
            {
                update_controller(msg.module.imu.angles.pitch);
            }
            if (IS_EVENT((&msg), qdec, QDEC_EVT_DATA_READY))
            {
                update_speed_controller(msg.module.qdec.travel);
            }
        } 
    }

}

K_THREAD_DEFINE(controller_module_thread, CONFIG_CONTROLLER_THREAD_STACK_SIZE,
                module_thread_fn, NULL, NULL, NULL,
                K_HIGHEST_APPLICATION_THREAD_PRIO, 0, 0);

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, imu_module_event);
APP_EVENT_SUBSCRIBE(MODULE, qdec_module_event);
APP_EVENT_SUBSCRIBE(MODULE, click_event);
/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#include <stdio.h>

#include "modules_common.h"
#include "events/imu_module_event.h"
#include "../../drivers/motors/motor.h"

#include <util/pid.h>

#define MODULE controller_module

#include <logging/log.h>
LOG_MODULE_REGISTER(controller_module, CONFIG_CONTROLLER_MODULE_LOG_LEVEL);

#define ANGLE_NORMALIZED_CONSTANT -CONFIG_STATIC_SET_POINT_PITCH

//========================================================================================
/*                                                                                      *
 *                                     Structs etc                                      *
 *                                                                                      */
//========================================================================================

static struct PID_time_t
{
    float prev_speed_time_ms;
    float prev_pitch_time_ms;
} pid_time;

static PID_t PID_pitch;
static PID_t PID_speed;

struct controller_msg_data
{
    union
    {
        struct imu_module_event imu;
    } module;
};

/* Controller module message queue. */
#define CONTROLLER_QUEUE_ENTRY_COUNT 10
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
    // LOG_DBG("Initializing motor drivers");
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

static int set_motor_speed(uint8_t speed)
{
    motor_drive_continous(device_motor_a, speed, 100, 1);
    motor_drive_continous(device_motor_b, speed, 100, 1);
    // k_work_schedule(&stop_motor_work, K_MSEC(time));
    return 0;
}

//========================================================================================
/*                                                                                      *
 *                                     Controller                                       *
 *                                                                                      */
//========================================================================================

void controller_init(void)
{
    PID_speed.set_point = CONFIG_STATIC_SET_POINT_SPEED;
    PID_speed.error = 0.0f;
    PID_speed.last_error = 0.0f;
    PID_speed.kp = CONFIG_KP_SPEED;
    PID_speed.ki = CONFIG_KI_SPEED;
    PID_speed.kd = CONFIG_KI_SPEED;
    PID_speed.i_lb = -CONFIG_SPEED_INTEGRATION_LIMITS;
    PID_speed.i_ub = CONFIG_SPEED_INTEGRATION_LIMITS;

    PID_pitch.set_point = CONFIG_STATIC_SET_POINT_PITCH + ANGLE_NORMALIZED_CONSTANT;
    PID_pitch.error = 0.0f;
    PID_pitch.last_error = 0.0f;
    PID_pitch.kp = CONFIG_KP_PITCH;
    PID_pitch.ki = CONFIG_KI_PITCH;
    PID_pitch.kd = CONFIG_KI_PITCH;
    PID_pitch.i_lb = CONFIG_PITCH_INTEGRATION_LIMITS;
    PID_pitch.i_ub = CONFIG_PITCH_INTEGRATION_LIMITS;

    /*    Yaw not in use    */
    //    PID_yaw.set_point    = 0;
    //    PID_yaw.error        = 0;
    //    PID_yaw.last_error   = 0;
    //    PID_yaw.kp           = KP_yaw;
    //    PID_yaw.ki           = KI_yaw;
    //    PID_yaw.kd           = KD_yaw;
    //    PID_yaw.i_lb         = PID_LOWER_INTEGRATION_LIMIT;
    //    PID_yaw.i_ub         = PID_UPPER_INTEGRATION_LIMIT;

    LOG_DBG("Controller initialized.");
}

/* Collect accelerometer data to add feeforward control action */

float update_feedforward(short disturbance)
{
    return 0;
}


void update_controller(float speed, float pitch_angle, short disturbance)
{
    // I don't like this function, I think it doesn't read well.
    // It's halfway rewritten from the base project.
    static float speed_output = 0;
    static float pitch_output = 0;
    static float yaw_output = 0;
    static float controller_output[2] = {0};
    static uint8_t loop_counter = 0;
    float current_time_ms;
    static bool first = true;
    static float prev_pitch_output;

    pitch_angle += ANGLE_NORMALIZED_CONSTANT; // Adding constant to center around zero degrees at upright position

    if (pitch_angle * pitch_angle <= CONFIG_ANGLE_FAILSAFE_LIMIT * CONFIG_ANGLE_FAILSAFE_LIMIT && !first)
    {
        current_time_ms = k_uptime_get();

        speed_output = update_PID(&PID_speed, speed, ((float)current_time_ms - (float)pid_time.prev_speed_time_ms) / 1000.0f);
        speed_output = CLAMP(speed_output, -CONFIG_SPEED_OUTPUT_LIMITS, CONFIG_SPEED_OUTPUT_LIMITS);

        PID_pitch.set_point = speed_output;
        pid_time.prev_speed_time_ms = current_time_ms;

        LOG_DBG("Speed output: %d", speed_output);

        pitch_output = update_PID(&PID_pitch, pitch_angle, ((float)current_time_ms - (float)pid_time.prev_pitch_time_ms) / 1000.0f);
        pitch_output = 0.7f * prev_pitch_output + 0.3f * pitch_output;
        prev_pitch_output = pitch_output;
        pid_time.prev_pitch_time_ms = current_time_ms;

        loop_counter++;

        LOG_DBG("Pitch PID output: %d", pitch_output);

        // TODO: call motor driver API
        set_motor_speed(speed_output);
        // set_motor_speed(motor_a, speed_a)
        // set_motor_speed(motor_b, speed_b)
    }

    // Shut down motors if pitch angle is out of bounds (fail safe)
    else
    {
        PID_pitch.set_point = CONFIG_STATIC_SET_POINT_PITCH + ANGLE_NORMALIZED_CONSTANT;

        // TODO: Set motor pwm to 0
        set_motor_speed(0);
    }
    first = false;
}


//========================================================================================
/*                                                                                      *
 *                             Setup and other utils                                    *
 *                                                                                      */
//========================================================================================

int setup()
{
    float current_time = k_uptime_get();
    pid_time.prev_pitch_time_ms = current_time;
    pid_time.prev_speed_time_ms = current_time;
    init_motors();
}

//========================================================================================
/*                                                                                      *
 *                                    Event handlers                                    *
 *                                                                                      */
//========================================================================================

static bool event_handler(const struct app_event_header *eh)
{
    struct controller_msg_data msg = {0};
    bool enqueue_msg = false;

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
            // If we get a new message, update the controller outputs
            // TODO: Get IMU data from message.
            // MPU9250 has an onboard DMP, but I couldn't find it in
            // the driver. Implementing use of the DMP is probably a fun challenge :)
            // TODO: update_controller(horizontal_speed, angle, accel[2])
        } 
    }

}

K_THREAD_DEFINE(controller_module_thread, CONFIG_CONTROLLER_THREAD_STACK_SIZE,
                module_thread_fn, NULL, NULL, NULL,
                K_HIGHEST_APPLICATION_THREAD_PRIO, 0, 0);

APP_EVENT_LISTENER(MODULE, event_handler);
APP_EVENT_SUBSCRIBE(MODULE, imu_module_event);
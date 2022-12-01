#pragma once

typedef struct
{
    float set_point;
    float error;
    float last_error;

    float kp;
    float ki;
    float kd;

    float i_lb; // Lower bound integrator
    float i_ub; // Upper bound integrator

    float out_P;
    float out_I;
    float out_D;
} PID_t;

/*  @brief Function to perform an iteration for a PID controller instance.
 *
 *  @detail PID controller is implemented in parallel
 *
 *  @param[in] PID_t *PID   Pointer to PID controller instance
 *  @param[in] float measurement    The measured value that is compared to the PID's set-point
 *  @param[in] float dt Delta time [s], ie time since last iteration, to be used in calculation of derivative part
 *
 *  @return Controller output from PID controller
 */
float update_PID(PID_t *PID, float measurement, float dt);

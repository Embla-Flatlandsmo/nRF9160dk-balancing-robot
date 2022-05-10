#ifndef _PID_UTIL_
#define _PID_UTIL_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr>
#include "pid.h"

#include "pid.h"

float update_PID(PID_t *PID, float measurement, float dt)
{
    float controller_output = 0;

    PID->error = PID->set_point - measurement;

    // Compute P
    PID->out_P = PID->kp * PID->error;

    // Compute I
    PID->out_I += PID->ki * PID->error * dt;

    if (PID->out_I > PID->i_ub)
        PID->out_I = PID->i_ub;
    else if (PID->out_I < PID->i_lb)
        PID->out_I = PID->i_lb;

    // Compute D
    PID->out_D = PID->kd * (PID->error - PID->last_error) / dt;

    controller_output = PID->out_P + PID->out_I + PID->out_D;

    PID->last_error = PID->error;

    return controller_output;
}

#endif /* _PID_UTIL_ */
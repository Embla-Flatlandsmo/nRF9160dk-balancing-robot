#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "eulerconversion.h"

float qToFloat(long number, unsigned char q)
{
    unsigned long mask;
    for (int i = 0; i < q; i++)
    {
        mask |= (1 << i);
    }
    return (number >> q) + ((number & mask) / (float)(2 << (q - 1)));
}

void compute_euler(long *quat, float *rpy)
{
    volatile float roll, pitch, yaw;

    long qw = quat[0];
    long qx = quat[1];
    long qy = quat[2];
    long qz = quat[3];

    float dqw = qToFloat(qw, 30);
    float dqx = qToFloat(qx, 30);
    float dqy = qToFloat(qy, 30);
    float dqz = qToFloat(qz, 30);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = +2.0f * (dqw * dqx + dqy * dqz);
    float t1 = +1.0f - 2.0f * (dqx * dqx + ysqr);
    roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0f * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    pitch = asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0f * (dqw * dqz + dqx * dqy);
    float t4 = +1.0f - 2.0f * (ysqr + dqz * dqz);
    yaw = atan2(t3, t4);

    pitch *= (180.0f / M_PI);
    roll *= (180.0f / M_PI);
    yaw *= (180.0f / M_PI);
    if (pitch < 0)
        pitch = 360.0f + pitch;
    if (roll < 0)
        roll = 360.0f + roll;
    if (yaw < 0)
        yaw = 360.0f + yaw;

    rpy[0] = pitch; // Roll and pitch swapped in order to coincide with the PCA63539 motor shield.
    rpy[1] = roll;
    rpy[2] = yaw;
}

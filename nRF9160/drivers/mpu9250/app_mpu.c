#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_mpu, 3);

#include "inv_mpu.h"
#include "inv_mpu_dmp.h"

#include "app_mpu.h"
#include "MPU9250_RegisterMap.h"
#include "eulerconversion.h"


bool app_mpu_who_am_i(void)
{
    uint16_t try_count = 0;
    uint8_t tmp_u8;

    do
    {
        if ( (try_count) < 100 )
        {
            ++try_count;
        }
        else
        {
            LOG_INF("[FAIL] MPU not found. Cancelling initiation. ");
            return false;
        }

    } while ( (mpu_read_reg(0x75, &tmp_u8) != 0)
    ||        (tmp_u8 != 0x71) );


    LOG_INF("[SUCCESS] MPU found at try %d", try_count);
    return true;

}

unsigned short app_mpu_fifo_available(void)
{
    unsigned char fifoH, fifoL;

	if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
		return 0;
	if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
		return 0;

	return (fifoH << 8 ) | fifoL;
}

inv_error_t app_dmp_enable_features(unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP;
	return dmp_enable_feature(enMask);
}

void app_dmp_load(void)
{
    unsigned short feat = FEATURES;

    if(dmp_load_motion_driver_firmware() == INV_SUCCESS)
    {
        LOG_INF("[SUCCESS] DMP firmware loaded. ");
    }
    else
    {
        LOG_INF("[FAIL] DMP firmware failed to load. ");
        return;
    }

    if (feat & DMP_FEATURE_LP_QUAT)
    {
        feat &= ~(DMP_FEATURE_6X_LP_QUAT);
        dmp_enable_lp_quat(1);
    }
    else if (feat & DMP_FEATURE_6X_LP_QUAT)
        dmp_enable_6x_lp_quat(1);

    if (feat & DMP_FEATURE_GYRO_CAL)
        dmp_enable_gyro_cal(1);

    if (app_dmp_enable_features(feat) != INV_SUCCESS)
    {
        LOG_INF("[FAIL] DMP failed to be configured.");
    }
    else
    {
        dmp_set_fifo_rate(DMP_FIFO_RATE);
        mpu_set_dmp_state(1);
        LOG_INF("[SUCCESS] DMP configured. ");
    }
}

void app_mpu_init(void)
{
    uint8_t result;
    struct int_param_s int_param;

    k_sleep(K_MSEC(500));

    if(!app_mpu_who_am_i()) //If MPU is not found, the initiation is cancelled.
        return;

    result = mpu_init(&int_param);
    if(!result)
    {
        LOG_INF("[SUCCESS] MPU initiated with default values. ");
    }
    else
    {
        LOG_INF("[FAIL] Failed to initiate MPU properly. ");
    }

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    mpu_set_bypass(1);


	#if ENABLE_DMP
		app_dmp_load();
	#endif
}

int app_mpu_get_angles(float *measurement, float *accel)
{
    static FIFO_t FIFO;
    if(app_mpu_fifo_available())
    {
        app_mpu_dmp_update_fifo(&FIFO);
        compute_euler(FIFO.quat, FIFO.rpy);


        /** Test site for filtering z-axis data and estimating position **/
       /* short filtered_measurement;
        static short prev_avg;
        static short position;

        filtered_measurement = app_mpu_exp_moving_avg(FIFO.accel[2], prev_avg, 0.12);
        position += filtered_measurement / 1000;
        prev_avg = filtered_measurement;
        //LOG_INF("%d %d\r", FIFO.accel[2], filtered_measurement);
        LOG_INF("%d\r", position);
        */
        /** Test site closed **/



        memcpy(measurement, FIFO.rpy, 12);
        memcpy(accel, FIFO.accel, 12);
        // #if PRINT_ANGLE_VALUES
            LOG_INF("Angles: roll %f - pitch %f", (FIFO.rpy[0]), (FIFO.rpy[1]));
						LOG_INF(" - yaw %f ", (FIFO.rpy[2]));
        // #endif

        #if PRINT_PITCH_ANGLE
            LOG_INF("pitch %f  ", (FIFO.rpy[1]));
        #endif

        #if PRINT_ACCELEROMETER_DATA
            LOG_INF("Acceleration: X: %7d \t Y: %7d \t Z: %7d ", FIFO.accel[0], FIFO.accel[1], FIFO.accel[2]);
        #endif

        #if PRINT_ACCELEROMETER_GRAPH_DATA
            LOG_INF("%d %d %d \r", FIFO.accel[0], FIFO.accel[1], FIFO.accel[2]);
        #endif

        return 0;

    }
    else
    {
        LOG_ERR("FIFO not available");

        return 1;
    }
}

void app_mpu_dmp_update_fifo(FIFO_t* FIFO)
{
    dmp_read_fifo(FIFO->gyro, FIFO->accel, FIFO->quat, &FIFO->timestamp, &FIFO->sensors, &FIFO->more);
}

float app_mpu_exp_moving_avg(short new_measurement, short prev_avg, float alpha)
{
    return (short)(alpha * (float)new_measurement + (1 - alpha) * (float)prev_avg);
}

void app_mpu_delay_ms(uint32_t ms) {
    k_sleep(K_MSEC(ms));
}

void app_mpu_get_ms(unsigned long *count)
{
    *count = k_uptime_get_32();
}


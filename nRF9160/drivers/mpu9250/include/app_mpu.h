#include <stdbool.h>
#include <stdint.h>
#include <string.h>


typedef int inv_error_t;
#define INV_SUCCESS                 0
#define INV_ERROR                   0x20

#define MPU_ADDRESS_PIN                24 // MISO in schematic

#define DMP_FIFO_RATE               200 // Hz

#define DMP_PITCH_ANGLE_OFFSET      1

#define FEATURES DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL

#define ENABLE_DMP  1


typedef struct
{
    short gyro[3];
    short accel[3];
    long quat[4];
    float rpy[3];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
} FIFO_t;

// Function to detect MPU

bool app_mpu_who_am_i(void);

// Initate MPU with default values

void app_mpu_init(void);

// Functions used for obtaining correct angle
int app_mpu_get_angles(float *measurement, float *accel);
void app_mpu_dmp_update_fifo(FIFO_t* FIFO);
float app_mpu_exp_moving_avg(short new_measurement, short prev_avg, float alpha);
void app_mpu_get_ms(unsigned long *count);
void app_mpu_delay_ms(uint32_t ms);
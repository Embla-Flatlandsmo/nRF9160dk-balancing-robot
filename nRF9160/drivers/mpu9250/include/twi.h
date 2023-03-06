#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MPU_SCL_PIN 25
#define MPU_SDA_PIN 23

#define QUAD_DECODER_SCL_PIN 27
#define QUAD_DECODER_SDA_PIN 26

typedef enum
{
	TWI_INSTANCE_MPU,
	TWI_INSTANCE_DECODER
} twi_instance_type_t;

// Prototypes used in twi.c

// Initializes the twi driver
int twi_init(void);

// Writes data on the twi-channel, returns 0 if successful

uint8_t twi_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);
uint8_t twi_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);

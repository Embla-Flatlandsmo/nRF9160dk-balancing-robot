#include <devicetree.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(twi, 3);

#include "twi.h"

static struct device *i2c_dev;

int imu_twi_init(const struct device *unused)
{
	i2c_dev = (struct device *)device_get_binding("I2C_2");
	if (!i2c_dev) {
		LOG_ERR("No binding to I2C");
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C dev not ready");
		return -ENODEV;
	}

	return 0;
}

uint8_t twi_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
	return i2c_burst_write(i2c_dev,  slave_addr, reg_addr, data, length);
}

uint8_t twi_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
	return i2c_burst_read(i2c_dev, slave_addr, reg_addr, data, length);
}


// SYS_INIT(imu_twi_init, APPLICATION, 90);

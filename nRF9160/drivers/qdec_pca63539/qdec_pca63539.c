
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_pca63539_qdec, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT nordic_pca63539_qdec

static struct driver_data
{
    struct sensor_value motor_a_pos;
    struct sensor_value motor_b_pos;
};

static struct driver_conf
{
    struct i2c_dt_spec i2c;
};

static int sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    if (!(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION))
    {
        LOG_ERR("Invalid channel %d. Only SENSOR_CHAN_ALL and SENSOR_CHAN_ROTATION are supported.", chan);
        return -ENOTSUP;
    }

    struct driver_data *const data = dev->data;
    const struct driver_conf *const conf = dev->config;

    size_t data_size = sizeof(struct sensor_value) * 2;
    struct sensor_value buf[2] = {0};
    if (i2c_read_dt(&conf->i2c, (uint8_t *)buf, data_size))
    {
        LOG_ERR("Failed to read data from bus %s address 0x%2x", conf->i2c.bus->name, conf->i2c.addr);
        return -EIO;
    }
    data->motor_a_pos = buf[0];
    data->motor_b_pos = buf[1];
    LOG_DBG("Fetched Motor a: %d.%d, Motor b: %d.%d", buf[0].val1, buf[0].val2, buf[1].val1, buf[1].val2);

    return 0;
}

/**
 * @brief Gets data from requested sensor channel.
 * Available data:
 *  - SENSOR_CHAN_ROTATION: Motor A position, Motor B position
 *
 * @param dev Device to get data from
 * @param chan Sensor channel to get data from. Only SENSOR_CHAN_ROTATION is supported.
 * @param val Pointer to array of at least 2 sensor value structs.
 * @return 0 on success, negative errno code otherwise.
 */
static int channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    if (chan != SENSOR_CHAN_ROTATION)
    {
        LOG_ERR("Invalid channel %d. Only SENSOR_CHAN_ROTATION is supported.", chan);
        return -ENOTSUP;
    }

    struct driver_data *const data = dev->data;
    const struct driver_conf *const conf = dev->config;

    val[0] = data->motor_a_pos;
    val[1] = data->motor_b_pos;

    return 0;
}

/**
 * @brief Not supported.
 *
 * @param dev Unused
 * @param trig Unused
 * @param handler Unused
 * @return -ENOTSUP
 */
static int trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
    LOG_ERR("Trigger set not supported on %s", dev->name);
    ARG_UNUSED(trig);
    ARG_UNUSED(handler);
    return -ENOTSUP;
}

static const struct sensor_driver_api driver_api = {
    .sample_fetch = sample_fetch,
    .channel_get = channel_get,
    .trigger_set = trigger_set,
};

static int init(const struct device *dev)
{
    struct driver_data *const data = dev->data;
    const struct driver_conf *const conf = dev->config;

    if (!device_is_ready(conf->i2c.bus))
    {
        LOG_ERR("Failed to initialize %s: I2C bus %s not ready", dev->name, conf->i2c.bus->name);
        return -ENODEV;
    }
    return 0;
}

#define INIT_PCA63539_QDEC_DRIVER(inst)             \
    static struct driver_data data_##inst = {       \
        .motor_a_pos = {0},                         \
        .motor_b_pos = {0},                         \
    };                                              \
    static const struct driver_conf conf_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst),          \
    };                                              \
    DEVICE_DT_INST_DEFINE(inst, init, NULL, &data_##inst, &conf_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_PCA63539_QDEC_DRIVER);
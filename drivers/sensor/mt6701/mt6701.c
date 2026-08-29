/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT mag_mt6701

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MT6701_ANGLE_REG 0x03
#define MT6701_ANGLE_BITS 14
#define MT6701_ANGLE_MAX (1U << MT6701_ANGLE_BITS)
#define MT6701_ANGLE_HALF (MT6701_ANGLE_MAX / 2)
#define MT6701_MICRODEG_PER_ROTATION 360000000LL
#define MICRODEG_PER_DEGREE 1000000LL

struct mt6701_config {
    struct i2c_dt_spec i2c;
    uint16_t poll_period_ms;
};

struct mt6701_data {
    const struct sensor_trigger *trigger;
    sensor_trigger_handler_t handler;
    const struct device *dev;

    struct k_work_delayable work;
    uint16_t poll_period_ms;

    bool initialized;
    uint16_t prev_angle;
    int32_t accumulated_steps;
};

static void mt6701_poll_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct mt6701_data *data = CONTAINER_OF(dwork, struct mt6701_data, work);

    int ret = k_work_reschedule(dwork, K_MSEC(data->poll_period_ms));
    if (ret < 0) {
        LOG_WRN("Failed to reschedule MT6701 poll: %d", ret);
    }

    if (data->handler) {
        data->handler(data->dev, data->trigger);
    }
}

static int mt6701_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                              sensor_trigger_handler_t handler) {
    struct mt6701_data *data = dev->data;
    const struct mt6701_config *cfg = dev->config;

    data->trigger = trig;
    data->handler = handler;

    int ret = k_work_schedule(&data->work, K_MSEC(cfg->poll_period_ms));
    if (ret < 0) {
        LOG_WRN("Failed to schedule MT6701 poll: %d", ret);
        return -EIO;
    }

    return 0;
}

static int mt6701_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    const struct mt6701_config *cfg = dev->config;
    struct mt6701_data *data = dev->data;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    uint8_t buf[2];
    int ret = i2c_burst_read_dt(&cfg->i2c, MT6701_ANGLE_REG, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read MT6701 angle register: %d", ret);
        return ret;
    }

    uint16_t raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);
    raw &= MT6701_ANGLE_MAX - 1;

    if (!data->initialized) {
        data->initialized = true;
        data->prev_angle = raw;
        return 0;
    }

    int16_t delta = raw - data->prev_angle;
    if (delta > MT6701_ANGLE_HALF) {
        delta -= MT6701_ANGLE_MAX;
    } else if (delta < -MT6701_ANGLE_HALF) {
        delta += MT6701_ANGLE_MAX;
    }

    data->accumulated_steps += delta;
    data->prev_angle = raw;

    return 0;
}

static int mt6701_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val) {
    struct mt6701_data *data = dev->data;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    int64_t microdegrees =
        (int64_t)data->accumulated_steps * MT6701_MICRODEG_PER_ROTATION / MT6701_ANGLE_MAX;

    val->val1 = microdegrees / MICRODEG_PER_DEGREE;
    val->val2 = microdegrees % MICRODEG_PER_DEGREE;

    data->accumulated_steps = 0;

    return 0;
}

static const struct sensor_driver_api mt6701_driver_api = {
    .trigger_set = mt6701_trigger_set,
    .sample_fetch = mt6701_sample_fetch,
    .channel_get = mt6701_channel_get,
};

static int mt6701_init(const struct device *dev) {
    const struct mt6701_config *cfg = dev->config;
    struct mt6701_data *data = dev->data;

    data->dev = dev;
    data->poll_period_ms = cfg->poll_period_ms;

    k_work_init_delayable(&data->work, mt6701_poll_work_cb);

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_WRN("I2C bus not ready at MT6701 init; polling will start as soon as it is");
    }

    return 0;
}

#define MT6701_INST(n)                                                                             \
    static struct mt6701_data mt6701_data_##n;                                                     \
    static const struct mt6701_config mt6701_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, mt6701_init, NULL, &mt6701_data_##n, &mt6701_config_##n, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &mt6701_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MT6701_INST)
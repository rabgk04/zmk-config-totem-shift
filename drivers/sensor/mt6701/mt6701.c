/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT mag_mt6701

#include <errno.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MT6701_ANGLE_REG 0x03
#define MT6701_ANGLE_BITS 14
/* Deliberately signed: these are compared against a signed delta below. As unsigned
 * they would drag the delta through the usual arithmetic conversions and make the
 * wrap correction fire on every value, including zero. */
#define MT6701_ANGLE_MAX (1 << MT6701_ANGLE_BITS)
#define MT6701_ANGLE_HALF (MT6701_ANGLE_MAX / 2)
#define MT6701_MICRODEG_PER_ROTATION 360000000LL
#define MICRODEG_PER_DEGREE 1000000LL

struct mt6701_config {
    struct i2c_dt_spec i2c;
    uint16_t poll_period_ms;
    uint16_t max_backlog_degrees;
    uint16_t min_report_interval_ms;
};

struct mt6701_data {
    const struct sensor_trigger *trigger;
    sensor_trigger_handler_t handler;
    const struct device *dev;

    struct k_work_delayable work;
    struct k_spinlock lock;
    uint16_t poll_period_ms;

    bool initialized;
    uint16_t prev_angle;
    int64_t last_report_time;
    /* Rotation not yet handed to the keymap, in microdegrees. Sub-degree motion stays
     * here between polls instead of being reported, so channel_get can always return
     * whole degrees in val1 with an empty val2. */
    int64_t pending_microdeg;
};

/* Reads the angle register and folds the movement since the last read into
 * pending_microdeg. Returns true once at least a whole degree has piled up. */
static bool mt6701_accumulate(const struct device *dev) {
    const struct mt6701_config *cfg = dev->config;
    struct mt6701_data *data = dev->data;

    uint8_t buf[2];
    int ret = i2c_burst_read_dt(&cfg->i2c, MT6701_ANGLE_REG, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read MT6701 angle register: %d", ret);
        return false;
    }

    uint16_t raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);
    raw &= MT6701_ANGLE_MAX - 1;

    if (!data->initialized) {
        data->initialized = true;
        data->prev_angle = raw;
        return false;
    }

    int32_t delta = (int32_t)raw - (int32_t)data->prev_angle;
    if (delta > MT6701_ANGLE_HALF) {
        delta -= MT6701_ANGLE_MAX;
    } else if (delta < -MT6701_ANGLE_HALF) {
        delta += MT6701_ANGLE_MAX;
    }
    data->prev_angle = raw;

    if (delta == 0) {
        return false;
    }

    int64_t microdeg = (int64_t)delta * MT6701_MICRODEG_PER_ROTATION / MT6701_ANGLE_MAX;
    int64_t now = k_uptime_get();

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->pending_microdeg += microdeg;

    /* Rate limiting, not smoothing. zmk,behavior-sensor-rotate turns every trigger into a
     * press/release pair on the shared behavior queue, which holds
     * CONFIG_ZMK_BEHAVIORS_QUEUE_SIZE entries and drains only one trigger per tap-ms. On
     * overflow k_msgq_put drops the item and the sensor-rotate behavior ignores the error.
     * Losing a release leaves &msc's speed accumulator non-zero, so its tick work reschedules
     * itself forever and the wheel scrolls until the board is reset. A detent-free magnetic
     * knob spins well past that drain rate, so cap both the burst size and how often we hand
     * work to the keymap. Fast spins saturate the scroll rate instead of latching it on. */
    int64_t max_backlog = (int64_t)cfg->max_backlog_degrees * MICRODEG_PER_DEGREE;
    data->pending_microdeg = CLAMP(data->pending_microdeg, -max_backlog, max_backlog);

    bool ready = llabs(data->pending_microdeg) >= MICRODEG_PER_DEGREE &&
                 (now - data->last_report_time) >= cfg->min_report_interval_ms;
    if (ready) {
        data->last_report_time = now;
    }
    k_spin_unlock(&data->lock, key);

    LOG_DBG("MT6701 raw=%u delta=%d pending_udeg=%lld", raw, delta,
            (long long)data->pending_microdeg);

    return ready;
}

static void mt6701_poll_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mt6701_data *data = CONTAINER_OF(dwork, struct mt6701_data, work);

    int ret = k_work_reschedule(dwork, K_MSEC(data->poll_period_ms));
    if (ret < 0) {
        LOG_WRN("Failed to reschedule MT6701 poll: %d", ret);
    }

    if (mt6701_accumulate(data->dev) && data->handler) {
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
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    /* The poll work already sampled the sensor; nothing to do here. */
    return 0;
}

static int mt6701_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val) {
    struct mt6701_data *data = dev->data;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    int32_t degrees = (int32_t)(data->pending_microdeg / MICRODEG_PER_DEGREE);
    data->pending_microdeg -= (int64_t)degrees * MICRODEG_PER_DEGREE;
    k_spin_unlock(&data->lock, key);

    /* zmk,behavior-sensor-rotate treats a zero val1 as the legacy "val2 holds a raw
     * trigger count" encoding, so the fractional degree must never leak into val2. */
    val->val1 = degrees;
    val->val2 = 0;

    LOG_DBG("MT6701 reporting %d degrees", degrees);

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
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                                 .max_backlog_degrees = DT_INST_PROP(n, max_backlog_degrees),                                       .min_report_interval_ms = DT_INST_PROP(n, min_report_interval_ms),                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, mt6701_init, NULL, &mt6701_data_##n, &mt6701_config_##n, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &mt6701_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MT6701_INST)

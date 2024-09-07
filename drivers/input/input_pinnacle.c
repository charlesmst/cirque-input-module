#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#if CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0
#include <zephyr/sys/util.h> // for CLAMP
#endif

#include <zephyr/logging/log.h>

#include "math.h"
#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);


#ifdef CONFIG_INPUT_PINNACLE_MACCEL
static int64_t maccel_timer;
static maccel_config_t g_maccel_config = {
    // clang-format off
    .growth_rate =  CONFIG_INPUT_PINNACLE_MACCEL_GROWTH_RATE / 100.0f,
    .offset =       CONFIG_INPUT_PINNACLE_MACCEL_OFFSET / 100.0f,
    .limit =        CONFIG_INPUT_PINNACLE_MACCEL_LIMIT / 100.0f,
    .takeoff =      CONFIG_INPUT_PINNACLE_MACCEL_TAKEOFF / 100.0f,
    .enabled =      true
    // clang-format on
};
#endif
static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
#if DT_INST_ON_BUS(0, spi)
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = 3,
        },
        {
            .buf = &tx_buffer[3],
            .len = len,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 2,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus, &tx, &rx);

    return ret;
#elif DT_INST_ON_BUS(0, i2c)
    return i2c_burst_read_dt(&config->bus, PINNACLE_READ | addr, buf, len);
#endif
}

static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
#if DT_INST_ON_BUS(0, spi)
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus, &tx, &rx);

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d", rx_buffer[1]);
        return -EIO;
    }

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }
    return ret;
#elif DT_INST_ON_BUS(0, i2c)
    return i2c_reg_write_byte_dt(&config->bus, PINNACLE_WRITE | addr, val);
#endif
}

static void set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

#if 0
static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}
#endif

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static void pinnacle_report_data(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    uint8_t packet[4];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }
    if (!(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 4);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }
    struct pinnacle_data *data = dev->data;
    uint8_t btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);
    int16_t dx = (int16_t)(int8_t)packet[1];
    int16_t dy = (int16_t)(int8_t)packet[2];
    int8_t dv = (int8_t)packet[3];
    LOG_DBG("button: %d, dx: %d dy: %d dv: %d", btn, dx, dy, dv);
    if (data->in_int) {
        LOG_DBG("Clearing status bit");
        ret = pinnacle_clear_status(dev);
        data->in_int = true;
    }

#if CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0
    bool has_key_report = false;
#endif

    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
                #if CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0
                    has_key_report = true;
                #endif
            }
        }
    }

    data->btn_cache = btn;

#if CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0

    static int64_t adx = 0;
    static int64_t ady = 0;
    static int64_t adv = 0;
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
    if (now - last_smp_time >= CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN) {
        adx = ady = 0;
    }
    last_smp_time = now;
    adx += dx;
    ady += dy;
    adv += dv;
    if (now - last_rpt_time < CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN) {
        // force report on tapped and scrolled
        if (!has_key_report && !adv) {
            return;
        }
    }

    // clamp report value
    int16_t rx = (int16_t)CLAMP(adx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(ady, INT16_MIN, INT16_MAX);
    int16_t rv = (int16_t)CLAMP(adv, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;
    bool have_v = rv != 0;

    if (have_x || have_y || have_v) {
        last_rpt_time = now;
        adx = ady = adv = 0;
        if (have_x) {
            input_report_rel(dev, INPUT_REL_X, rx, !have_y && !have_v, K_NO_WAIT);
        }
        if (have_y) {
            input_report_rel(dev, INPUT_REL_Y, ry, !have_v, K_NO_WAIT);
        }
        if (have_v) {
            input_report_rel(dev, INPUT_REL_WHEEL, rv, true, K_NO_WAIT);
        }
    }
    else if (has_key_report) {
        // force sync key report
        input_report_rel(dev, INPUT_REL_X, 0, true, K_NO_WAIT);
    }

#else /* CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0 */

    // either delta xy, or scroll wheel should be read
    if (dv) {
        input_report_rel(dev, INPUT_REL_WHEEL, dv, true, K_FOREVER);
    }
    else {

#ifdef CONFIG_INPUT_PINNACLE_MACCEL
            static float rounding_carry_x = 0;
            static float rounding_carry_y = 0;
            int16_t x = dx;
            int16_t y = dy;

            const int64_t delta_time = k_uptime_delta(&maccel_timer);
            if (delta_time > CONFIG_INPUT_PINNACLE_MACCEL_ROUNDING_CARRY_TIMEOUT_MS) {
                rounding_carry_x = 0;
                rounding_carry_y = 0;
            }
            maccel_timer = k_uptime_get();
            uint32_t device_cpi = 200;
            const float dpi_correction = (float)1000.0f / device_cpi;
            // calculate euclidean distance moved (sqrt(x^2 + y^2))
            const float distance = sqrtf(x * x + y * y);
            // calculate delta velocity: dv = distance/dt
            const float velocity_raw = distance / delta_time;
            // correct raw velocity for dpi
            const float velocity = dpi_correction * velocity_raw;
            // letter variables for readability of maths:
            const float k = g_maccel_config.takeoff;
            const float g = g_maccel_config.growth_rate;
            const float s = g_maccel_config.offset;
            const float m = g_maccel_config.limit;
            // acceleration factor: f(v) = 1 - (1 - M) / {1 + e^[K(v - S)]}^(G/K):
            // Generalised Sigmoid Function, see https://www.desmos.com/calculator/k9vr0y2gev
            const float upper_limit = CONFIG_INPUT_PINNACLE_MACCEL_LIMIT_UPPER / 100.0f;
            const float maccel_factor = upper_limit - (upper_limit - m) / powf(1 + expf(k * (velocity - s)), g / k);
            // multiply mouse reports by acceleration factor, and account for previous quantization errors:
            const float new_x = rounding_carry_x + maccel_factor * x;
            const float new_y = rounding_carry_y + maccel_factor * y;
            // Accumulate any difference from next integer (quantization).
            rounding_carry_x = new_x - (int)new_x;
            rounding_carry_y = new_y - (int)new_y;
            // Clamp values and report back accelerated values.
            dx = (int16_t)CLAMP(new_x, INT16_MIN, INT16_MAX);
            dy = (int16_t)CLAMP(new_y, INT16_MIN, INT16_MAX);
#endif
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
    }

#endif /* CONFIG_INPUT_PINNACLE_REPORT_INTERVAL_MIN > 0 */

    return;
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    pinnacle_report_data(data->dev);
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);
    data->in_int = true;
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;

    LOG_WRN("pinnacle start");
    data->in_int = false;
    int ret;
    k_msleep(4);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, 0x05); // No Z-Idle packets
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    if (config->sleep_en) {
        ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_EN_SLEEP);
        if (ret < 0) {
            LOG_ERR("can't write %d", ret);
            return ret;
        }
    }

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG,
                             pinnacle_adc_sensitivity_reg_value(config->sensitivity));
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    uint8_t feed_cfg3 = 0x00;
    if (config->no_smoothing) {
        feed_cfg3 |= PINNACLE_FEED_CFG3_DIS_SMO;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG3, feed_cfg3);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM;
    if (config->no_taps) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_TAP;
    }
    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN_FEED;
    if (feed_cfg1) {
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        .bus = COND_CODE_1(                                                                        \
            DT_INST_ON_BUS(0, i2c), (I2C_DT_SPEC_INST_GET(0)),                                     \
            (SPI_DT_SPEC_INST_GET(0,                                                               \
                                  SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE |        \
                                      SPI_TRANSFER_MSB | SPI_MODE_CPHA,                            \
                                  0))),                                                            \
        .rotate_90 = DT_INST_PROP(0, rotate_90),                                                   \
        .sleep_en = DT_INST_PROP(0, sleep),                                                        \
        .no_taps = DT_INST_PROP(0, no_taps),                                                       \
        .no_smoothing = DT_INST_PROP(0, no_smoothing),                                             \
        .sensitivity = DT_INST_ENUM_IDX_OR(0, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), dr_gpios, {}),                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, NULL, &pinnacle_data_##n, &pinnacle_config_##n,        \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)

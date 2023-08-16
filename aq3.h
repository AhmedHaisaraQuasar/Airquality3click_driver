/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT mikroelektronika_aq3

#include <zephyr/kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/init.h>
#include <stdio.h>
#include <zephyr/net/buf.h>

static const struct gpio_dt_spec aq3_reset = GPIO_DT_SPEC_INST_GET(
		0, aq3_reset_gpios);
static const struct gpio_dt_spec aq3_wk = GPIO_DT_SPEC_INST_GET(
		0, aq3_wk_gpios);

/* AQ3 register addresses */
#define AIRQUALITY3_DEVICE_SLAVE_ADDRESS 0x5A

#define AIRQUALITY3_I2C_DEV_NAME		DEVICE_DT_NAME(DT_INST_BUS(0))

/* AQ3 command & constant */
#define AIRQUALITY3_APP_START       0xF4
#define AIRQUALITY3_REG_STATUS      0x00
#define AIRQUALITY3_REG_ALG_DATA    0x02
#define AIRQUALITY3_POWER_STATE_ON  1
#define AIRQUALITY3_POWER_STATE_OFF 0
#define AIRQUALITY3_REG_MEAS_MODE   0x01
#define AIRQUALITY3_DRIVE_MODE_4    ( 0x04 << 4 )
#define AIRQUALITY3_INT_DATARDY_1   ( 0x01 << 3 )

struct airquality3_data {
    uint16_t co2;
    uint16_t tvoc;
    uint16_t raw_data;
    uint8_t m_status;
};
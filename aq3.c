/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT mikroelektronika_aq3

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include "aq3.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mikroelektronika_aq3, CONFIG_MIPOT_LOG_LEVEL);

static int airquality3_generic_write ( const struct device *dev, uint8_t reg, uint8_t *data_buf, uint32_t len )
{
    uint8_t tx_buf[ 256 ];
    int8_t ret;
	uint8_t cnt;
    
    tx_buf[ 0 ] = reg;
    
    for ( cnt = 1; cnt <= len; cnt++ )
    {
        tx_buf[ cnt ] = data_buf[ cnt - 1 ]; 
    }
    
    ret = i2c_write(dev ,tx_buf ,len + 1 ,AIRQUALITY3_DEVICE_SLAVE_ADDRESS ); 
	if (ret){
        printk("Failed to write AQ3 data: %d\n", ret);
        return ret;
    } 
	return 0; 
}

static int airquality3_generic_read ( const struct device *dev, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
	int8_t ret;
	ret = i2c_write_read(dev,AIRQUALITY3_DEVICE_SLAVE_ADDRESS,&reg,1,data_buf,len);
	if (ret){
        printk("Failed to write/read AQ3 data: %d\n", ret);
        return ret;
    }
}

uint8_t airquality3_get_status ( const struct device *dev )
{
    uint8_t rx_data;

    airquality3_generic_read( dev, AIRQUALITY3_REG_STATUS, &rx_data, 1 );
    k_sleep(K_MSEC(100));

    return rx_data;
}

static int airquality3_set_measurement_mode ( const struct device *dev, uint8_t mode )
{
   int ret = airquality3_generic_write( dev, AIRQUALITY3_REG_MEAS_MODE, &mode, 1 );
   if (ret){
        printk("Failed to set measurement mode AQ3 : %d\n", ret);
        return ret;
    }
	return 0; 
}

static int airquality3_init(const struct device *dev)
{
	int ret;
	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER ;
	
	/* I2C & GPIO initialization */ 

    if (!device_is_ready(dev)) {
		printk("I2C device is not ready\n");
		return -1;
	}

	if (i2c_configure(dev, i2c_cfg)) {
		printk("I2C config failed\n");
		return -1;
	}

	if (!gpio_is_ready_dt(&aq3_wk)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&aq3_wk, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

    if (!gpio_is_ready_dt(&aq3_reset)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&aq3_reset, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}
	
    printk("INITIALISATION Success\n");
	
    /* Wake-up click procedure */
	ret = gpio_pin_set_dt(&aq3_wk, AIRQUALITY3_POWER_STATE_ON);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_set_dt(&aq3_reset, AIRQUALITY3_POWER_STATE_ON);
	if (ret < 0) {
		return -1;
	}
	k_sleep(K_MSEC(2000));
	ret = gpio_pin_set_dt(&aq3_reset, AIRQUALITY3_POWER_STATE_OFF);
	if (ret < 0) {
		return -1;
	}
	k_sleep(K_MSEC(200));
	
	airquality3_generic_write(dev,AIRQUALITY3_APP_START,0,0);
    
    k_sleep(K_MSEC(1));
	
    airquality3_set_measurement_mode(dev,AIRQUALITY3_DRIVE_MODE_4 | AIRQUALITY3_INT_DATARDY_1);
    
    k_sleep(K_MSEC(500));

	printk("Wake up procedure success\nStart measurement\n");
	return 0;
}

static int airquality3_sample_fetch(const struct device *dev,enum sensor_channel chan)
{
	struct airquality3_data *drv_data = dev->data;
	uint8_t rx_buf[ 8 ];
    uint8_t drdy_f;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	drdy_f = airquality3_get_status( dev );
	if ( ( drdy_f & 0x08 ) != 0 )
    {
        int ret = airquality3_generic_read( dev, AIRQUALITY3_REG_ALG_DATA, rx_buf, 8 );
		if (ret){
        	printk("Failed to fecth AQ3 : %d\n", ret);
        	return ret;
    	}

        drv_data->co2 = rx_buf[ 0 ];   // CO2
        drv_data->co2 <<= 8;
        drv_data->co2 |= rx_buf[ 1 ];

        drv_data->tvoc = rx_buf[ 2 ];  // TVOC
        drv_data->tvoc <<= 8;
        drv_data->tvoc |= rx_buf[ 3 ];

        drv_data->m_status = rx_buf[ 4 ];  // status reg

        drv_data->raw_data = rx_buf[ 6 ];  // RAW DATA
        drv_data->raw_data <<= 8;
        drv_data->raw_data |= rx_buf[ 7 ];

        return rx_buf[ 5 ];  // error ID
    }
    return 0;
}

static int airquality3_channel_get(const struct device *dev,enum sensor_channel chan, struct sensor_value *val)
{
	struct airquality3_data *drv_data = dev->data;
	uint16_t tmp;

	if (chan == SENSOR_CHAN_CO2) {
		tmp = drv_data->co2;
		val->val1 = tmp;
		val->val2 = 0;
	} else if (chan == SENSOR_CHAN_VOC) {
		tmp = drv_data->tvoc;
		val->val1 = tmp;
		val->val2 = 0;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api airquality3_driver_api = {
	.sample_fetch = airquality3_sample_fetch,
	.channel_get = airquality3_channel_get,
};

#define AIRQUALITY3_DEFINE(inst)								\
	static struct airquality3_data airquality3_data_##inst;					\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, airquality3_init, NULL,				\
			      &airquality3_data_##inst, NULL, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &airquality3_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(AIRQUALITY3_DEFINE)
#include "imu.h"
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h> //for k_msleep
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>

#define LOG_MODULE_NAME imu
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct i2c_dt_spec dev_i2c_imu = I2C_DT_SPEC_GET(DT_NODELABEL(ism330is));


void initIMU(void)
{
    if (!device_is_ready(dev_i2c_imu.bus)) {
		LOG_INF("I2C bus %s is not ready!\n\r",dev_i2c_imu.bus->name);
		return;
	}

    uint8_t write_buf[2];
    uint8_t read_buf[1];

	
	LOG_INF("Get Thermistor");


    write_buf[0]=WHO_AM_I;
	int ret = i2c_write_read_dt(&dev_i2c_imu, write_buf, 1,read_buf, sizeof(read_buf));
    if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_imu.addr);
		return 0;
	}
    LOG_INF("WHO AM I response %02X", read_buf[0]);

    write_buf[0]=CTRL6_C;
    write_buf[1]=0x10; //low power mode Acc
    ret = i2c_write_dt(&dev_i2c_imu, write_buf, sizeof(write_buf));

    write_buf[0]=CTRL7_G;
    write_buf[1]=0x80; //low power mode Gyro
	ret = i2c_write_dt(&dev_i2c_imu, write_buf, sizeof(write_buf));
	
    write_buf[0]=CTRL1_XL;
    write_buf[1]=0x20; //26Hz, 2g, Acc
	ret = i2c_write_dt(&dev_i2c_imu, write_buf, sizeof(write_buf));

    write_buf[0]=CTRL2_G;
    write_buf[1]=0x20; //26Hz, +/-250dps, Gyro
	ret = i2c_write_dt(&dev_i2c_imu, write_buf, sizeof(write_buf));

    LOG_INF("IMU initialized for Accel and Gyro 26Hz, 2g, 250dps");
}

void getAccel(uint8_t *bytes)
{
    uint8_t write_buf[1];
    uint8_t read_buf[6];

    write_buf[0]=OUTX_L_A;

    int ret = i2c_write_read_dt(&dev_i2c_imu, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
    if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_imu.addr);
		return 0;
	}
    memcpy(bytes, read_buf, sizeof(read_buf));
}

void getGyro(uint8_t *bytes)
{
    uint8_t write_buf[1];
    uint8_t read_buf[6];

    write_buf[0]=OUTX_L_G;

    int ret = i2c_write_read_dt(&dev_i2c_imu, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
    if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_imu.addr);
		return 0;
	}
    memcpy(bytes, read_buf, sizeof(read_buf));
}
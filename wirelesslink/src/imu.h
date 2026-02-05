#ifndef _IMU_H
#define _IMU_H


#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#define FUNC_CFG_ACCESS 0x01
#define PIN_CTRL		0x02
#define DRDY_PULSED_REG 0x0B
#define INT1_CTRL		0x0D
#define INT2_CTRL		0x0E
#define WHO_AM_I		0x0F
#define CTRL1_XL		0x10
#define CTRL2_G			0x11
#define CTRL3_C			0x12
#define CTRL4_C			0x13
#define CTRL5_C			0x14
#define CTRL6_C			0x15
#define CTRL7_G			0x16
#define CTRL9_C			0x18
#define CTRL10_C		0x19
#define STATUS_REG		0x1E
#define	MD1_CFG			0x5E

#define ISPU_CONFIG		0x02
#define ISPU_STATUS		0x04
#define ISPU_MEM_SEL    0x08
#define ISPU_MEM_ADDR1  0x09
#define ISPU_MEM_ADDR0  0x0A
#define ISPU_MEM_DATA   0x0B
#define ISPU_DOUT_00_L  0x10

#define ISPU_INT1_CTRL0 0x50
#define ISPU_INT2_CTRL0 0x54
#define ISPU_ALGO0 		0x70

#define OUT_TEMP_L        	0x20
#define OUTX_L_G        	0x22
#define OUTX_L_A		0x28
#define WHO_AM_I		0x0F

void initIMU(void);
void getAccel(uint8_t *bytes);
void getGyro(uint8_t *bytes);
#endif
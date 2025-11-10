#ifndef _WLGPIO_H
#define _WLGPIO_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


void init_wl_gpio(void);
void enable3V3(bool enable);
void enableDCDC(bool enable);
void enableCoilDrive(bool enable);
void enableDisplay(bool enable);
void enableDebug0(bool enable);
void enableDebug1(bool enable);
void enableWiFi(bool enable);

uint8_t getChargingStatus(void);
void startCoilDrive(void);
void stopCoilDrive(void);
void configureRadioInterrupt(void);

#endif
#ifndef _CHARGER_H
#define _CHARGER_H


#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

void charger_thread(void);
void coil_reqresp_thread(void);

#define ROUTE_BLE   1
#define ROUTE_UART  2
#define ROUTE_USB   3
#define ROUTE_LOCAL 4


#define ROUTE_BIT_PMBOOT_WRITE   0x80
#define ROUTE_BIT_PMBOOT_READ    0x40
#define ROUTE_BIT_PMSCRIPT_WRITE 0x20
#define ROUTE_BIT_PMSCRIPT_READ  0x10
#define ROUTE_BIT_PMFILE_READ    0x08

extern struct k_msgq coil_resp_msgq;
extern struct k_msgq coil_req_msgq;

extern struct k_msgq charger_resp_msgq;

struct smartCoil_type {
   	uint8_t buf[64]; //must be divisible by 4
	uint8_t len;
	uint8_t source;
	uint8_t unused2;
	uint8_t unused3;
};


void OutputDisplay(bool clear, char* str1, char* str2, char* str3, char* str4, uint8_t len1, uint8_t len2, uint8_t len3, uint8_t len4);
void OutputDisplayLine(uint8_t line, char* str, uint8_t len);
void readRTC(uint8_t* rtc);
void writeRTC(uint8_t* rtc);

void startDCDC();
void stopDCDC();
void setDCDC(uint16_t voltage);
void keepCoilOn();
uint32_t setCoilPeriod(uint32_t period);
uint32_t getCoilPeriod(void);
uint16_t getThermistor(void);
uint8_t getCDPower(int16_t* current, uint16_t* voltage);
uint8_t getSysPower(int16_t* current, uint16_t* voltage);


bool logToFile(char * data, uint8_t len);
bool readFromFile(char * data, uint8_t len,  uint32_t off);
void initSD(void);
void dirSD(void);
uint32_t getLogLength(void);
bool flushLog(void);

uint8_t getChargeMode(void);
uint8_t setChargeMode(uint8_t mode);

void tuneCoil(void);
void metalDetect(void);
void charge(void);
void powerDownPM(void);

#define UPDATE_COIL   0
#define TURN_ON_COIL  1
#define TURN_OFF_COIL 2


void testNMT(void);
void testReadSDO(void);
void testWriteSDO(void);
uint8_t ReadSDO(uint8_t node, uint16_t index, uint8_t subindex, uint8_t numSubIndices, uint8_t* resp);
uint8_t WriteSDO(uint8_t node, uint16_t index, uint8_t subindex, uint8_t numSubIndices, uint8_t* writeData, uint8_t sizeBytes);
uint8_t NMT(uint8_t node, uint8_t cmd, uint8_t param1, uint8_t param2);
int8_t transmit(uint8_t node, uint8_t* data, uint8_t len, uint8_t counter, uint8_t protocol, uint8_t* resp);
uint8_t getChargerParams(uint8_t* settings);
void setChargerParams(uint8_t* settings);
uint8_t getAppRadioFromPMBoot(uint8_t* addrAP, uint8_t* addrPM, uint8_t* chan, uint8_t* power);


#endif


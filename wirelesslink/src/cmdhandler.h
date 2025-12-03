#ifndef _CMD_HANDLER_H
#define _CMD_HANDLER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define WL_IN_CHARGER false
#define HW_REV 1 //up to 65535
#define SW_REV 8 //up to 65535

#define PKT_HEADER_LEN 3
#define MIN_PKT_LEN PKT_HEADER_LEN // when checksum added, this will change
#define MAX_PKT_LEN  RX_BUF_SIZE
#define PKT_LEN_UNDEFINED 0 
#define MAX_MEDRADIO_PAYLOAD 60 //+ 64-length byte, address byte, 2 CRC/LQI bytes
#define MIN_MEDRADIO_PAYLOAD 0 //


//JML: does this need the __aligned(4) attribute
struct cmdHandler_type {
   	uint8_t buf[256]; //must be divisible by 4
	uint8_t len;
	uint8_t route;
	uint8_t unused2;
	uint8_t unused3;
};

extern struct k_msgq cmd_resp_msgq;

#define ROUTE_BLE   1
#define ROUTE_UART  2
#define ROUTE_USB   3
#define ROUTE_LOCAL 4
#define ROUTE_CHARGER 5

#define SOURCE_CMDHANDLER 1 //includes programmed button/accel actions + USB/BLE
#define SOURCE_CHARGER	  2
#define SOURCE_SENSOR	  3 //JML TODO:remove this eventually

struct action_type {
   	uint8_t buf[60]; 
	uint8_t len;
	uint8_t source;
};

// struct pmboot_header {
//    	uint32_t addr; 
// 	uint16_t pgsize;
// 	uint8_t sector;
// 	uint8_t unused;
// };

#define BUTTON_1_SHORT 0
#define BUTTON_2_SHORT 1
#define BUTTON_1_LONG  2
#define BUTTON_2_LONG  3
//#define ACCELEROMETER  4
#define MAX_ACTIONS    4
extern struct action_type action[MAX_ACTIONS];

#define RADIO_SETTINGS_ID 1
#define ACTION_SETTINGS_ID 2 
#define COIL_SETTINGS_ID   2 + MAX_ACTIONS


extern struct k_msgq cmd_req_msgq;

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define BLUE_LED_BLE_ADV         BIT0
#define GREEN_LED_RADIO_RESPONSE BIT1
#define RED_LED_RADIO_ERROR		 BIT2
#define LED_CHARGER				 BIT3
#define LED_MANUAL				 BIT6
#define LED_AUDIO_OFF  		 	 BIT7

#define LED_DEFAULT_MODE_WL		 BIT0|BIT1|BIT2
#define LED_DEFAULT_MODE_CHARGER BIT3  //|BIT7

extern uint8_t modeLED;

#define RESP_INVALID_PARAMETERS 0x0B
#define RESP_MEDRADIO_TIMEOUT   0x0D
#define RESP_COIL_TIMEOUT	    0x0E
#define RESP_SUCCESS            0x06

#define RESP_LOCAL_PMBOOT_WRITE 0x81
#define RESP_LOCAL_PMBOOT_READ  0x82
#define RESP_LOCAL_PMFILE_READ  0x83

#define INDEX_RESP_SYNC        0 
#define INDEX_RESP_TYPE        1 
#define INDEX_RESP_LEN         2    
#define INDEX_RESP_PAYLOAD     3
#define NON_PAYLOAD_RESP_BYTES 3

#define MAX_CMD_RESPONSE_LEN 255
#define MIN_CMD_RESPONSE_LEN NON_PAYLOAD_RESP_BYTES 
#define AWAITING_RESPONSE   1 

#define INDEX_CMD_SYNC        0 
#define INDEX_CMD_COMMAND     1
#define INDEX_CMD_LEN         2
#define INDEX_CMD_PAYLOAD     3
#define NON_PAYLOAD_CMD_BYTES 3

#define SYNC_BYTE  0xFF

#define MSG_DEPTH 1

void command_handler(uint8_t *packet, uint8_t len, uint8_t route);
void uart_send(uint8_t* packet, uint8_t len);
void ble_send(uint8_t* packet, uint8_t len);

void setLowPower(void);

uint16_t setBuzzerPeriod(uint16_t period, uint16_t timeon);
void ble_start_advertising(void);
void ble_stop_advertising(void);
void power_down_sensor(void);
int16_t get_adc_sample(void);

uint8_t pmboot_write_supervisor(uint8_t *pmboot, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len);
uint8_t pmboot_read_supervisor(uint8_t *pmboot, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len);
uint8_t pmfile_read_supervisor(uint8_t *pmboot, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len);

void resetWL(void);
void setLEDs( uint8_t red, uint8_t green, uint8_t blue);

//void enable_sensor(uint8_t enable);

uint8_t erasebonds(void);
void ble_start_advertising_pairing_mode(void);
uint32_t getpasskey(void);

#endif
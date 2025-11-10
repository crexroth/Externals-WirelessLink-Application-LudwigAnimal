// Doxygen
/*! 
** @defgroup radio Radio Interface
** @ingroup radiomanagement
*/
/*!
** @file   cc1101.h
** @author Hardway
** @date   12/14/2009
**
** @brief State machine for taking incoming packets from the radio and 
** either reading or writing to the local Object Dictionary or routing 
** to the CAN network. The corresponding replies are returned to the source.
**
** @ingroup radio
*/    
#ifndef _CC1101_RADIO_H
#define _CC1101_RADIO_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>




// -------- DEFINITIONS ----------
#define APPCODE_ADDRESS	0x04
#define TOWER_ADDRESS	0x03

#define RADIO_TX_TIMEOUT_TICKS  50/MS_PER_TICK

#define RAD_INT 26


// --------   DATA   ------------
extern uint8_t remoteAddress;

extern struct k_msgq imp_resp_msgq;
extern struct k_msgq imp_req_msgq;
extern struct k_sem medradio_init_ok;
extern struct k_sem medradio_sent;


//JML: does this need the __aligned(4) attribute
struct medRadio_type {
   	uint8_t buf[64]; //must be divisible by 4
	uint8_t len;
	uint8_t source;
	uint8_t unused2;
	uint8_t unused3;
};






// -------- PROTOTYPES ----------
void initRadioConfig( void );
void sendRadioPacket(const uint8_t *data, uint8_t dataLen );
uint8_t getRadioPacket( uint8_t *data );
uint8_t clearChannelSearch(uint8_t dwell, int8_t* maxRSSI, int8_t* avgRSSI);

void idleMedRadio(void);
void powerDownRadio(void);
uint16_t getMedRadioTimeout( void );
void copyRadioSettings(uint8_t* buf);
bool updateMedRadioLocalAddress(uint8_t val);
bool updateMedRadioRemoteAddress(uint8_t val);
bool updateMedRadioChannel(uint8_t val);
bool updateMedRadioTXPower(uint8_t val);
bool updateMedRadioRXTimeout(uint8_t val);
bool updateMedRadioWORInterval(uint8_t val);
bool updateMedRadioRetries(uint8_t val);
bool updateMedRadioEncryption(uint8_t val);
bool updateMedRadioSessionTime(uint8_t val);
bool updateMedRadioMaintain(uint8_t val);

uint16_t getTimeRemainingInSession(); //in ms

int saved_settings_write(uint16_t id, const void* buf, size_t len);
int saved_settings_read(uint16_t id, const void* buf, size_t len);

void radio_interrupt(void);

void enableEncryption(bool en);
int8_t getLastRSSI(void);

void btea(uint32_t *v, int n, uint32_t const key[4]);




#endif
 

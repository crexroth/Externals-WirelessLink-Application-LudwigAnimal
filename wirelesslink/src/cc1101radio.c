// Doxygen
/*!
** @file   cc1101.c
** @author Joris Lambrecht modified from Hardway
** @date   12/14/2009
**
** @brief configuration for c1101 radio chip. 
**
** The configuration is initially determined using the Smart RF Studio from TI.
** The radio is used in Addressed & variable packet mode; 250khz; 
** Set: packets <= 64 bytes; crcOn; idle-after-tx,rx; calib before tx,rx; 
** Monitor GDO0 status on separate pin for rx or tx done status. Isr if needed. 
** Chip is ready for commands on SO low (simply read Miso pin in ioxpin reg).
** @ingroup radio
** 
*/    


/*	THEORY OF OPERATION:

	Half duplex. App tasks call init, send, enable receiver, and read
	routines.  Routines run to completion in < 10msec.  The receiver must be
	re-enabled for each packet received.

	Radio Configuration:
	Using hardware supported packet handler at 100Kbits/sec.  
	Tx Data size = 1-62 bytes (including length and address byte); variable length. 
	Receiver appends rx status to received packet.
	AutoFlush rx buffer on crc error.
	Receiver discards packets on len violation or addr mismatch.
	Offer WakeOnRadio WOR mode since we are online only a few minutes / day.

	Configuration changes 6/6/2011: 
	- Sleep for 1sec, listen for packet for 1.95msec. Using 1385uSec osc stab time. about 61uA.
		Enable RC osc.  Wor has rx timeout, nonWor receives until packet found.
	- Tower streams NoOp messages (894uSec ea) for 1.1 secs; PM discards all NoOps without responding.
	- any received packet retriggers the 5.1sec go-to-sleep timeout.
	- 4byte preamble and sync; gdo0 = 6 for rx (and tx).  Use rising edge EINT0 and poll flag for 
		sync reception(or busy tx'ing) and low level on input pin for pkt received (or tx pkt sent).
	- all packet config is done in IDLE mode.
	- autoFlush rx buffer on crc error.

	Operational changes 4/4/2012: 
	- Using radio interrupt to flag app code when packet reception has begun.
	- GD0 rising edge causes ISR on synch detection; app should call getRadioPacket() after 
		receiving the semaphore.  
	- Data len is returned by getRadioPacket(); no action is required for insufficient len, else 
		process the packet and send response.  Always enable receiver to prepare for next packet.

*/

#include "cc1101radio.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h> //for k_msleep
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "wlgpio.h"
#include "spi.h"
#include "cmdhandler.h"

#define LOG_MODULE_NAME cc1101radio
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "charger.h" //JML temporary until SDO functions are moved outt of charger


K_SEM_DEFINE(medradio_init_ok, 0, 1);
K_SEM_DEFINE(medradio_sent, 0, 1);




// -------- DEFINITIONS ----------

                                                       
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

/* normal defs */
enum				// Command strobes, PATABLE and FIFO addresses
{
	SRES	= 0x30,
	SRX	= 0x34,
	STX	= 0x35,
	SIDLE	= 0x36,
	SWOR	= 0x38,
  SPWD	= 0x39,
	SFRX	= 0x3A,
	SFTX	= 0x3B,
	SNOP	= 0x3D,
	PATABLE	= 0x3E,
	SFIFO	= 0x3F
};

enum				// Status Register Addresses
{
	LQI	= 0x33,
	RSSI	= 0x34,
	TXBYTES	= 0x3A,
	RXBYTES	= 0x3B
};

enum CcRegister
{
    FSCTRL1  = 0x0B,  // frequency synthesizer control.
    FSCTRL0  = 0x0C,  // frequency synthesizer control.
    FREQ2    = 0x0D,  // frequency control word, high byte.
    FREQ1    = 0x0E,  // frequency control word, middle byte.
    FREQ0    = 0x0F,  // frequency control word, low byte.
    MDMCFG4  = 0x10,  // modem configuration.
    MDMCFG3  = 0x11,  // modem configuration.
    MDMCFG2  = 0x12,  // modem configuration.
    MDMCFG1  = 0x13,  // modem configuration.
    MDMCFG0  = 0x14,  // modem configuration.
    CHANNR   = 0x0A,  // channel number.
    DEVIATN  = 0x15,  // modem deviation setting (when fsk modulation is enabled).
    FREND1   = 0x21,  // front end rx configuration.
    FREND0   = 0x22,  // front end tx configuration.
    MCSM2    = 0x16,  // main radio control state machine configuration.
    MCSM1    = 0x17,  // main radio control state machine configuration.
    MCSM0    = 0x18,  // main radio control state machine configuration.
    FOCCFG   = 0x19,  // frequency offset compensation configuration.
    BSCFG    = 0x1A,  // bit synchronization configuration.
    AGCCTRL2 = 0x1B,  // agc control.
    AGCCTRL1 = 0x1C,  // agc control.
    AGCCTRL0 = 0x1D,  // agc control.
    FSCAL3   = 0x23,  // frequency synthesizer calibration.
    FSCAL2   = 0x24,  // frequency synthesizer calibration.
    FSCAL1   = 0x25,  // frequency synthesizer calibration.
    FSCAL0   = 0x26,  // frequency synthesizer calibration.
    FSTEST   = 0x29,  // frequency synthesizer calibration.
    TEST2    = 0x2C,  // various test settings.
    TEST1    = 0x2D,  // various test settings.
    TEST0    = 0x2E,  // various test settings.
    FIFOTHR  = 0x03,  // rxfifo and txfifo thresholds.
    IOCFG2   = 0x00,  // gdo2 output pin configuration.
    IOCFG1   = 0x01,  // gdo1 output pin configuration.
    IOCFG0   = 0x02,  // gdo0 output pin configuration. refer to smartrf� studio user manual for detailed pseudo register explanation.
    SYNC1    = 0x04,  // MSB.
    SYNC0    = 0x05,  // LSB.
    PKTCTRL1 = 0x07,  // packet automation control.
    PKTCTRL0 = 0x08,  // packet automation control.
    ADDR     = 0x09,  // device address.
    WOREVT1  = 0x1E,  // WOR event0 timeout HighByte.
    WOREVT0  = 0x1f,  // WOR event0 timeout LowByte.
    WORCTRL  = 0x20,  // WOR control.
    PKTLEN   = 0x06   // packet length.
};


struct CcInit
{
    enum CcRegister regNum;
    uint8_t value;
};



// --------   DATA   ------------

static struct
{
	enum {RS_IDLE, RS_RECEIVER, RS_TRANSMITTER, RS_PREAMBLE, RS_SEARCH} state;
	uint8_t sessionTime;  // if 0, session concepts are disabled.  SHould be 5s for MedRadio compliance
	uint8_t inSession; //starts false, clear channel search sets to true, session end timer sets to false (receiving valid packet resets timer)  
	uint8_t newSession;   //starts false, gets set true after clear channel search, set to false on receiving valid packet. indicates that TX needs to include long preamble
	uint8_t isrCnt;
	uint8_t errorCnt;
	uint8_t localAddress;
	uint8_t remoteAddress;
	uint8_t channel;
	uint8_t txPower;
	uint8_t worInterval;
	uint8_t rxTimeout;
	uint8_t retries;
	uint8_t encryption;
} radio;

#define RADIO_TX_POWER 20
#define RADIO_CHAN 3
#define RADIO_ADDR 1

/*************
	Chipcon

	Product = CC1101
	Chip version = A   (VERSION = 0x04)
	Crystal accuracy = 10 ppm
	X-tal frequency = 26 MHz
	RF output power = -15 dBm

	RX filterbandwidth = 325K 
	Deviation = 47.6K	
	Datarate = 100k		
	Modulation = (1) GFSK
	Manchester enable = (0) Manchester disabled
	RF Frequency = 402.15M		
	Channel spacing = 299.9		
	IF Freq = 203.1K	

	Channel number = 0
	Optimization = Sensitivity
	Sync mode = (3) 30/32 sync word bits detected
	Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
	CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
	Forward Error Correction = (0) FEC disabled
	Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
	Packetlength = 61
	Preamble count = (2)  4 bytes
	Append status = 1
	Address check = (1) Address check, no broadcast
	FIFO autoflush = 1
	Device address = 190 (0xBe)
	GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
	GDO2 signal selection = (41) CHIP_RDY
	GDO1 signal selection = (41) CHIP_RDY
	Wor on at 1sec x 1.9msec and 2.4 stabilize
*************/

struct CcInit ccInitTable[] = {
	IOCFG2,   0x06,  //0x08 asserts when PQT reached //0x29,  //gdo2 output pin configuration; ChipRdy.
	IOCFG0,   0x06,  //gdo0 output pin configuration. high on startof_tx/rx; low on end-of-packet.
	FIFOTHR,  0x07,  //rxfifo and txfifo thresholds. use 0x47 if MDMCFG4 is not 0x5B
	PKTLEN,   0x3F,	 //max packet length byte value 
	PKTCTRL1, 0x0D,  //packet automation control. -- enable autoflush_CRC; append status; 
	PKTCTRL0, 0x05,  //packet automation control. -- CRC enabled (bit 2);
	ADDR,    RADIO_ADDR,  //device address.
	CHANNR,  RADIO_CHAN,  //channel number.
	FSCTRL1,  0x08,  //frequency synthesizer control.
	FSCTRL0,  0x00,  //frequency synthesizer control.
	FREQ2,    0x0f,  //frequency control word, high byte.
	FREQ1,    0x77,  //frequency control word, middle byte.
	FREQ0,    0xa2,  //frequency control word, low byte.
	MDMCFG4,  0x5b,  //modem configuration. 0x8b works too and reduces off-band effects
	MDMCFG3,  0xf8,  //modem configuration.
	MDMCFG2,  0x13,  //modem configuration.
	MDMCFG1,  0x23,  //modem configuration.
	MDMCFG0,  0x7a,  //modem configuration.
	DEVIATN,  0x47,  //modem deviation setting (when fsk modulation is enabled).
  MCSM0,    0x18,  //main radio control state machine configuration.
	FOCCFG,   0x1D,  //frequency offset compensation configuration.
	BSCFG,    0x1C,  //bit synchronization configuration.
	AGCCTRL2, 0xC7,  //agc control.
	AGCCTRL1, 0x00,  //agc control.
	AGCCTRL0, 0xB2,  //agc control.
	WORCTRL,  0xfb,	 //not using WOR
	FREND1,   0xB6,  //front end rx configuration.
	FREND0,   0x10,  //front end tx configuration.
	FSCAL3,   0xEA,  //frequency synthesizer calibration.
	FSCAL2,   0x2A,  //frequency synthesizer calibration.
	FSCAL1,   0x00,  //frequency synthesizer calibration.
	FSCAL0,   0x1F,  //frequency synthesizer calibration.
	FSTEST,   0x59,  //frequency synthesizer calibration.
	TEST2,    0x88,  //various test settings.
	TEST1,    0x31,  //various test settings.
	TEST0,    0x0b   //various test settings.
		
};

#define NUM_CCINIT_REGS		(sizeof( ccInitTable ) / sizeof( struct CcInit ))



                                 //reg      #  dBm     mA
const uint8_t ccValueLookUp[47] = { 0x6F,  // 0 -15.50  12.4
                                  0x6E,  // 1  -8.90  12.8
                                  0x6D,  // 2  -7.70  13
                                  0x6C,  // 3  -7.10  13.2
                                  0x6B,  // 4  -6.50  13.3
                                  0x6A,  // 5  -5.90  13.5
                                  0x69,  // 6  -5.30  13.6
                                  0x68,  // 7  -4.70  13.8
                                  0x67,  // 8  -4.10  14
                                  0x57,  // 9  -4.00  14.1
                                  0x66,  //10  -3.50  14.2
                                  0x56,  //11  -3.30  14.3
                                  0x55,  //12  -2.80  14.5
                                  0x64,  //13  -2.30  14.7
                                  0x54,  //14  -2.20  14.8
                                  0x53,  //15  -1.50  15
                                  0x52,  //16  -0.90  15.3
                                  0x40,  //17  -0.80  15.4
                                  0x61,  //18  -0.50  15.6
                                  0x51,  //19  -0.30  15.7
                                  0x60,  //20   0.10  15.9 *Bootloader/Default Setting
                                  0x50,  //21   0.40  16
                                  0x8D,  //22   1.40  16.8
                                  0x8C,  //23   1.90  17.1
                                  0x8B,  //24   2.30  17.3
                                  0x8A,  //25   2.80  17.6
                                  0x89,  //26   3.20  17.9
                                  0x88,  //27   3.60  18.2
                                  0x87,  //28   4.00  18.5
                                  0x86,  //29   4.40  18.8
                                  0x85,  //30   4.80  19.1
                                  0x84,  //31   5.10  19.4
                                  0x83,  //32   5.50  19.7
                                  0x82,  //33   5.80  20
                                  0x81,  //34   6.00  20.3
                                  0x80,  //35   6.30  20.6
                                  0xCA,  //36   6.40  23.4
                                  0xC9,  //37   6.80  23.8
                                  0xC8,  //38   7.10  24.2
                                  0xC7,  //39   7.40  24.7
                                  0xC6,  //40   7.80  25.2
                                  0xC5,  //41   8.10  25.7
                                  0xC4,  //42   8.50  26.3
                                  0xC3,  //43   8.80  26.9
                                  0xC2,  //44   9.20  27.6
                                  0xC1,  //45   9.50  28.3
                                  0xC0   //46   9.90  29.1
                                  };

																	









K_MSGQ_DEFINE(imp_req_msgq, sizeof(struct medRadio_type), MSG_DEPTH, 4);
K_MSGQ_DEFINE(imp_resp_msgq, sizeof(struct medRadio_type), MSG_DEPTH, 4);

																
// -------- PROTOTYPES ----------

//static void configRadioControlPins( void );

//static void flushFifosSetIdle( void );

static uint8_t RADIO_RSSI;
static uint8_t RADIO_LQI;

static void startTx( void );
static void startRx( void );
static void startRxSearch( void );

static uint8_t readStatusRegister( uint8_t regNum );
static uint8_t readRegister( uint8_t regAddr);
static uint8_t readRegisters( uint8_t regAddr, uint8_t *buf, uint8_t numRegisters );
static uint8_t writeRegister( uint8_t regAddr, uint8_t value );
static uint8_t writeRegisters( uint8_t regAddr, uint8_t *buf, uint8_t numRegisters );
static void resetRadio(void);

int8_t getLastRSSI(void);

 
//============================
//    DEFINES
//============================

/* read status once and use macros to decipher it, to reduce 
	the number of cc reads since the reads affect the chip. see errata pdf */

#define SEND_COMMAND_STROBE_GET_RXFIFO(cmd)		readRegisters( (cmd), NULL, 0 )
#define SEND_COMMAND_STROBE_GET_TXFIFO(cmd)		writeRegisters( (cmd), NULL, 0 )

#define MAX_DATALEN			60		// 64 -len-addr-rssi-lqi bytes
#define TXRX_MODE_MS		        2               

#define RSSIOFFSET  74


void  medRadio_session_end_action(struct k_timer *dummy)
{
	radio.inSession = false;
	LOG_INF("MedRadio Session ended");
	
}

K_TIMER_DEFINE(medRadio_session_timer, medRadio_session_end_action, NULL);



int8_t getLastRSSI(void)
{
	int8_t rssi;
	if(RADIO_RSSI < 128)
	{
        rssi =  RADIO_RSSI/2 - RSSIOFFSET;
	}
    else
	{
		rssi = (RADIO_RSSI-256)/2 - RSSIOFFSET;
	}
	return rssi;
}

uint16_t getTimeRemainingInSession()
{
	uint32_t t = k_timer_remaining_get(&medRadio_session_timer);
	return (uint16_t) t;  //session time is max 60s, value in ms
}


void maintain_medRadio_thread(void)
{
	//uint8_t pkt[] = {0x0B, 0x24, 0x00, 0x01, 0x07, 0x18, 0x10, 0x04, 0x00};
	uint8_t resp[21];
	uint16_t b[3];
	uint16_t bavg;
	
	while(1)
	{
		if(getTimeRemainingInSession() == 0 )
		{
			LOG_INF("Could not maintain session");
		}
		else if (getTimeRemainingInSession() < 500 ) //if less than 0.5 s remains in the session try sending a packet until response received
		{
			LOG_INF("Nearing end of session - transmitting to maintain");
			//sendRadioPacket(pkt, sizeof(pkt));
			ReadSDO(7, 0x3000, 13, 1, resp);
			memcpy(b, &resp[6], sizeof(b));
			bavg = (b[0] + b[1] + b[2])/3;
			LOG_INF("battery %d", bavg);

		}
		k_msleep(100);
	}
}

#define MEDRADIO_SESSION_STACKSIZE 1024
#define MEDRADIO_SESSION_PRIORITY 9
K_THREAD_DEFINE(medRadio_session_thread_id, MEDRADIO_SESSION_STACKSIZE, maintain_medRadio_thread, NULL, NULL, NULL, MEDRADIO_SESSION_PRIORITY, 0, 0);

bool updateMedRadioMaintain(uint8_t val)
{
	if (val==1 && radio.sessionTime > 0)
	{
		k_thread_resume(medRadio_session_thread_id);
		LOG_INF("Starting Maintain MedRadio Session Thread");
		return true;
	}
	else if (val==0)
	{
		k_thread_suspend(medRadio_session_thread_id);
		LOG_INF("Suspending Maintain MedRadio Session Thread");
		return true;
	}
	else{
		return false;
	}

}



void isr_offload_function(struct k_work *work_tem)
{
	LOG_INF("ISR Offload function");
	if(radio.state == RS_TRANSMITTER)
	{
		//Finished Transmit
		startRx();
		k_sem_give(&medradio_sent);
		LOG_INF("StartRX done");
	}
	else if(radio.state == RS_RECEIVER)
	{
		//Finished Receive
		//handle the radio message.  
		//Message will go out to BLE on main loop and then return to idle
		struct medRadio_type medRadio;
		medRadio.len = getRadioPacket(&medRadio.buf);
		
		

		while(k_msgq_put(&imp_resp_msgq, &medRadio, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging ImpResp msg_q");
            k_msgq_purge(&imp_resp_msgq);
		}
		LOG_INF("ImplantResponse k_msg_q_put complete");


	}
	else
	{
		LOG_INF("Errant MedRadio Interrupt!");
	}

}
K_WORK_DEFINE(isr_work, isr_offload_function);



void radio_interrupt(void)
{
	k_work_submit(&isr_work);
}


bool updateMedRadioLocalAddress(uint8_t val)
{
	
	if(val == 0 ||val == 255)
	{
		return false;
	}
	else if( val == radio.localAddress)
	{
		return true;
	}
	else
	{
		LOG_INF("Update Local Address");
		radio.localAddress = val;

		writeRegister( ADDR, val );

		return true;
	}
}

bool updateMedRadioRemoteAddress(uint8_t val)
{
	if(val == 0 ||val == 255 )
	{
		return false;
	}
	else
	{
		LOG_INF("Update Remote Address");
		radio.remoteAddress = val;
		return true;
	}
}

bool updateMedRadioChannel(uint8_t val)
{
	if(val > 9)
	{
		return false;
	}
	else if( val == radio.channel)
	{
		return true;
	}
	else
	{
		LOG_INF("Update Chan");
		radio.channel = val;
		writeRegister( CHANNR, val );
		
		return true;
	}
}

bool updateMedRadioTXPower(uint8_t val)
{
	if(val >= sizeof(ccValueLookUp))
	{
		return false;
	}
	else if (val == radio.txPower)
	{
		return true;
	}
	else
	{
		LOG_INF("Update TX Power.");
		radio.txPower = val;
		writeRegister( PATABLE, ccValueLookUp[val] );
		
		return true;
	}
}

bool updateMedRadioWORInterval(uint8_t val)
{
	if(val < 14 && val > 0)
	{
		val = 14;
	}

	LOG_INF("Update WOR");
	radio.worInterval= val;
	return true;
	
}

bool updateMedRadioRetries(uint8_t val)
{
	if(val > 5)
	{
		return false;
	}
	else
	{
		LOG_INF("Update Retries");
		radio.retries= val;
		return true;
	}
}

bool updateMedRadioRXTimeout(uint8_t val)
{
	if(val < 5)
	{
		return false;
	}
	else
	{
		LOG_INF("Update Timeout");
		radio.rxTimeout = val;
		return true;
	}
}

bool updateMedRadioEncryption(uint8_t val)
{
	if(val > 1)
	{
		return false;
	}
	else
	{
		radio.encryption = val;
		LOG_INF("Update Encryption");
		return true;
	}		
}

bool updateMedRadioSessionTime(uint8_t val)
{
	if(val > 60)
	{
		return false;
	}
	else if (val == radio.sessionTime)
	{
		return true; //don't force end os session if value hasn't changed
	}
	else
	{
		//end session timer if running
		if (radio.inSession)
		{
			k_timer_stop(&medRadio_session_timer);
		}
		radio.sessionTime = val;
		radio.inSession = false; //force start of new session
		if( val==0)
		{
			k_thread_suspend(medRadio_session_thread_id);
		}
		LOG_INF("Update SessionTime");
		return true;
	}
}

void copyRadioSettings(uint8_t* buf)
{
	buf[0] = radio.localAddress;
	buf[1] = radio.remoteAddress;
	buf[2] = radio.channel;
	buf[3] = radio.txPower;
	buf[4] = radio.worInterval;
	buf[5] = radio.rxTimeout;
	buf[6] = radio.retries;

	LOG_HEXDUMP_INF(buf, 7, "Radio Settings");
}

uint16_t getMedRadioTimeout( void )
{
	uint16_t timeout;
	
	switch(radio.rxTimeout)
	{
		case 255:
			timeout = 0;
			break;
		case 254:
			timeout = 40000;
			break;
		case 253:
			timeout = 10000;
			break;
		case 252:
			timeout = 4000;
			break;
		case 251:
			timeout = 1000;
			break;
		default: 
			timeout = radio.rxTimeout;
	}

	return timeout;
}

/**********************************************************************************************************
*                                             initRadioConfig()
**********************************************************************************************************/
/**
* @brief  Sets up radio including interrupt
*
* @param none
* @return none
*
*/
void initRadioConfig( void )
{
	
	uint8_t   ccValue;
	struct CcInit *reg;
	uint8_t	buf[7];

	radio.state = RS_IDLE;
	resetRadio();
	k_msleep(1);

	/* copy configuration registers */
	for( reg = ccInitTable ; reg < &ccInitTable[ NUM_CCINIT_REGS ] ; reg++ )
  	{
		writeRegister( reg->regNum, reg->value );
    }
                
	ccValue = ccValueLookUp[RADIO_TX_POWER];					
	writeRegister( PATABLE, ccValue );
 
	
	/* verify configuration registers */
	for( reg = ccInitTable ; reg < &ccInitTable[ NUM_CCINIT_REGS ] ; reg++ )
	{
		if( readRegister( reg->regNum ) != reg->value )
		{
			LOG_INF("Radio Init Failed Reg# %02X!", reg->regNum);
			LOG_INF("RegValue Intended %02X!", reg->value);
			LOG_INF("RegValue Read %02X!", readRegister( reg->regNum ));
			
			return;
		}
	}
	
	//store default settings in radio structure
	radio.localAddress = RADIO_ADDR;
	radio.remoteAddress = 2;
	radio.channel = RADIO_CHAN;
	radio.txPower = RADIO_TX_POWER;
	radio.worInterval = 20;
	radio.rxTimeout = 100;
	radio.retries = 0;
	radio.inSession = false;
	radio.newSession = false;
	radio.sessionTime = 0; //JML hard coded - this should be one of the flash settings, use 5 for MedRadio
	
	configureRadioInterrupt(); //radio interrupt configured in wlgpio.c

	if(saved_settings_read(RADIO_SETTINGS_ID, buf, sizeof(buf)) > 0)
	{
		//replace with read from flash, and then update Settings
		updateMedRadioLocalAddress(buf[0]);
		updateMedRadioRemoteAddress(buf[1]);
		updateMedRadioChannel(buf[2]);
		updateMedRadioTXPower( buf[3]);
		updateMedRadioWORInterval(buf[4]);
		updateMedRadioRXTimeout(buf[5]);
		updateMedRadioRetries(buf[6]);

	} else{
		LOG_INF("No Radio settings found in flash");
	}

	k_thread_suspend(medRadio_session_thread_id);

	k_sem_give(&medradio_init_ok);

}












/**********************************************************************************************************
*                                             sendRadioPacket()
**********************************************************************************************************/
/**
 * @brief using variable data len (0-60), addressed messages. Limiting data to 60 
 *   so the receiving radio can tack on rssi + lqi bytes. Radio returns to Idle state, 
 *   Receive mode must be reenabled by the calling function
 * @param deviceID
 * @param *data
 * @param dataLen: packet struct: Pktlen +addr +data[ dataLen = 0-60 bytes ].   PktLen 
 *                includes the address byte, so it is dataLen + 1.
 */ 
void sendRadioPacket( const uint8_t *data, uint8_t dataLen )
{
	
	//if session time configured and session ended, we need to do channel search to start enw session
	if (radio.sessionTime > 0 && !radio.inSession) 
	{
		clearChannelSearch(1, NULL, NULL);
	}
	
	uint8_t	msg[MAX_DATALEN+2] = {0};

		
	if( dataLen > MAX_DATALEN )
		return;

        
    memcpy( &msg[2], data, dataLen ); //copy from pointer into msg

	//encryption

	if(radio.encryption)
	{
		uint32_t p[15];
		uint32_t k[4] = {0x1EA098D4, 0x8FAECA4B, 0x2BBCF0DA, 0xFA12E8E4};
		uint8_t pad=0;
		static uint16_t cnt;
		
		dataLen+=2;
		pad = (4 - dataLen%4)&0x03;
		dataLen+=pad;
		if( dataLen > MAX_DATALEN || dataLen%4 != 0 ) //dataLen must be a multiple of 4 now
			return;

		cnt++;
		if(cnt>0x3FFF) {cnt = 0;}

		//add counter and padding indicator as last 2 bytes
		msg[dataLen]=cnt & 0xFF;  
		msg[dataLen+1]=(pad<<6) | (cnt>>8);

		memcpy(p,&msg[2],dataLen);
		btea(p,dataLen/4,k);
		memcpy( &msg[2], p, dataLen );
	}

	
	/* build header */
	msg[0] = dataLen + 1;
	msg[1] = radio.remoteAddress;
	

	if(radio.newSession && radio.sessionTime > 0)
	{
		//preamble must be long enough to guarantee PM will pick it up as its cycling through channels
		startTx();	//Strobe and delay (for preamble)
		LOG_INF("New session: Using long TX preamble");
		k_msleep((radio.worInterval + 4)*10+5); 

		
		/* load Tx buffer */
		writeRegisters( SFIFO, msg, dataLen + 2 );

	}
	else
	{
		if(radio.worInterval == 0)
		{
			/* load Tx buffer */
			writeRegisters( SFIFO, msg, dataLen + 2 );
			startTx();	//Strobe 
		}
		else
		{
			startTx();	//Strobe and delay (for preamble)
			k_msleep(radio.worInterval + 2); //pause 22ms
			/* load Tx buffer */
			writeRegisters( SFIFO, msg, dataLen + 2 );
		}
	}
	//JML NOTE: Is it possible for the TX radio interrupt to fire before reaching the next line?
	radio.state = RS_TRANSMITTER; //IRQ should only look to end transmit after preamble sent

	//The interrupt at the end of transmission allows radio to switch to receive mode             
	
}


#define MIN_MEDRADIO_PKT_LEN		4					// len, addr, data, rssi, lqi


/**********************************************************************************************************
*                                             getRadioPacket()
**********************************************************************************************************/
/**
* @brief When this function is called, it will block the calling task until a radio packet is received    
*       Receiving a packet returns the radio to Idle.  enableRadioReceiver must be called to go back to RX or WOR mode
*	
*	We are using variable len, addressed messages.
*	packet struct: Pktlen +addr +data(1-58bytes) +rssi +lqi.
*	PktLen count includes the address+rssi+lqi, so it is data count + 5.
*	Rssi and Lqi are appended by receiver chip.
*	
*	In WakeOnRadio (WOR) mode the receiver is in RX mode for a short duty cycle. The device sending the packet must send a preamble 
*      that is sufficiently long to insure that the receiving radio wakes up, and stays in RX mode to hear the sent message
*      Once WOR is enabled, the radio will go back to WOR mode after sending a response.  WOR mode
*      is on by default if BIT6 of BatteryControl_PowerControl is set
*
* @param *data pointer to location to stored packet data
* @return number of bytes received(including status), else 0 if none or w/errors
*/

uint8_t getRadioPacket( uint8_t *data )
{
	
    uint8_t len = 0 ;
   
                     
    /* packet is in fifo, read all data */

    len = readStatusRegister( RXBYTES );
    
    if (len > MIN_MEDRADIO_PKT_LEN)
	{ 
		readRegisters( SFIFO, data, len );

		RADIO_RSSI = data[len - 2]; 	// last two bytes of received data
		RADIO_LQI  = data[len - 1];
		
		if(radio.encryption)
		{
			//decryption
			uint32_t p[15];
			uint32_t k[4] = {0x1EA098D4, 0x8FAECA4B, 0x2BBCF0DA, 0xFA12E8E4};
			int8_t n;
			uint8_t pad;
			
			memcpy(p, &data[2], len-4); //copy payload everything but first 2 and last 2 bytes)
			n = (len-4)/4;
			btea(p, -n, k);
			memcpy(&data[2], p, len-4);
			pad = (data[len-3])>>6;
			LOG_INF("pad %X", pad);
			LOG_HEXDUMP_INF(data, len, "Received MedRadio Packet-prepadremoval");
			//make packet consistent with unencrypted packets (remove padding and counter)
			data[len-pad-4]=data[len-2];
			data[len-pad-3]=data[len-1];
			data[0]-=(pad+2);
			len-=(pad+2);
		}
		
		//if radio session timer enabled
		if (radio.sessionTime > 0 && radio.inSession)
		{
			k_timer_start(&medRadio_session_timer, K_MSEC(5000), K_NO_WAIT); //start the session timer over
			radio.newSession = false;
		}
		LOG_HEXDUMP_INF(data, len, "Received Valid MedRadio Packet");
	}
	else
	{
		LOG_INF("Received Errant MedRadio Packet, length: %d", len);
	}
	idleMedRadio();
    return len;	
}



//dwell time in multiples of 10ms.  
//returns clearest channel based on lowest maximum RSSI across dwell time
//also sets channel and starts session timer
//sets maxRSSI and avgRSSI.  These should each be 10 byte arrays (ch 0 to ch 9) 
uint8_t clearChannelSearch(uint8_t dwell, int8_t* maxRSSI, int8_t* avgRSSI){

	uint8_t ch = readRegister(CHANNR);
	if (dwell==0)
	{
		return ch;
	}

	
	uint8_t regMDMCFG2 = readRegister(MDMCFG2);
	uint8_t regPKTCTRL1 = readRegister(PKTCTRL1);
	uint8_t regPKTCTRL0 = readRegister(PKTCTRL0);


    uint8_t clearestChannel = ch;
    //int32_t clearestSum = 9999;
    int32_t sum;
    uint8_t rssi_raw = 0;
    int16_t rssi_dBm = 0;
	int16_t chMaxRSSI = -9999;
	int16_t chAvgRSSI = -9999;
	int16_t minRSSI = 9999;
	uint64_t timeCheck;
	
	if(dwell==255 && radio.sessionTime >0)
	{
		dwell = radio.sessionTime*100; //max 60*100, usually 5*100
	}
	uint32_t nsamples;
    uint8_t i = 0;
    uint32_t j= 0;

    writeRegister(MDMCFG2, regMDMCFG2 &~ (BIT0|BIT1|BIT2)); //clear SYNC_MODE to prevent sync detection
	writeRegister(PKTCTRL1, regPKTCTRL1 &~ (BIT0|BIT1|BIT3)); //clear ADR_CHECK and CRC_AUTOFLUSH
	writeRegister(PKTCTRL0, 0x12); //PKTFORMAT = 1, LENGTH_CONFIG  =2
    startRxSearch();
	

    for (i=0; i<10; i++) //channel loop (starts on current channel)
    {
        k_msleep(1); //delay to allow RSSI to settle on new channel
        sum = 0; //reset sum
		chMaxRSSI = -9999; //reset maxRSSI
		timeCheck = k_uptime_get();
		nsamples = 0;

        while (k_uptime_get() - timeCheck <= dwell*10) //sample loop (takes dwell*10 ms)
        {
            nsamples++;
			rssi_raw = readStatusRegister(RSSI);

            if (rssi_raw >= 128)
            {
                rssi_dBm = (((int16_t)rssi_raw - 256) / 2) - RSSIOFFSET;
            }
            else
            {
                rssi_dBm = ((int16_t)rssi_raw / 2) - RSSIOFFSET;
            }

            sum += rssi_dBm;
			if (rssi_dBm > chMaxRSSI)
			{
				chMaxRSSI = rssi_dBm;
			}	
        }
		//Usually RSSI will not be lower than -128, but it could be.  Saturate at -128 to fit in an int8
		if (maxRSSI != NULL)
		{
			if(chMaxRSSI<-128){
				maxRSSI[ch]=-128;
			}
			else{
				maxRSSI[ch]=(int8_t) chMaxRSSI;
			}
		}
		chAvgRSSI =  (int16_t)(sum/(int32_t)nsamples);
	
		if (avgRSSI != NULL)
		{
			if(chAvgRSSI<-128){
				avgRSSI[ch]=-128;
			}
			else{
				avgRSSI[ch]=(int8_t) chAvgRSSI;
			}
		}

		if (chMaxRSSI < minRSSI && ch != 5) //don't allow use of channel 5
		{
			minRSSI = chMaxRSSI;
			clearestChannel = ch;
		}

        ch++;
        if(ch==10)
        {
            ch=0;
        }
        writeRegister(CHANNR, ch); //set new channel
		

    }

   
    writeRegister(CHANNR, clearestChannel); //set channel to clearest channel
    radio.channel = clearestChannel;
    writeRegister(MDMCFG2, regMDMCFG2); //restore settings
	writeRegister(PKTCTRL1, regPKTCTRL1); //restore settings
	writeRegister(PKTCTRL0, regPKTCTRL0); //restore settings

	//if session timer enabled, start it
	if (radio.sessionTime > 0)
	{
		LOG_INF("MedRadio Session started on Channel %d", clearestChannel);
		k_timer_start(&medRadio_session_timer, K_MSEC(5000), K_NO_WAIT); //start the session
		radio.inSession = true;
		radio.newSession = true;
	}
    return clearestChannel;

}






/**********************************************************************************************************
*                                            powerDownRadio()
**********************************************************************************************************/
/**
 *@brief Turns off radio.
 *@return none
 */
void powerDownRadio()
{
      SEND_COMMAND_STROBE_GET_RXFIFO( SIDLE );
      SEND_COMMAND_STROBE_GET_RXFIFO( SPWD );
}



//============================
//    LOCAL CODE
//============================


static uint8_t tx_buf[64];  
static uint8_t rx_buf[64];

static uint8_t writeRegister( uint8_t regAddr, uint8_t value )
{
	return writeRegisters( regAddr, &value, 1 ); //returns status
}

static uint8_t readRegister( uint8_t regAddr)
{
	uint8_t value;
	readRegisters( regAddr, &value, 1 ); //nothing done with status
	
	return value;
}

static uint8_t readStatusRegister( uint8_t regAddr )
{
	uint8_t value;
	
	readRegisters( regAddr | BIT6, &value, 1 ); //nothing done with status
	
	return value;
}


static uint8_t readRegisters( uint8_t regAddr, uint8_t *buf, uint8_t numRegisters )
{
	// read num config registers from cc1101 device starting at address
	// return chip status
	//^^ SPI1 is not a shared resource, so no blocking required
	
	uint8_t header, burst;

	burst = (numRegisters > 1) ?  BIT6 : 0 ;
	header = regAddr | burst | BIT7;  //BIT7 is for read

	memset(tx_buf, 0, numRegisters+1);
	tx_buf[0] = header;
	
	spi_transmit(tx_buf, rx_buf, numRegisters+1); //write headerbyte + numRegisters
	
	memcpy(buf, &rx_buf[1], numRegisters);
	
	return rx_buf[0]; //status
}



static uint8_t writeRegisters( uint8_t regAddr, uint8_t *buf, uint8_t numRegisters )
{
	// write num config registers to cc1101 device starting at address
	// return chip status
	//^^ SPI1 is not a shared resource, so no blocking required
	
	//JML note: the 64 bte allocation for tx_buf and rx_buf 
	
	uint8_t header, burst;

	burst = (numRegisters > 1) ?  BIT6 : 0 ;
	header = regAddr | burst ;
		
	tx_buf[0] = header;
	memcpy(&tx_buf[1], buf, numRegisters); 
	
	spi_transmit(tx_buf, rx_buf, numRegisters+1) ; //write headerbyte + numRegisters
		
	return rx_buf[0]; //status
}


static void resetRadio()
{

	radio.state = RS_IDLE;

	tx_buf[0] = SRES;
	
	spi_transmit(tx_buf, rx_buf, 1) ; 
		
}

static void startTx()
{

	radio.state = RS_PREAMBLE;

	tx_buf[0] = SIDLE; //may not be necessary
	tx_buf[1] = SNOP;  //may not be necessary
	tx_buf[2] = SFRX;  //flush RX fifo
	tx_buf[3] = STX;  //enter TX mode
	
	spi_transmit(tx_buf, rx_buf, 4) ; 
		
}

static void startRx( void )
{
	radio.state = RS_RECEIVER;
	
	tx_buf[0] = SIDLE; //may not be necessary
	tx_buf[1] = SNOP;  //may not be necessary
	tx_buf[2] = SFTX;  //flush TX fifo
	tx_buf[3] = SRX;   //enter RX mode
	
	spi_transmit(tx_buf, rx_buf, 4) ; 
		
}

static void startRxSearch( void )
{
	radio.state = RS_SEARCH;
	
	tx_buf[0] = SIDLE; //enter Idle mode
	tx_buf[1] = SRX;   //enter RX mode
	
	spi_transmit(tx_buf, rx_buf, 2) ; 
		
}

void idleMedRadio( void )
{
	
	//memset(medRadioRX_buf, 0, sizeof(medRadioRX_buf));
	//medRadioRX_len = 0;
	
	radio.state = RS_IDLE;
	
	tx_buf[0] = SIDLE; //enter Idle mode
	tx_buf[1] = SNOP;  //may not be necessary
	tx_buf[2] = SFRX;  //may not be necessary
	tx_buf[3] = SFTX;  //may not be necessary
	
	spi_transmit(tx_buf, rx_buf, 4) ; 
		
}

//from: https://en.wikipedia.org/wiki/XXTEA clarified version
  #define DELTA 0x9e3779b9
  #define MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)))
  
  void btea(uint32_t *v, int n, uint32_t const key[4]) {
    uint32_t y, z, sum;
    unsigned p, rounds, e;
    if (n > 1) {          /* Coding Part */
      rounds = 12 + 52/n; //replace 6 with 12?
      sum = 0;
      z = v[n-1];
      do {
        sum += DELTA;
        e = (sum >> 2) & 3;
        for (p=0; p<n-1; p++) {
          y = v[p+1]; 
          z = v[p] += MX;
        }
        y = v[0];
        z = v[n-1] += MX;
      } while (--rounds);
    } else if (n < -1) {  /* Decoding Part */
      n = -n;
      rounds = 12 + 52/n; //replace 6 with 12?
      sum = rounds*DELTA;
      y = v[0];
      do {
        e = (sum >> 2) & 3;
        for (p=n-1; p>0; p--) {
          z = v[p-1];
          y = v[p] -= MX;
        }
        z = v[n-1];
        y = v[0] -= MX;
        sum -= DELTA;
      } while (--rounds);
    }
  }

















#include "charger.h"
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h> //for k_msleep
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include "wlgpio.h"
#include "cmdhandler.h"
#include "cc1101radio.h"
#include <zephyr/drivers/watchdog.h>
#include "audio.h"
#include "customcharacter.h"
#include <stdio.h>
//#include <zephyr/sys_clock.h>


typedef struct 
{
	uint16_t degC;
	uint16_t adsCounts;
	
} THERMTBL;


//degC(100=10.0), adsCounts   based on 32768 FS
static THERMTBL const thermTable[] =
{	{100,	23798},
	{110,	23483},
	{120,	23163},
	{130,	22839},
	{140,	22511},
	{150,	22179},
	{160,	21844},
	{170,	21506},
	{180,	21164},
	{190,	20821},
	{200,	20475},
	{210,	20127},
	{220,	19778},
	{230,	19427},
	{240,	19076},
	{250,	18725},
	{260,	18373},
	{270,	18021},
	{280,	17670},
	{290,	17320},
	{300,	16971},
	{310,	16623},
	{320,	16278},
	{330,	15933},
	{340,	15592},
	{350,	15253},
	{360,	14915},
	{370,	14586},
	{380,	14256},
	{390,	13931},
	{400,	13609},
	{410,	13291},
	{420,	12977},
	{430,	12668},
	{440,	12363},
	{450,	12063},
	{460,	11764},
	{470,	11473},
	{480,	11190},
	{490,	10909},
	{500,	10634} };

#define LOG_MODULE_NAME charger
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"
#define MAX_PATH 128
#define PARAM_FILE_NAME "param.dat"
#define LOG_FILE_NAME "log.dat"
#define USER_DIR_NAME "user"
#define REQUIRED_LEN MAX(sizeof(PARAM_FILE_NAME), sizeof(LOG_FILE_NAME))

K_MSGQ_DEFINE(charger_resp_msgq, sizeof(struct cmdHandler_type), MSG_DEPTH, 4);


static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

void initDisplay(void);

static int lsdir(const char *path);

/*
*  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
*  in ffconf.h
*/
static const char *disk_mount_pt = DISK_MOUNT_PT;



//Hardware Devices
static const struct pwm_dt_spec pwm_coil = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1)); //JML TODO: change alias to something more descriptive
static const struct i2c_dt_spec dev_i2c_lcd = I2C_DT_SPEC_GET(DT_NODELABEL(lcddisplay));
static const struct i2c_dt_spec dev_i2c_rtc = I2C_DT_SPEC_GET(DT_NODELABEL(ertc));
static const struct i2c_dt_spec dev_i2c_dcdc = I2C_DT_SPEC_GET(DT_NODELABEL(dcdc));
static const struct i2c_dt_spec dev_i2c_adc = I2C_DT_SPEC_GET(DT_NODELABEL(adctherm));
static const struct i2c_dt_spec dev_i2c_syspwr = I2C_DT_SPEC_GET(DT_NODELABEL(syspower));
static const struct i2c_dt_spec dev_i2c_cdpwr = I2C_DT_SPEC_GET(DT_NODELABEL(cdpower));
//Watchdog
static const struct device *const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel_id;

//Messaeg Queues
K_MSGQ_DEFINE(coil_req_msgq, sizeof(struct smartCoil_type), MSG_DEPTH, 4);
K_MSGQ_DEFINE(coil_resp_msgq, sizeof(struct smartCoil_type), MSG_DEPTH, 4);


#define COILPKT_HEADER_LEN 3
#define COILMIN_PKT_LEN COILPKT_HEADER_LEN // when checksum added, this will change
#define COILMAX_PKT_LEN  COIL_RX_BUF_SIZE-COILMIN_PKT_LEN
#define COILPKT_LEN_UNDEFINED 0 
#define COILUART_WAIT_FOR_RX 50 //in ms

#define DEFAULT_COIL_PERIOD 285714 //in ns (~3.5kHz)

#define COIL_RX_BUF_SIZE 64
#define COIL_TX_BUF_SIZE 64
uint8_t coil_rx_buf[COIL_RX_BUF_SIZE] = {0};
uint8_t coil_tx_buf[COIL_TX_BUF_SIZE] = {0};
static uint32_t coil_period = 0;


typedef struct 
{
	uint16_t mincurrent;
	uint16_t maxcurrent;
	uint16_t optimalfreq;
	
} CHARGERPARAMS;

static CHARGERPARAMS chargerParams = {
	1500, 
	1501,
	3500
};

static void uart_coil_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	

	static bool disable_req;

	static uint16_t len = 0;
	static uint8_t cmd = 0;
	static uint16_t pkt_len = COILPKT_LEN_UNDEFINED;

	static bool look_for_start = true;

	bool end_of_packet = false;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_INF("COIL UART_TX_DONE");
		break;

	case UART_RX_RDY:
		LOG_INF("offset %d, len %d, pkt_len %d, cmd %d", evt->data.rx.offset, len, pkt_len, cmd);
		LOG_HEXDUMP_INF(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len, "COIL UART_RX_RDY");

		if (look_for_start) {
			cmd = 0;
			len = 0;
			
			if(evt->data.rx.buf[0]==0xFF){
				LOG_INF("Found Packet Header");
				look_for_start = false;
			} 
			else {
				LOG_INF("Bad Packet Header");
				end_of_packet = true;
			}			
		} 

		if (!look_for_start){
			len += evt->data.rx.len;
			if(len >= COILMIN_PKT_LEN && pkt_len == COILPKT_LEN_UNDEFINED){
				
				cmd = evt->data.rx.buf[1];
				pkt_len = evt->data.rx.buf[2];

				if (pkt_len > COILMAX_PKT_LEN || pkt_len < COILMIN_PKT_LEN ){ //bad packet: pkt len too long or too short
					LOG_INF("Bad Packet Length");
					pkt_len = COILPKT_LEN_UNDEFINED;
					look_for_start = true;
					end_of_packet = true;
				}
			}
			if(len >= pkt_len && len >= COILMIN_PKT_LEN ){ //reached end of packet
				//TODO: add checksum here
				end_of_packet = true;
			}
		}
		if (disable_req) {
			return;
		}
		if(end_of_packet){
			uart_rx_disable(dev);
		}

		break;

	case UART_RX_DISABLED:
		LOG_INF("COIL UART_RX_DISABLED");
		//reinitialize stuff
		disable_req = false;
		pkt_len = COILPKT_LEN_UNDEFINED;
		look_for_start = true;

		uart_rx_enable(dev, coil_rx_buf, sizeof(coil_rx_buf), COILUART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_INF("COIL UART_RX_BUF_REQUEST");
		break;

	case UART_RX_BUF_RELEASED:
		LOG_INF("COIL UART_RX_BUF_RELEASED");

		struct smartCoil_type smartCoil; 
		smartCoil.len = len;
		memcpy(&smartCoil.buf[0],&evt->data.rx.buf[0],len);

		while(k_msgq_put(&coil_resp_msgq, &smartCoil, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CoilResp msg_q");
			k_msgq_purge(&coil_resp_msgq);
		}
		LOG_INF("CoilResponse k_msg_q_put");
	
		break;

	case UART_TX_ABORTED:
		LOG_INF("UART_TX_ABORTED");
		break;

	default:
		break;
	}
}

uint16_t convertThermistorData(uint16_t counts)
{

	THERMTBL const *p1, *p2;
	uint16_t tempr;
	
	for( p1 = thermTable; counts < p1->adsCounts  ; p1++ )
	{
		if( p1->degC >= 500 )
			break;
	}
	
	
	if( p1->degC <= 100 )
		tempr = 100;
	
	else if( p1->degC >= 500 )
		tempr = 500;
	
	else
	{
		p2 = p1 - 1;
		
		tempr = p1->degC - (10 * (counts - p1->adsCounts) / (p2->adsCounts - p1->adsCounts) );
		
	}
	
	return tempr;
}




#define MAX_COIL_RESPONSE_LEN 40



uint32_t setCoilPeriod(uint32_t period)
{

	//uint32_t period_ns = ((uint32_t)period) *1000;
	coil_period = period;
	//LOG_INF("Set Coil Period");

	int err = pwm_set_dt(&pwm_coil, period, period >> 1);
	if (err) {
		LOG_WRN("Error %d: failed to set PWM pulse width/period\n", err);
		return 0;
	}
	return period;
	

}

uint32_t getCoilPeriod(void)
{
	return coil_period;
}

//this is used for communication to and from the smartcoil
void coil_reqresp_thread(void)
{
	int err = 0;
    uint8_t checksum = 0;
    uint8_t i = 0;

	static const struct device *uartcoil = DEVICE_DT_GET(DT_NODELABEL(uart3)); //should this be global?

	LOG_INF("Starting coil request-response thread");
	err = uart_callback_set(uartcoil, uart_coil_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize coil UART callback: %d", err);
		return;	
	}
	uart_rx_enable(uartcoil, coil_rx_buf, sizeof(coil_rx_buf), COILUART_WAIT_FOR_RX);


	
	struct smartCoil_type smartCoil; 
	struct cmdHandler_type cmdHandler; 
	uint16_t timeout = 100;
	uint8_t source = 0;

	for (;;) {
		/* Wait indefinitely for request coming from BLE or UART or locally */
		err = k_msgq_get(&coil_req_msgq, &smartCoil, K_FOREVER); //wait indefinitely for a request from cmdhandler or elsewhere

		/* Add a checksum to the message and then send out */
        checksum = 0;               // Set checksum = 0
        for (i = 0; i < smartCoil.buf[2]; i++) {      // Calculate checksum before 2s compliment
            checksum = checksum + smartCoil.buf[i];
        }
        checksum =  (~checksum) + 1;                  // 2s compliment
        smartCoil.buf[i]=checksum;
		smartCoil.len = smartCoil.len+1;
		source = smartCoil.source;
		LOG_INF("coil UART send");

		uart_tx(uartcoil, &smartCoil.buf[0], smartCoil.len, SYS_FOREVER_MS);   


		/* Wait (not indefinitely) for response from coil*/
		err = k_msgq_get(&coil_resp_msgq, &smartCoil, K_MSEC(timeout));


		
		if (err!=0){
			//JML TODO: send an appropriate timeout response)
			LOG_INF("SmartCoil Timeout");
			smartCoil.len = 0;
		} 
		
		if (smartCoil.len > 1){ 

			cmdHandler.len = smartCoil.len -1; //remove CRC?
			
			//JML TODO?:Add error checking to confirm length does not exceed space at destination
			memcpy(&cmdHandler.buf[0], &smartCoil.buf[0], smartCoil.len-1); 
		}
		else{ 
			//timed out
			cmdHandler.len = 3;
			cmdHandler.buf[INDEX_RESP_SYNC] = SYNC_BYTE;
			cmdHandler.buf[INDEX_RESP_TYPE] = RESP_COIL_TIMEOUT;
			cmdHandler.buf[INDEX_RESP_LEN] = cmdHandler.len;
		}

		switch(source){
			case SOURCE_CMDHANDLER:
				while(k_msgq_put(&cmd_resp_msgq, &cmdHandler, K_NO_WAIT) != 0)
				{
					/* message queue is full: purge old data & try again */
					LOG_INF("Purging CmdResp msg_q");
					k_msgq_purge(&cmd_resp_msgq);
				}
				LOG_INF("CommandResponse k_msg_q_put complete");
				break;
			case SOURCE_CHARGER:
				LOG_INF("Not Yet implemented");
				break;
			default:
				LOG_INF("Undefined Source");

		}


		

		
	}
}

bool startWatchdog()
{
	struct wdt_timeout_cfg wdt_config = {
		/* Reset SoC when watchdog timer expires. */
		.flags = WDT_FLAG_RESET_SOC,

		/* Expire watchdog after max window */
		.window.min = 0,
		.window.max = 60000, //one minute
	};
	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install error\n");
		return false;
	}

	int err = wdt_setup(wdt, 0);
	if (err < 0) {
		LOG_ERR("Watchdog setup error %d\n", err);
		return false;
	}

	LOG_INF("Watchdog Started!");
	return true;

}

void disableWatchdog()
{
	int err;
	err = wdt_disable(wdt);
	if (err<0)
	{
		LOG_ERR("Watchdog disable error: %d\n", err);
	}
	else
	{
		LOG_INF("Watchdog Stopped!");
	}
}

void feedWatchdog()
{
	int err;
	err = wdt_feed(wdt, wdt_channel_id);
	if (err<0)
	{
		LOG_ERR("Watchdog feed error: %d\n", err);
	}
	else
	{
		LOG_INF("Watchdog Fed - must feed again within 60s!");
	}
}

void keepCoilOn()
{
	feedWatchdog();
}


uint16_t getThermistor(void)
{
	int ret;
	uint8_t write_buf[1];
	uint8_t read_buf[3];  //HB, LB, status
	write_buf[0] = 0x9C;
	
	
	LOG_INF("Get Thermistor");

	ret = i2c_write_dt(&dev_i2c_adc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_adc.addr);
		return 0;
	}
	k_sleep(K_MSEC(75)); 

	ret = i2c_read_dt(&dev_i2c_adc, read_buf,  sizeof(read_buf));
	if(ret != 0){
		LOG_INF("Failed to readI2C device address %x at reg \n\r", dev_i2c_adc.addr);
		return 0;
	}

	if(read_buf[2] != 0x1C)
	{
		LOG_INF("Thermistor A/D not configured correctly or delay inadequate\n\r");
	}

	return convertThermistorData( ( ((uint16_t)read_buf[0])<<8) + (uint16_t) read_buf[1] );
}

uint8_t getSysPower(int16_t* current, uint16_t* voltage)
{
	int ret;
	
	//LOG_INF("Get System Power Measurement");

	uint8_t write_buf[1];
	uint8_t read_buf[2]={0,0};
	
	if (current != NULL)
	{
		write_buf[0]=0x01;		
		ret = i2c_write_read_dt(&dev_i2c_syspwr, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
		if(ret != 0){
			LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_syspwr.addr);
			return 0;
		}
		//LOG_HEXDUMP_INF(read_buf, 2, "SYS SHUNT");
		*current = (int16_t)(  (((uint16_t)read_buf[0])<<8) + (uint16_t)read_buf[1]  )>>2; //in mA 2.5uV/LSB across 0.01ohm I=V/R 
		//LOG_INF("SYS current: %d", *current);
	}

	if (voltage != NULL)
	{
		write_buf[0]=0x02;
		ret = i2c_write_read_dt(&dev_i2c_syspwr, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
		if(ret != 0){
			LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_syspwr.addr);
			return 0;
		}
		//LOG_HEXDUMP_INF(read_buf, 2, "SYS BUS");
		*voltage = ((  (((uint16_t)read_buf[0])<<8) + (uint16_t)read_buf[1]  )<<3)/5; //in mV (x1.6)
		//LOG_INF("SYS voltage: %d", *voltage);
	}

	return 1;
}

void configCDPower(void)
{
	//uint8_t write_buf[3] = {0, 0x45, 0x27}; //16 sample average @1100us (17.6ms)
	uint8_t write_buf[3] = {0, 0x47, 0x27}; //64 sample average @1100us (70.4ms)
	int ret = i2c_write_dt(&dev_i2c_cdpwr, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_cdpwr.addr);
	}
}

uint8_t getCDPower(int16_t* current, uint16_t* voltage)
{
	int ret;
	
	//LOG_INF("Get CD Power Measurement");

	uint8_t write_buf[1];
	uint8_t read_buf[2]={0,0}; 

	
	if (current != NULL)
	{
		write_buf[0]=0x01;
		ret = i2c_write_read_dt(&dev_i2c_cdpwr, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
		if(ret != 0){
			LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_cdpwr.addr);
			return 0;
		}
		//LOG_HEXDUMP_INF(read_buf, 2, "CD SHUNT");
		*current = -(int16_t)(  (((uint16_t)read_buf[0])<<8) + (uint16_t)read_buf[1]  )>>2; //in mA 2.5uV/LSB across 0.01ohm I=V/R 
		//LOG_INF("CD current: %d", *current);
	}

	if (voltage != NULL)
	{
		write_buf[0]=0x02;
		ret = i2c_write_read_dt(&dev_i2c_cdpwr, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
		if(ret != 0){
			LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_cdpwr.addr);
			return 0;
		}
		//LOG_HEXDUMP_INF(read_buf, 2, "CD US");
		*voltage = ((  (((uint16_t)read_buf[0])<<8) + (uint16_t)read_buf[1]  )<<3)/5; //in mV (x1.6)
		//LOG_INF("CD voltage: %d", *voltage);
	}

	return 1;
}

void startCoilDrive(void)
{
	if(startWatchdog())
	{
		if (coil_period == 0)
		{
			coil_period = DEFAULT_COIL_PERIOD;
		}

		setCoilPeriod(coil_period); //start PWM
		enableCoilDrive(1);
		LOG_INF("Started COil Drive.  Watchdog must be fed to prevent reset");
	}
	else
	{
		LOG_INF("Could not start watchdog. Starting Coil Drive not allowed");
	}
}

void stopCoilDrive(void)
{
	enableCoilDrive(0);
	setCoilPeriod(0);  //stop PWM
	disableWatchdog(); 
}

//Fullscale range options for TPS55289 DCDC converter
#define DCDC_5V  0
#define DCDC_10V 1
#define DCDC_15V 2
#define DCDC_20V 3
#define DCDC_FS  DCDC_10V

#define MAX_COIL_VOLTAGE 9000
#define MIN_COIL_VOLTAGE 0

void initDCDC()
{
	int ret;
	uint8_t write_buf[2];
	write_buf[0] = 0x04;	//VOUT_FS register
	write_buf[1] = DCDC_FS;    

	LOG_INF("Init DCDC");

	ret = i2c_write_dt(&dev_i2c_dcdc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_dcdc.addr);
	}
}

void startDCDC()
{
	int ret;
	uint8_t write_buf[2];
	uint8_t read_buf[1];

	write_buf[0] = 0x04;	//VOUT_FS register
		
	//LOG_INF("Reading RTC");

	ret = i2c_write_read_dt(&dev_i2c_dcdc, write_buf, 1 ,read_buf, sizeof(read_buf));
	if(ret != 0){
		LOG_INF("Could not confirm VOUT_FS DCDC register: Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_rtc.addr);
		return;
	}
	if (read_buf[0] != DCDC_FS)
	{
		LOG_INF("VOUT_FS DCDC not as intended.  Won't start coil. Reads: %X", read_buf[0]) ;
		return;
	}

	write_buf[0] = 0x06;  //MODE register
	write_buf[1] = 0xB0;  //Enable Output, Enable Hiccup, Ennable Discharge

	LOG_INF("Start DCDC");

	ret = i2c_write_dt(&dev_i2c_dcdc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_dcdc.addr);
	}
}

void stopDCDC()
{
	int ret;
	uint8_t write_buf[2];
	write_buf[0] = 0x06; //MODE register
	write_buf[1] = 0x30; //Disable Output, Enable Hiccup, Ennable Discharge
	
	LOG_INF("Stop DCDC");

	ret = i2c_write_dt(&dev_i2c_dcdc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_dcdc.addr);
	}
}

/*voltage in mV.  depending on DCDC_FS, max attainable is 5000, 10000, 15000, or 20000.  Min attainable is 800mV */

void setDCDC(uint16_t voltage)
{

	uint16_t voltagesetting;
	
	if(voltage < 800)
	{
		voltage = 800;
	}
	if(voltage > (5000*(DCDC_FS+1)))
	{
		voltage = 5000*(DCDC_FS+1);
	}

	if(voltage > MAX_COIL_VOLTAGE)
	{
		voltage = MAX_COIL_VOLTAGE;
	}
	if(voltage < MIN_COIL_VOLTAGE)
	{
		voltage = MIN_COIL_VOLTAGE;
	}


	voltagesetting = (uint16_t) ( (  ((uint32_t)voltage) * (2256/(1+DCDC_FS)) - 450000  )/5645 );
	int ret;
	uint8_t write_buf[3];
	write_buf[0] = 0x00;  //REF register
	write_buf[1] = (uint8_t)(voltagesetting & 0xFF); //Low Byte
	write_buf[2] = (uint8_t)(voltagesetting >> 8); //High Byte


	LOG_INF("Set DCDC: %d", voltagesetting);

	ret = i2c_write_dt(&dev_i2c_dcdc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_dcdc.addr);
	}
}

//sec, min, hour, weekday, day, month, year
void readRTC(uint8_t* rtc)
{
	int ret;
	uint8_t write_buf[1]={0};
	uint8_t read_buf[7]={0,0,0,0,0,0,0};
	
	//LOG_INF("Reading RTC");

	ret = i2c_write_read_dt(&dev_i2c_rtc, write_buf, sizeof(write_buf),read_buf, sizeof(read_buf));
	if(ret != 0){
		LOG_INF("Failed to write/read to I2C device address %x at reg \n\r", dev_i2c_rtc.addr);
	}

	memcpy(rtc, read_buf, sizeof(read_buf));
}

uint8_t getSeconds( uint8_t* rtc )
{
	return ((rtc[0]&(BIT4|BIT5|BIT6))>>4)*10 + (rtc[0]&(BIT0|BIT1|BIT2|BIT3));
}
uint8_t getMinutes( uint8_t* rtc )
{
	return ((rtc[1]&(BIT4|BIT5|BIT6))>>4)*10 + (rtc[1]&(BIT0|BIT1|BIT2|BIT3));
}
uint8_t getHours( uint8_t* rtc )
{
	if(rtc[2]&BIT6)	//using AM/PM
	{
		//convert to 24h time
		return ((rtc[2]&BIT5)==BIT5)*12 + ((rtc[2]&BIT4)==BIT4)*10 + (rtc[2]&(BIT0|BIT1|BIT2|BIT3));
	}
	else //using 24H
	{
		return ((rtc[2]&(BIT4|BIT5))>>4)*10 + (rtc[2]&(BIT0|BIT1|BIT2|BIT3));
	}
}

uint8_t getDayOfWeek( uint8_t* rtc )
{
	return rtc[3]&(BIT0|BIT1|BIT2);
}

uint8_t getDate( uint8_t* rtc )
{
	return ((rtc[4]&(BIT4|BIT5))>>4)*10 + (rtc[4]&(BIT0|BIT1|BIT2|BIT3));
}

uint8_t getMonth( uint8_t* rtc )
{
	return ((rtc[5]&(BIT4))>>4)*10 + (rtc[5]&(BIT0|BIT1|BIT2|BIT3));
}

uint8_t getYear( uint8_t* rtc )
{
	return ((rtc[6]&(BIT4|BIT5|BIT6|BIT7))>>4)*10 + (rtc[6]&(BIT0|BIT1|BIT2|BIT3));
}

void writeRTC(uint8_t *rtc)
{
	
	int ret;
	uint8_t write_buf[8]={0,0,0,0,0,0,0,0}; //reg, sec, min, hour, weekday, day, month, year

	LOG_INF("Setting RTC");

	memcpy(&write_buf[1], rtc, 7);
	
	ret = i2c_write_dt(&dev_i2c_rtc, write_buf, sizeof(write_buf));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg \n\r", dev_i2c_rtc.addr);
	}
}


void writeDisplay(uint8_t reg, uint8_t val)
{
	int ret;
	uint8_t config[2];
	config[0]=reg;
	config[1]=val;
	ret = i2c_write_dt(&dev_i2c_lcd, config, sizeof(config));
	if(ret != 0){
		LOG_INF("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c_lcd.addr,config[0]);
	}
}

#define DISP_CMD 0x00
#define DISP_DATA 0x40

void OutputDisplay(bool clear, char* str1, char* str2, char* str3, char* str4, uint8_t len1, uint8_t len2, uint8_t len3, uint8_t len4)
{
    static bool isBusy = false;
	if(isBusy)
	{
		LOG_INF("Display busy");
		return;
	}

	isBusy = true;

	if(clear)
	{
		writeDisplay(DISP_CMD, 0x01); //clear display
	}
    if(len1>0){
   		OutputDisplayLine(1, str1, len1);
	}
	if(len2>0){
		OutputDisplayLine(2, str2, len2);
	}
	if(len3>0){
    	OutputDisplayLine(3, str3, len3);
	}
	if(len4>0){
    	OutputDisplayLine(4, str4, len4);
	}
	isBusy = false;
}

//This function may be called from different tasks and we want to guarantee mutual exclusion
//SHould this use a mutex instead of iSbuSy variable?
bool OutputDisplayLine(uint8_t line, char* str, uint8_t len)
{
	static bool isBusy = false;

	if(isBusy)
	{
		LOG_INF("Display busy");
		return false;
	}

	isBusy = true;
	//LOG_INF("Updating Display Line %d, len%d:", line, len);
	//LOG_HEXDUMP_INF(str, len, "Data");

    int i;
	if(line>4){
		line=4;
	}

	if(line==1){
    	writeDisplay(DISP_CMD, 0x02); //return home
	} else if (line==2){
		writeDisplay(DISP_CMD, 0xA0); //line 2
	} else if (line==3){
		writeDisplay(DISP_CMD, 0xC0); //line 3
	} else if (line==4){
		writeDisplay(DISP_CMD, 0xE0); //line 4
	}

	for(i=0;i<len;i++)    // Output line 1
	{
		writeDisplay(DISP_DATA, str[i]); //write character i
	}

	isBusy = false;
	return true;

}

//8 custom characters available
#define CG_RADIO     0
#define CG_BT		 0
#define CG_BATTERY   1
#define CG_BTPAIR	 1
#define CG_DEGREEC   2
#define CG_TUNER     2
#define CG_PERSON    3
#define CG_POWER     3
#define CG_LIGHTNING 4
#define CG_COIL      5
#define CG_DOUBLEI   5
#define CG_BIGSTAR   6
#define CG_LILSTAR   7
#define CG_NUM		 8

#define CG_BLANK 	0b00100000 
#define CG_MUSIC    0b10010000 
#define CG_TRIANGLE 0b00010000 

static uint8_t customCharCharge[8][8] = {CUSTOMCHAR_RADIO0, 
						   CUSTOMCHAR_BATTERY0,
						   CUSTOMCHAR_DEGREEC, 
						   CUSTOMCHAR_PERSON, 
						   CUSTOMCHAR_LIGHTNING,
						   CUSTOMCHAR_COIL,
						   CUSTOMCHAR_BIGSTAR,      //not currently used on this screen
						   CUSTOMCHAR_LILSTAR	};  //not currently used on this screen

static uint8_t customCharMenu[8][8] = {CUSTOMCHAR_BT, 
							CUSTOMCHAR_BTPAIR, 
							CUSTOMCHAR_TUNER,
							CUSTOMCHAR_POWER,
							CUSTOMCHAR_LIGHTNING,
							CUSTOMCHAR_DOUBLEI, 
							CUSTOMCHAR_BIGSTAR, 
							CUSTOMCHAR_LILSTAR}; 		


static const uint8_t customCharRadio[4][8] = { CUSTOMCHAR_RADIO0,
										CUSTOMCHAR_RADIO1, 
										CUSTOMCHAR_RADIO2,
										CUSTOMCHAR_RADIO3};

static const uint8_t customCharBattery[7][8] = {CUSTOMCHAR_BATTERY0, 
										CUSTOMCHAR_BATTERY1,
										CUSTOMCHAR_BATTERY2,										
										CUSTOMCHAR_BATTERY3,										
										CUSTOMCHAR_BATTERY4,
										CUSTOMCHAR_BATTERY5,
										CUSTOMCHAR_BATTERY6 };																			


void setMenuIcons()
{
	for (uint8_t  i = 0; i<CG_NUM; i++)
	{
		writeDisplay(DISP_CMD, 0x40 + i*8); //set CustomGraphinc Addr
		for (uint8_t j = 0; j<8; j++)
		{
			writeDisplay(DISP_DATA, customCharMenu[i][j]);
		}
	}
}


void setChargerIcons()
{
	for (uint8_t i = 0; i<CG_NUM; i++)
	{
		writeDisplay(DISP_CMD, 0x40 + i*8); //set CustomGraphinc Addr
		for (uint8_t j = 0; j<8; j++)
		{
			writeDisplay(DISP_DATA, customCharCharge[i][j]);
		}
	}
}

void setRadioIcon(uint8_t val)
{
	LOG_INF("setting new radio icon %d", val);
	writeDisplay(DISP_CMD, 0x40 + CG_RADIO*8); //set CustomGraphinc Addr
	for (uint8_t j = 0; j<8; j++)
	{
		writeDisplay(DISP_DATA, customCharRadio[val][j]);
	}
}

void setBatteryIcon(uint8_t val)
{
	LOG_INF("setting new battery icon %d", val);
	writeDisplay(DISP_CMD, 0x40 + CG_BATTERY*8); //set CustomGraphinc Addr
	for (uint8_t j = 0; j<8; j++)
	{
		writeDisplay(DISP_DATA, customCharBattery[val][j]);
	}
}

void initDisplay(void)
{
 // NHD-0420CW-AB3
//JML WORKAROUND for lower I2C rate
	((NRF_TWIM_Type *) NRF_TWIM2)->FREQUENCY = 0x00CCCCCCUL; // 50000 * 2^32 / 16000000

	k_msleep(1); //wait at least 10us
	enableDisplay(1);

	if (!device_is_ready(dev_i2c_lcd.bus)) {
		LOG_INF("I2C bus %s is not ready!\n\r",dev_i2c_lcd.bus->name);
		return;
	}

	// Code based on Newhaven Display NHD-0220CW-AB3 Example Program Code
	// Modified for I2C communications
    writeDisplay(DISP_CMD, 0x2A); //function set (extended command set)
    writeDisplay(DISP_CMD, 0x71); //function selection A
    writeDisplay(DISP_DATA, 0x00); // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
    writeDisplay(DISP_CMD, 0x28); //function set (fundamental command set)
    writeDisplay(DISP_CMD, 0x08); //display off, cursor off, blink off
    writeDisplay(DISP_CMD, 0x2A); //function set (extended command set)
    writeDisplay(DISP_CMD, 0x79); //OLED command set enabled
    writeDisplay(DISP_CMD, 0xD5); //set display clock divide ratio/oscillator frequency
    writeDisplay(DISP_CMD, 0x70); //set display clock divide ratio/oscillator frequency
    writeDisplay(DISP_CMD, 0x78); //OLED command set disabled
    writeDisplay(DISP_CMD, 0x09); //extended function set (4-lines)
    writeDisplay(DISP_CMD, 0x06); //COM SEG direction
    writeDisplay(DISP_CMD, 0x72); //function selection B
    writeDisplay(DISP_DATA, 0x00); //ROM CGRAM selection
    writeDisplay(DISP_CMD, 0x2A); //function set (extended command set)
    writeDisplay(DISP_CMD, 0x79); //OLED command set enabled
    writeDisplay(DISP_CMD, 0xDA); //set SEG pins hardware configuration
    writeDisplay(DISP_CMD, 0x10); //set SEG pins hardware configuration
    writeDisplay(DISP_CMD, 0xDC); //function selection C
    writeDisplay(DISP_CMD, 0x00); //function selection C
    writeDisplay(DISP_CMD, 0x81); //set contrast control
    writeDisplay(DISP_CMD, 0x7F); //set contrast control
    writeDisplay(DISP_CMD, 0xD9); //set phase length
    writeDisplay(DISP_CMD, 0xF1); //set phase length
    writeDisplay(DISP_CMD, 0xDB); //set VCOMH deselect level
    writeDisplay(DISP_CMD, 0x40); //set VCOMH deselect level
    writeDisplay(DISP_CMD, 0x78); //OLED command set disabled
    writeDisplay(DISP_CMD, 0x28); //function set (fundamental command set)
    writeDisplay(DISP_CMD, 0x01); //clear display
    writeDisplay(DISP_CMD, 0x80); //set DDRAM address to 0x00
    writeDisplay(DISP_CMD, 0x0C); //display ON

	setMenuIcons();
}




static uint8_t chargerMode = CHARGER_MODE_DEFAULT; //Only the setChargeMode function should set this variable

uint8_t getChargeMode()
{
	return chargerMode;
}

uint8_t setChargeMode(uint8_t mode)
{
	
	//if the requested mode is the same as the current mode, exit current mode, go to default
	if(mode == chargerMode || mode > CHARGER_MODE_MAX)
	{
		chargerMode = CHARGER_MODE_DEFAULT;
	}
	else 
	{
		chargerMode = mode;
	}
	
	
	return chargerMode;
}


void charger_thread(void)
{
	char str[80];
	uint8_t cnt;
	uint8_t rtc[7] = {0,0,0,0,0,0,0}; //sec, min, hour, weekday, day, month, year 
	static uint8_t previousMode = 0;
	uint8_t currentMode = 0;
	//static uint8_t bondcount = 0;

	char chardayofweek1[7]	= {'S', 'M', 'T', 'W', 'T', 'F', 'S'};
	char chardayofweek2[7]	= {'u', 'o', 'u', 'e', 'h', 'r', 'a'};

	if (!device_is_ready(wdt)) {
		printk("Watchdog device not ready.\n");
		return;
	}



	if (!device_is_ready(pwm_coil.dev)) {
	LOG_ERR("Error: PWM device %s is not ready. No coil pwm supported\n",
			pwm_coil.dev->name);
			return;
	}

	configCDPower(); //set long average on coil drive power calculations (~70ms)

	initDisplay();  //also confirms that I2C bus is available and ready
	initDCDC();
	initSD();
	//uint8_t buf[6];
	if(saved_settings_read(COIL_SETTINGS_ID, &chargerParams, sizeof(chargerParams)) > 0)
	{
		//replace with read from flash, and then update Settings
		// memcpy(chargerParams.mincurrent, &buf[0], 2);
		// memcpy(chargerParams.maxcurrent, &buf[2], 2);
		// memcpy(chargerParams.optimalfreq, &buf[4], 2);

	} else{
		LOG_INF("No Coil settings found in flash");
		// memcpy( &buf[0],chargerParams.mincurrent, 2);
		// memcpy( &buf[2],chargerParams.maxcurrent, 2);
		// memcpy( &buf[4],chargerParams.optimalfreq, 2);
		//if(saved_settings_write(COIL_SETTINGS_ID, buf, sizeof(buf)) > 0)
		if(saved_settings_write(COIL_SETTINGS_ID,  &chargerParams, sizeof(chargerParams)) > 0)
		{
			LOG_INF("Default Coil settings saved to flash");
		}
		else
		{
			LOG_WRN("Could not save Coil settings to flash");
		}
	}

	for(;;)
	{
		//LOG_INF("ChargerMode %d", chargerMode);
		//each option in this loop must have a sleep that allows the change in chargerMode to be handled
		//the sleep time should be < 1s such that button presses likely trigger a change do not have a 
		//long latency before change takes effect

		if(chargerMode == CHARGER_MODE_CLOCK)
		{
			setMenuIcons();

			if(modeLED & LED_CHARGER) setLEDs(0,0,0);
			sprintf(str,"  %1c       %1c%1c     %1c  "\
						"  %1c    COSM%1cC    %1c  "\
						" Smart Charger v%03d "\
						"                    ",
						CG_LIGHTNING, CG_BIGSTAR, CG_LILSTAR, CG_MUSIC,
						CG_POWER, CG_DOUBLEI, CG_TUNER,
						 SW_REV );
			OutputDisplay(1, str,"","","",80,0,0,0 );

			while(chargerMode == CHARGER_MODE_CLOCK)
			{
				//LOG_INF("ChargerMode Clock Loop");
				readRTC(rtc);


				sprintf(str, " %02X/%02X/%02X  %02X:%02X:%02X ", 
								rtc[5], /*month*/
								rtc[4], /*date*/
								rtc[6], /*year*/
								rtc[2], /*hour*/
								rtc[1], /*minute*/
								rtc[0] /*second*/ );
				OutputDisplayLine(4, str, 20 );

				currentMode = getBLEMode();
				if(currentMode == BLE_MODE_READY)
				{ 
					if(previousMode != BLE_MODE_READY)
					{
						//add bluetooth symbol only if we haven't done it yet
						sprintf(str,"  %1c     %1c %1c%1c     %1c  ",	CG_LIGHTNING, CG_BT, CG_BIGSTAR, CG_LILSTAR, CG_MUSIC);
						OutputDisplayLine(1, str, 20 );
					}
					k_msleep(1000);
				}
				else if(currentMode == BLE_MODE_ADVERT || currentMode == BLE_MODE_OPEN_ADVERT)
				{	
					sprintf(str,"  %1c     %1c %1c%1c     %1c  ",	CG_LIGHTNING, CG_BT, CG_BIGSTAR, CG_LILSTAR, CG_MUSIC);
					OutputDisplayLine(1, str, 20 );
					if(currentMode == BLE_MODE_ADVERT){
						k_msleep(500);
					} else {
						k_msleep(100); //blink fast for open advertising (pairing)
					}
					sprintf(str,"  %1c       %1c%1c     %1c  ", CG_LIGHTNING, CG_BIGSTAR, CG_LILSTAR, CG_MUSIC);
					OutputDisplayLine(1, str, 20 );
					if(currentMode == BLE_MODE_ADVERT){
						k_msleep(500);
					} else {
						k_msleep(100); //blink fast for open advertising (pairing)
					}
				} 
				else if(previousMode == BLE_MODE_READY)
				{ 
					//remove bluetooth symbol only if we haven't done it yet
					sprintf(str,"  %1c       %1c%1c     %1c  ",	CG_LIGHTNING, CG_BIGSTAR, CG_LILSTAR, CG_MUSIC);
					OutputDisplayLine(1, str, 20 );
					k_msleep(1000);
				}
				else {
					k_msleep(1000);
				}
				previousMode = currentMode;
			}
			
		}
		else if (chargerMode == CHARGER_MODE_CHARGE || chargerMode == CHARGER_MODE_CHARGE_NOFEEDBACK)
		{
			setChargerIcons();
			charge();
			//if there was an error, we want to leave error screen until acknowledged, so don't change mode
		}
		else if(chargerMode == CHARGER_MODE_DETECT)
		{
			metalDetect();
			setChargeMode(CHARGER_MODE_DEFAULT);
			LOG_INF("**** finished Metal Detect ******");
		}
		else if(chargerMode == CHARGER_MODE_TUNE)
		{
			setChargeMode(CHARGER_MODE_NONE);
			OutputDisplay(1, "","     TUNE COIL?     ","  Long Press again  ","     to confirm     ",0,20,20,0 );
			cnt = 0;
			while(chargerMode == CHARGER_MODE_NONE && cnt++ < 5) //wait up to 5s for completion of follow up long press
			{
				k_msleep(1000);
			}
			if (chargerMode == CHARGER_MODE_TUNE)
			{
				tuneCoil();
			}
			setChargeMode(CHARGER_MODE_DEFAULT);
		}
		else if (chargerMode == CHARGER_MODE_POWERDOWN)
		{
			setChargeMode(CHARGER_MODE_NONE);
			OutputDisplay(1, ""," POWER DOWN IMPLANT?","  Long Press again  ","     to confirm     ",0,20,20,0 );
			cnt = 0;
			while(chargerMode == CHARGER_MODE_NONE && cnt++ < 5) //wait up to 5s for completion of follow up long press
			{
				k_msleep(1000);
			}
			if (chargerMode == CHARGER_MODE_POWERDOWN)
			{
				powerDownPM();
			}
			setChargeMode(CHARGER_MODE_DEFAULT);
		}
		else if (chargerMode == CHARGER_MODE_DISPLAY_PASSKEY)
		{
			sprintf(str, "(#%1d) passkey:%06d ", get_bonded_devices(NULL)+1, getpasskey());		
			//sprintf(str, "    passkey:%06d    ", getpasskey());		
			OutputDisplayLine(4,str,20);	

			//wait until pairing completed, failed, or cancelled
			while(getpasskey()!=0)
			{
				k_msleep(1000);
			}
			setChargeMode(CHARGER_MODE_DEFAULT);
			
		}
		else if (chargerMode == CHARGER_MODE_DISPLAY_ERASEBONDS)
		{	
			sprintf(str, " Pairing Data Erased");		
			OutputDisplayLine(4,str,20);	
			k_msleep(1000);
			k_msleep(1000);
			setChargeMode(CHARGER_MODE_DEFAULT);
		}
		else
		{
			k_msleep(1000);
		}	
		
	}

}

void initSD(void)
{
	int res;
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = DISK_DRIVE_NAME;
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("Unable to get sector size");
			break;
		}
		LOG_INF("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		LOG_INF("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	mp.mnt_point = disk_mount_pt;

	res = fs_unmount(&mp);
	LOG_INF("Disk unmounted %d", res);

	res = fs_mount(&mp);
	
	if (res == FR_OK) {
		LOG_INF("Disk mounted.\n");
			lsdir(disk_mount_pt); 

		} else {
		LOG_INF("Error mounting disk.\n");
	}
	

}



bool logToFile(char * data, uint8_t len)
{
	struct fs_file_t file;
	char path[] =  "/SD:/logfile.dat";
	int err = 0;

	LOG_INF("Opening file");
	
	fs_file_t_init(&file);	
	err = fs_open(&file, path, FS_O_CREATE|FS_O_APPEND|FS_O_RDWR);
	if (err != 0)
	{
		LOG_ERR("Failed to open file %s err:%d", path, err);
		return false;
	}
	LOG_INF("Writing to file");
	fs_write(&file,data,len);
	LOG_INF("Closing file");
	fs_close(&file);
	LOG_INF("Done");
	return true;
}

uint32_t getLogLength()
{
	char path[] =  "/SD:";
	char filename[] = "logfile.dat";
	uint32_t len = 0;
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_INF("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	res = fs_readdir(&dirp, &entry);

	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_FILE && strcmp(entry.name, filename)==0) {
			len = entry.size;
			break;
		}
	}


	/* Verify fs_closedir() */
	res = fs_closedir(&dirp);

	return len;

}

bool readFromFile(char * data, uint8_t len,  uint32_t off)
{
	struct fs_file_t file;
	char path[] =  "/SD:/logfile.dat";
	int err = 0;

	LOG_INF("Opening file");
	fs_file_t_init(&file);
	err = fs_open(&file, path, FS_O_READ);
	if(err != 0) {
		LOG_ERR("Failed to open file for reading %s err:%d", path, err);
		return false;
	}
	LOG_INF("Reading from file");
	fs_seek(&file, off, FS_SEEK_SET); //read from offset relative to beginning of file
	fs_read(&file,data,len);
	LOG_INF("Closing file");
	fs_close(&file);
	LOG_INF("Done");
	return true;

}

bool flushLog(void)
{
	int err;
	char path[] =  "/SD:/logfile.dat";
	err = fs_unlink(path);
	return err==0;
}

void dirSD(void)
{
	lsdir(disk_mount_pt);
}


/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_INF("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	LOG_INF("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			LOG_INF("[DIR ] %s\n", entry.name);
		} else {
			LOG_INF("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		count++;
	}

	/* Verify fs_closedir() */
	res = fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}

	return res;
}

//JML Note: if this is set to leave coil on for longer than watchdog reset time, Charger will reset since it does not feed the watchdog
void tuneCoil(void)
{
	uint16_t voltageSetting = 5000; //5V
	uint16_t min_freq = 3450;
	uint16_t max_freq = 3550;
	uint16_t opt_freq = 0; //in Hz
	uint16_t freq = min_freq;
	uint16_t max_current = 0; //in mA
	uint16_t current = 0; //in mA
	uint32_t period;
	uint16_t opt_period = 0;
	char str[21];
	uint8_t n = 0;



	setDCDC(voltageSetting);
	startDCDC();
	startCoilDrive();
	if(modeLED & LED_CHARGER) setLEDs(2,1,2);
	OutputDisplay(1, "Scanning Coil Freq: ", "","   Keep coil away   ","from implant/metals ",20,0,20,20 );
	k_msleep(200);
	period = 1000000000/freq; //in ns
	setCoilPeriod(period);
	while(freq <= max_freq)
	{
		if(chargerMode != CHARGER_MODE_TUNE)
		{
			break;
		}


		k_msleep(100);


		if(getCDPower(&current, NULL))
		{
			n = sprintf(str, " %4d Hz  %4d mA ", freq, current);
			OutputDisplayLine(2, str, n);
		}
		else
		{
			LOG_INF("Failed to get CD Power");
		}


		LOG_INF("Current:%d", current);
		if(current > max_current)
		{
			max_current = current;
			opt_freq = freq;
			opt_period = period;
		}
		freq++;
		period = 1000000000/freq; //in us
		setCoilPeriod(period);
		

	}
	
	if(freq > max_freq)
	{
		n = sprintf(str, " %4d Hz  %4d mA ", opt_freq, max_current);
		
		OutputDisplay(1, " Optimal Coil Freq: ", str,"","",20,n,0,0 );
		if (opt_freq != chargerParams.optimalfreq || max_current != chargerParams.maxcurrent)
		{
			chargerParams.optimalfreq = opt_freq;
			chargerParams.maxcurrent = max_current;
			chargerParams.mincurrent = max_current; //reset min current value
			saved_settings_write(COIL_SETTINGS_ID, &chargerParams, sizeof(chargerParams));
			OutputDisplay(1, " Optimal Coil Freq: ", str,"","",20,n,0,0 );
		}

		if(opt_period > 0)
		{
			setCoilPeriod(opt_period);
		}
		if(modeLED & LED_CHARGER) setLEDs(2,0,2);
		stopDCDC();
		stopCoilDrive();

		//pause on current screen.  Leave error screen until charging mode is changed
		do {
			k_msleep(1000);
		} while(chargerMode == CHARGER_MODE_TUNE); 
	}
	else
	{
		//quit early
		n = sprintf(str, " Coil Freq: %4d Hz ", chargerParams.optimalfreq);
		OutputDisplay(1, "    Quit Tuning     ", " No changes applied ",str,"",20,20,n,0 );
		if(modeLED & LED_CHARGER) setLEDs(2,0,2);
		stopDCDC();
		stopCoilDrive();
		k_msleep(1000);
	}
}


//JML Note: if this is set to leave coil on for longer than watchdog reset time, Charger will reset since it does not feed the watchdog
#define METAL_DETECT_TIME 30 //in s
#define DATA_INDEX 8

void metalDetect(void)
{

	uint16_t voltageSetting = 5000; //4V
	int16_t max_current = chargerParams.maxcurrent; //in mA
	int16_t current = 0; //in mA
	uint16_t voltage = 0; //in mV
	int16_t min_current = 9999; //in mA
	int16_t thresh_current = 0;
	


	uint64_t timeCheck;
	char str[21];
	bool commSuccess = true;;

	uint8_t data = 0;
	uint8_t target = 0;
	
	int32_t t = 0, d;

	if (chargerParams.mincurrent + 5   < chargerParams.maxcurrent) 
	{
		thresh_current = chargerParams.mincurrent + 5; //threshold for constant sound
	}

	
	//read charge target, then set it to 0
	if(ReadSDO(7,0x3000, 0x0B, 1, &target )==1) //SDO read 7:3000:0B, should return 1 (length)
	{
		LOG_INF("Target: %d", target);
		if(WriteSDO(7,0x3000, 0x0B, 1, &data, 1)) //SDO write 7:3000:0B, resp is uint8_t (0 if successful)
		{	
			//set charge target on PM successful	
			OutputDisplay(1, "   Implant found    ", "  Charging Disabled ","   Successfully     ","                    ",20,20,20,20 ); 
			commSuccess = true;
		}
	}	
	else
	{
		OutputDisplay(1, " Implant not found  ", "  Try Metal Detect  ","      Again         ","                    ",20,20,20,20 ); 
	}

	setCoilPeriod(1000000000/chargerParams.optimalfreq);  
	setDCDC(voltageSetting);
	startDCDC();
	startCoilDrive(); 
	k_msleep(1000);
	OutputDisplay(0, "     Target:        ", "   Current:         ","   Minimum:         ","   Maximum:         ",20,20,20,20 );

	sprintf(str, "   Target:%6d mA ", thresh_current);
	OutputDisplayLine(1, str, 20);
	sprintf(str, "  Maximum:%6d mA ", max_current);
	OutputDisplayLine(4, str, 20);

	LOG_INF("Failed to get CD Power");

	timeCheck = k_uptime_get();
	k_msleep(100);
	while((k_uptime_get()-timeCheck)/1000 < METAL_DETECT_TIME)
	{
		if(chargerMode != CHARGER_MODE_DETECT)
		{
			break;
		}

		if(getCDPower(&current, &voltage))
		{
			sprintf(str, "  Current:%6d mA ", current);
			OutputDisplayLine(2, str, 20);
			LOG_INF("Current:%d", current);
			
			if(current > max_current)
			{
				max_current = current;
				sprintf(str, "  Maximum:%6d mA ", max_current);
				OutputDisplayLine(4, str, 20);
			}
			if(current > 0  && current < min_current)
			{
				min_current = current;
				sprintf(str, "  Minimum:%6d mA ", min_current);
				OutputDisplayLine(3, str, 20);
			}
			setBuzzerPeriod(1912, 0);
			if(modeLED & LED_CHARGER) {setLEDs(2,1,2);}
			k_msleep(20);
			
			t = current-thresh_current;

			if(max_current <= thresh_current)
			{
				thresh_current = max_current -1; 
			}

			d = max_current-thresh_current;
			
			if(t<0) { t=0;}
			t = t*t*1000/(d*d); 
			if (t>1000) {t=1000;}
			

			if(t>0)
			{
				if(modeLED & LED_CHARGER) {setLEDs(2,0,2);}
				setBuzzerPeriod(0, 0);
				k_msleep(t);
			}
			
			
		}
		else
		{
			LOG_INF("Failed to get CD Power");
		}

		LOG_INF("METAL DETECT");
		
		k_msleep(20);
	}
	setBuzzerPeriod(0, 0);
	if(modeLED & LED_CHARGER) {setLEDs(2,0,2);}
	if(thresh_current ==0 && min_current < chargerParams.mincurrent && min_current>0 && min_current<9999)
	{
		LOG_INF("Saving new min current");
		chargerParams.mincurrent = (uint16_t) min_current;
		saved_settings_write(COIL_SETTINGS_ID, &chargerParams, sizeof(chargerParams));
	}

	if(commSuccess)
	{
		if(target <= 100)
		{
			if(WriteSDO(7,0x3000, 0x0B, 1, &target, 1)) //SDO write 7:3000:0B, resp is uint8_t (0 if successful)
			{	
				//set charge target on PM successful	
				LOG_INF("Charging Target Reconfig");
			}
			else
			{
				LOG_INF("Failed to reconfig charging Target");
			}
		}

	}
	stopDCDC();
	stopCoilDrive();
}

uint8_t getChargerParams(uint8_t* params)
{
	memcpy(params, &chargerParams, sizeof(chargerParams));
	return sizeof(chargerParams);
}

void setChargerParams(uint8_t* params)
{
	memcpy(&chargerParams, params, sizeof(chargerParams));
}


#define MAX_SYSTEM_CURRENT  2500 //in mA (sytem has 3A fuse)
#define MAX_COILDRIVE_CURRENT 3500 //in mA                    8V*3.5A = 28W < 30W = 12V*2.5A

#define COIL_OPEN_TEMP	100
#define COIL_GOOD_TEMP	400
#define COIL_WARN_TEMP  420
#define COIL_MAX_TEMP	440

#define PM_OPEN_TEMP	100
#define PM_GOOD_TEMP	370
#define PM_WARN_TEMP    380
#define PM_MAX_TEMP		400

#define TIME_MIN_TARGET_CHANGE 5 //in s
#define TARGET_INCREMENT 5
#define TARGET_MAX      100
#define TARGET_MIN      20

#define TIME_OUT_APP       60 //in s  (max time to get into app mode, coil at 5V )
#define TIME_OUT_RADIO     20 //in s  (max time between successful radio commands, coil may be higher)
#define TIME_BOOT_TO_APP    2 //in s  (time to get between boot and app...radio commands expected to fail)
#define TIME_OUT_COUPLING  120 //in s (max time with insufficient coupling (charging command = 0))
#define TIME_OUT_COILTEMP   5 //in s  (max time with invalid external coil temperature)
#define TIME_TO_BOOT	   16 //in s  (expected time for PM to get through boot to app)
#define TIME_WARN_RADIO     5 //in s 
#define TIME_WARN_COUPLING  5// in s

#define CHARGING_ERROR_NONE         0
#define CHARGING_ERROR_RADIO        1 //timed out on radio but did communicate with app
#define CHARGING_ERROR_STALE        2 //timed out on because PM wasn't updating
#define CHARGING_ERROR_NO_PM        3 //timed out waiting, never communicated to PM
#define CHARGING_ERROR_NO_APP       4 //timed out waiting to get into app mode, but communicated in Boot mode
#define CHARGING_ERROR_PMTEMP       5
#define CHARGING_ERROR_PMTEMP_OPEN  6
#define CHARGING_ERROR_CHARGERTEMP  7
#define CHARGING_ERROR_CHARGERTEMP_OPEN  8
// #define CHARGING_ERROR_COILTEMP1    8
// #define CHARGING_ERROR_COILTEMP2    9
#define CHARGING_ERROR_COUPLING     10
#define CHARGING_ERROR_SYSCURRENT   11
#define CHARGING_ERROR_CDCURRENT    12
#define CHARGING_ERROR_CDVOLTAGE    13


#define PLAYSOUND_NONE       0
#define PLAYSOUND_COUPLE     1
#define PLAYSOUND_UNCOUPLE   2
#define PLAYSOUND_ERROR      3
#define PLAYSOUND_CHARGEFULL 4

#define UNCOUPLED    0
#define COUPLED      1
#define FULLYCOUPLED 2

#define VREC_MIN_STEADY   125  //avoid VREC below 10V while steady
#define VREC_MIN_STEPPING 185  //avoid VREC below 15V while stepping.  This can be lower for a deep implant, but needs to be higher for shallow implant
#define VREC_MAX_STEADY   135  //avoid VREC above 13V while steady
#define VREC_MAX_STEPPING 195  //avoid VREC above 18V while stepping
#define VREC_MAX_QUICK    210  //avoid VREC above 20V ever

#define DISPLAY_NONE 0
#define DISPLAY_WAITINGFORBOOT 1
#define DISPLAY_BOOT_RADIO_MATCH 2
#define DISPLAY_BOOT_RADIO_MISMATCH 3
#define DISPLAY_PMFEEDBACK 4
#define DISPLAY_RADIO_WARN 5
#define DISPLAY_SERIAL_REV 6
#define DISPLAY_SET_CLOCK  7
#define DISPLAY_NO_FEEDBACK 8

#define MAX_COIL_VOLTAGE_NOFEEDBACK 6500 //in mV

#define TIME_CYLCE_LED_ON  250
#define TIME_CYLCE_LED_OFF 250

//action 


// Every cycle, external coil and charger stats are checked
// If Implant Feedback is on (required for regular user charging):
//  Set Clock (Halt RTC, Set DateTime OD Indices, Set/Start RTC)
//  Read target charge rate
//  Get Charging array
// Assume that PM may be in bootloader mode when charging is first started.  Don't trigger a radio warning/error until the 
// boot time has passed  or until after the first radio response has been recieved

void setIconRSSI(bool radio)
{
	static uint8_t previous = 0xFF;
	uint8_t bars;
	int8_t rssi;

	if (radio)
	{ 
		rssi = getLastRSSI();
		LOG_INF("RSSI%d", rssi);
		if (rssi<-70){
			bars = 1;//0xD4; //1 bar
		} else if (rssi<-60){
			bars =  2;//0xD3; //2 bars
		} else {
			bars = 3;//0xD2; //3 bars
		} 
	}
	else
	{
		bars = 0;
	}
	if (bars != previous)
	{
		setRadioIcon(bars);
		previous = bars;
	}
}

void setIconVoltage(uint16_t voltage)
{
	static uint8_t previous = 0xFF;
	uint8_t level;

	if (voltage < 3200){
		level = 0;
	} else if(voltage < 3400){
		level = 1;
	} else if(voltage < 3600){
		level = 2;
	} else if(voltage < 3800){
		level = 3;
	} else if(voltage < 3950){
		level = 3;
	} else if(voltage < 4050){
		level = 5;
	} else {
		level = 6;
	}

	if (level != previous)
	{
		setBatteryIcon(level);
		previous = level;
	}
}

void charge(void)
{

	uint16_t voltageSetting = 5000;  //5V
	uint16_t minVoltageSetting = 4000; //4V
	uint16_t maxVoltageSetting = 8000; //8V
	uint16_t stepVoltage = 10; //0.01V 10mV minimum
	int8_t delta;

	
	
	uint16_t alertSoundNotes[6] = {2273, 9092, 2273, 9092, 2273, 9092};
	uint16_t alertSoundTimes[6] = {200, 200, 200, 200, 200, 200};
	
	uint16_t coupleSoundNotes[6] = {1911, 1517, 1275, 956, 758, 478};
	uint16_t coupleSoundTimes[6] = {200, 150, 125, 100, 50, 50};

	uint16_t uncoupleSoundNotes[6] = {478, 758, 956, 1275, 1517, 1911};
	uint16_t uncoupleSoundTimes[6] = {50, 50, 100, 125, 150, 200};

	uint8_t playsound = PLAYSOUND_NONE;


	
	char str[81]; //leave space for null character
	char charWarnCoilTemp; 
	char charWarnPMTemp;
	char charWarnCoupling;
	char charWarnCoilV;
	uint8_t target = 0xFF; 
	uint8_t disptarg = 0;
	uint8_t buf[8];
	uint16_t previous_test;

	uint16_t serialPM = 0, revPM = 0; //serialPM becomes nonzero when successfully queried from PM App
	uint8_t configPM = 0; 

	uint16_t B1V = 0, B2V = 0, B3V = 0, BavgV = 0, vrec = 0, tempPM = 0, test = 0, NCL = 0 ;
	int16_t B1I = 0, B2I = 0, B3I = 0, BavgI = 0;
	uint8_t cmd = 0;
	uint8_t status = UNCOUPLED;
	
	
	uint16_t delayCycle;
	uint64_t timeCycle;
	uint64_t timeCheckAppRadio, timeCheckBootRadio, timeCheckCoupling, timeCheckTargetChange, timeCheckStart, timeCheckBootloader, timeCheckCoil;
	uint32_t timeSince;
	uint16_t vrecMin = VREC_MIN_STEPPING; 
	uint16_t vrecMax = VREC_MAX_STEPPING;

	uint8_t resp[21];
	uint8_t cntTempPM = 0;

	uint16_t currentCD = 0, voltageCD = 0, currentSys = 0, voltageSys = 0,tempCD = 0;

	uint8_t chargingError = CHARGING_ERROR_NONE;
	bool fullUpdate = false;
	
	
	struct audioList_type audioList;
	uint8_t displayMode = DISPLAY_NONE;

	uint8_t n;
	char logStr[120];

	uint8_t rtc[7] = {0,0,0,0,0,0,0}; //sec, min, hour, weekday, day, month, year in BCD format        

	//Turn on the Coil
	setCoilPeriod(1000000000/chargerParams.optimalfreq);
	setDCDC(voltageSetting);
	startDCDC();
	startCoilDrive();

	timeCheckStart = k_uptime_get();
	timeCheckCoupling = timeCheckStart; //time since last good coupling or Start
	
	timeCheckBootRadio = timeCheckStart; //time since last message with PM Boot or Start
	timeCheckTargetChange = timeCheckStart; //time since last changing of charging target or Start
	timeCheckCoil = timeCheckStart; //time since last valid Coil temperature check or Start
	timeCheckBootloader = 0; //time since first successful sequential messages with PM Boot 
	timeCheckAppRadio = 0; //time since last message with PM App 
	//startWatchdog();

	bool madeChangeLastCycle = true;

	OutputDisplay(1, "","","","",0,0,0,0 ); //Clear Display
	
	uint8_t radioIndicator;
	uint8_t addrAP = 0, addrPM = 0, chan = 0, power = 0, bootradio = 0;
	bool hasCommunicated = false; 
	
	setIconVoltage(0); 
	setIconRSSI(0);

	while(chargerMode == CHARGER_MODE_CHARGE || chargerMode == CHARGER_MODE_CHARGE_NOFEEDBACK )
	{
		radioIndicator = 0;
		readRTC(rtc); //read the realtime clock in order to set PM time and for logging

		charWarnCoilTemp = CG_BLANK; //space
	    charWarnPMTemp   = CG_BLANK;
	    charWarnCoupling = CG_BLANK;
		charWarnCoilV    = CG_BLANK;

		if(modeLED & LED_CHARGER) setLEDs(0,1,0);  //set green LED to indicate coil is on, reset warn/error LED
		timeCycle = k_uptime_get();

		//Get System Data
		if(getSysPower(&currentSys, &voltageSys))
		{
			LOG_INF("System Power: %d mA, %d mV", currentSys, voltageSys);
			if(currentSys > MAX_SYSTEM_CURRENT)
			{
				chargingError = CHARGING_ERROR_SYSCURRENT;
				break;
			}
		}
		else
		{
			LOG_INF("Failed to get System Power");
		}

		//Get Coil Data
		if(getCDPower(&currentCD, &voltageCD))
		{
			LOG_INF("CoilDrive Power: %d mA, %d mV", currentCD, voltageCD);
			if(currentCD > MAX_COILDRIVE_CURRENT)
			{
				chargingError = CHARGING_ERROR_CDCURRENT;
				break;
			}
			if(voltageCD > MAX_COIL_VOLTAGE || (chargerMode == CHARGER_MODE_CHARGE_NOFEEDBACK && voltageCD > MAX_COIL_VOLTAGE_NOFEEDBACK))
			{
				chargingError = CHARGING_ERROR_CDVOLTAGE;
				break;
			}
		}
		else
		{
			LOG_INF("Failed to get CD Power");
		}

		//Get Coil temperature
		tempCD = getThermistor();
		if(tempCD > COIL_OPEN_TEMP)
		{
			//valid temperature
			timeCheckCoil = k_uptime_get();
		}
		
		if (tempCD > COIL_MAX_TEMP)
		{
			chargingError = CHARGING_ERROR_CHARGERTEMP;
			break;
		}
			
		if(tempCD <= COIL_OPEN_TEMP)
		{
			LOG_INF("Failed to get CD Thermistor");
			if((k_uptime_get()-timeCheckCoil)/1000 >= TIME_OUT_COILTEMP)
			{
				chargingError = CHARGING_ERROR_CHARGERTEMP_OPEN;
				break;
			}
			
		}

		if (chargerMode == CHARGER_MODE_CHARGE_NOFEEDBACK)
		{
			displayMode = DISPLAY_NO_FEEDBACK;

		}
		else{ //CHARGER_MODE_CHARGE: Charge with feedback from PM
			//First, check timoeuts
			if ((k_uptime_get()-timeCheckCoupling)/1000 > TIME_OUT_COUPLING )
			{
				chargingError = CHARGING_ERROR_COUPLING;
				break;
			}

			if (timeCheckAppRadio != 0)
			{
				if((k_uptime_get()-timeCheckAppRadio)/1000 > TIME_OUT_RADIO)
				{
					chargingError = CHARGING_ERROR_RADIO;
					break;
				}
			}
			else
			{
				if((k_uptime_get()-timeCheckStart)/1000 > TIME_OUT_APP)
				{
					if (hasCommunicated)
					{
						chargingError = CHARGING_ERROR_NO_APP;
					}
					else
					{
						chargingError = CHARGING_ERROR_NO_PM;
					}
					break;
				}
			}

			if( serialPM == 0 )  //We have not yet made radio contact with this PM in App mode....likely in bootloader mode
			{ 
				if(ReadSDO(7, 0x1018, 3, 2, buf) == 8) //Read PM Rev and Serial 
				{
					timeCheckAppRadio = k_uptime_get();
				    revPM    = (uint16_t) buf[0] + (((uint16_t) buf[1]) << 8);
					serialPM = (uint16_t) buf[4] + (((uint16_t) buf[5]) << 8);
					LOG_INF("Found PM#%d, rev %d", serialPM, revPM);
					hasCommunicated = true;
					displayMode = DISPLAY_SERIAL_REV;
					configPM = 1;
				}
				else{ //PM may still be in bootloader mode
					loadRadioSettingsForPMBoot();
					bootradio = getAppRadioFromPMBoot(&addrAP, &addrPM, &chan, &power);
					loadRadioSettingsFromFlash();
					
					if(bootradio)
					{
						hasCommunicated = true;
						timeCheckBootRadio = k_uptime_get();
						if(timeCheckBootloader == 0) 
						{
							timeCheckBootloader = timeCheckBootRadio;
						}

						LOG_INF("***** Found implant in BL Mode!");
						copyRadioSettings(buf);
						if(addrAP == buf[0] && addrPM == buf[1] && chan == buf[2] && power == buf[3]){
							displayMode = DISPLAY_BOOT_RADIO_MATCH;
							LOG_INF("App readio settings good");
						}
						else{
							displayMode = DISPLAY_BOOT_RADIO_MISMATCH;
							updateMedRadioLocalAddress(addrAP);
							updateMedRadioRemoteAddress(addrPM);
							updateMedRadioChannel(chan);
							updateMedRadioTXPower(power);
							copyRadioSettings(&buf[0]);
							saved_settings_write(RADIO_SETTINGS_ID, buf, 7);
							LOG_INF("App readio settings don't match flash");
						}
					}
					else{
						LOG_INF("***** Could not Find implant in BL Mode");
						if (addrPM == 0 || (k_uptime_get()-timeCheckBootRadio)/1000 > TIME_BOOT_TO_APP) 
						{
							//The charger hasn't detected a PM yet in either App or BL Mode or has missed more messages than expected 
							displayMode = DISPLAY_WAITINGFORBOOT;
							timeCheckBootloader = 0;   //PM may have lost power, restart bootloader clock
						}
						timeCheckBootloader = 0;   //PM may have lost power, restart bootloader clock
						//JML TODO: consider turning on and off the coil here or suggesting magnet
					}
				}
			}
			else if (configPM)
			{
				if (configPM == 1)
				{
					if(ReadSDO(7, 0x2004, 1, 2, buf)==8) 
					{
						//Check if the time is already set on the PM and is accurate to within a minute of the PM. 
						//JML Note: This does not account for rollovers in hour/day/month/year, but it doesn't matter if we set the clock again
						if (buf[0] == getDate(rtc) && //date
							buf[1] == getMonth(rtc) && //month
							buf[2] == (uint8_t) ((2000 + (uint16_t) getYear(rtc))&0xFF) && //year LB  
							buf[3] == (uint8_t) ((2000 + (uint16_t) getYear(rtc))>>8) && //year HB
							buf[6] == getHours(rtc) &&  //hours
							abs(buf[5] - getMinutes(rtc)) <=1){ //minutes
							LOG_INF("Clock already set");
							configPM = 0;
							
						}
						else{
							//got a valid time but it doesn't match, set clock
							displayMode = DISPLAY_SET_CLOCK;
							configPM = 2;
						}
					} 
				}
				//Generate and send out a message to the PM
				if (configPM == 2)
				{
					if(NMT(7, 0x97, 0, 0)) //Halt PM RTC
					{
						configPM = 3;
						LOG_INF("\n ** HALT PM RTC ** \n");
					}
					else
					{
						//JML TODO: how many times to retry?
					} 
				}
				if (configPM == 3)
				{
					//set date and time from same RTC read:

					buf[0] = getDate(rtc); //date
					buf[1] = getMonth(rtc); //month
					buf[2] = (uint8_t) ((2000 + (uint16_t) getYear(rtc))&0xFF); //year LB
					buf[3] = (uint8_t) ((2000 + (uint16_t) getYear(rtc))>>8); //year HB
					buf[4] = getSeconds(rtc); //seconds
					buf[5] = getMinutes(rtc); //minutes
					buf[6] = getHours(rtc); //hours
					buf[7] = getDayOfWeek(rtc); //day of week
					if(WriteSDO(7, 0x2004, 1, 2, buf, 8)) 
					{
						//set Date on PM successful		   
						LOG_INF("\n ** SET PM DATE/TIME ** \n");
						configPM = 4;
					}
					else
					{
						LOG_INF("Set PM date/time unconfirmed");
						//JML TODO: how many times to retry?
					} 
				}
				if (configPM == 4)
				{
					if(NMT(7, 0x88, 0, 0)) //Set PM RTC
					{
						configPM = 0;
						LOG_INF("\n ** Set/Start PM RTC ** \n");
					}
					else
					{
						LOG_INF("Set/Start PM RTC unconfirmed");
						//JML TODO: how many times to retry?
					}
				}
			}
			else if (target == 0xFF)
			{
				//SDO read 7:3000:0B, resp is uint8_t
				if(ReadSDO(7, 0x3000, 0x0B, 1, &target))
				{
					LOG_INF("Read target success");
					disptarg = target;
					radioIndicator = 1;
				}
				else
				{
					LOG_INF("Read target failed");
					//JML TODO: how many times to retry?
				} 
			}
			else if (target >= 0x80)
			{
				//SDO write 7:3000:0B, resp is uint8_t (0 if successful)
				buf[0] = target & 0x7F;
				if(WriteSDO(7, 0x3000, 0x0B, 1, buf, 1))
				{
					LOG_INF("Write target success");
					target = 0xFF; //trigger read next cycle
					radioIndicator = 1;
				}
				else
				{
					LOG_INF("Write target failed");
					//JML TODO: how many times to retry?
				} 
			}
			else //Attempt full update
			{
				//SDO read 7:3000:0D, resp is 21 uint8_ts
				n = ReadSDO(7, 0x3000, 0x0D, 1, resp);
				if(n < 21)
				{
					LOG_INF("Read charge data failed");
					//unexpected or null response e.g. radio timeout
				
					if(modeLED & LED_CHARGER) setLEDs(1,2,2); //set red LED

					timeSince = (k_uptime_get()-timeCheckAppRadio)/1000;
					if (timeSince > TIME_WARN_RADIO )
					{
						displayMode = DISPLAY_RADIO_WARN;
					}
					else if (fullUpdate)
					{
						//leave screen with stagnant PM data and updated Coil data
						//JML: warning triangles are currently missing, because they are cleared each cycle
						displayMode = DISPLAY_PMFEEDBACK;
					}
				}
				else
				{
					radioIndicator = 1;
					fullUpdate = true;
					
					vrec   = (uint16_t) resp[ 0] + (((uint16_t) resp[ 1])<<8);
					tempPM = (uint16_t) resp[ 2] + (((uint16_t) resp[ 3])<<8);
					test   = (uint16_t) resp[ 4] + (((uint16_t) resp[ 5])<<8);
					B1V    = (uint16_t) resp[ 6] + (((uint16_t) resp[ 7])<<8);
					B2V    = (uint16_t) resp[ 8] + (((uint16_t) resp[ 9])<<8);
					B3V    = (uint16_t) resp[10] + (((uint16_t) resp[11])<<8);
					NCL    = (uint16_t) resp[12] + (((uint16_t) resp[13])<<8);
					B1I    = (int16_t)((uint16_t) resp[14] + (((uint16_t) resp[15])<<8));
					B2I    = (int16_t)((uint16_t) resp[16] + (((uint16_t) resp[17])<<8));
					B3I    = (int16_t)((uint16_t) resp[18] + (((uint16_t) resp[19])<<8));
					cmd    = resp[20];


					n=sprintf(logStr, "%02X/%02X/%02X %02X:%02X:%02X %8d COIL T%3d V%4d I%4d PM T%3d V%3d BV%4d %4d %4d BI%4d %4d %4d %3d/%3d\n", 
							rtc[5], /*month*/
							rtc[4], /*date*/
							rtc[6], /*year*/
							rtc[2], /*hour*/
							rtc[1], /*minute*/
							rtc[0], /*second*/ 
							(uint32_t)(k_uptime_get()-timeCheckStart)/100, /* time in 0.1s since starting charge */
							tempCD,	voltageCD,	currentCD,
							tempPM, vrec, B1V, B2V, B3V, B1I, B2I, B3I, cmd, target); 




					logToFile(logStr, n);


					//Confirm Test value (counter) is changing
					LOG_INF("test:%d", test);
					if (test != previous_test)
					{
						timeCheckAppRadio = k_uptime_get(); //Valid app radio message and PM is updating
					}
					else{
						LOG_INF("PM Test value not changing");
						//PM repeating same packet?
						if(modeLED & LED_CHARGER) setLEDs(1,2,2); //set Red LED
						timeSince = (k_uptime_get()-timeCheckAppRadio)/1000;
						LOG_INF("No response: Time since last PM test change: %ds", timeSince);
						if (timeSince > TIME_OUT_RADIO )
						{
							chargingError = CHARGING_ERROR_STALE;
							break;
						}
					}
					previous_test = test;

					if(cmd > 0)
					{
						timeCheckCoupling = k_uptime_get(); //Good coupling confirmed, implant charging
						if(status != COUPLED)
						{	
							playsound = PLAYSOUND_COUPLE;
							status = COUPLED;	
						}
					}
					else
					{
						if ((k_uptime_get()-timeCheckCoupling)/1000 > TIME_WARN_COUPLING )
						{
							charWarnCoupling = CG_TRIANGLE;

							if(modeLED & LED_CHARGER) setLEDs(2,2,1); //Set Warn LED
							if(status == COUPLED)
							{	
								playsound = PLAYSOUND_UNCOUPLE;
								status = UNCOUPLED;	
							}
						}
					}
					if (cmd<target) 
					{
						vrecMin = VREC_MIN_STEPPING;
						vrecMax = VREC_MAX_STEPPING;
					}
					else
					{
						vrecMin = VREC_MIN_STEADY;
						vrecMax = VREC_MAX_STEADY;
					}

				
					if(tempPM <= PM_OPEN_TEMP)
					{
						cntTempPM++;
						if(cntTempPM > 20)
						{
							chargingError = CHARGING_ERROR_PMTEMP_OPEN;
							break;
						}
					}
					else
					{
						cntTempPM = 0;
					}
					if(tempPM >= PM_MAX_TEMP)
					{
						chargingError = CHARGING_ERROR_PMTEMP;
						LOG_INF("Temp PM: %d", tempPM);
						break;
					} 
					else if (tempPM >= PM_WARN_TEMP || tempCD >= COIL_WARN_TEMP)
					{
						if(modeLED & LED_CHARGER) setLEDs(2,2,1); //Set warn LED and warn indication for LCD
						if(tempPM >= PM_WARN_TEMP)
						{
							charWarnPMTemp = CG_TRIANGLE;
						}
						if(tempCD >= COIL_WARN_TEMP)
						{
							charWarnCoilTemp = CG_TRIANGLE;	
						}
						//reduce coil voltage if VREC is above the No Charge Threshold
						if (vrec >= VREC_MIN_STEADY && voltageSetting > minVoltageSetting)
						{
							if (vrec >  VREC_MAX_QUICK)
							{
								voltageSetting -= 500; 
								if(voltageSetting < minVoltageSetting)
								{
									voltageSetting = minVoltageSetting;
								}
								setDCDC(voltageSetting);	
							}
							else
							{
								voltageSetting -= stepVoltage;
							}
							setDCDC(voltageSetting);
							LOG_INF("PM or Coil Temp Warn, reducing coil voltage: %d ", voltageSetting);
						}			
					}
					else//PM and Coil Temperature is below warn threshold
					{
						//Increase the Coil Drive Voltage if VREC is insufficient 
						//Decrease the Coil Drive Voltage if VREC is excessive
						//VREC is excessive: VREC>VREC_MAX_STEADY or VREC>VREC_MAX_STEPPING
						//VREC is insufficient: VREC<VREC_MIN_STEADY or VREC>VREC_MIN_STEPPING
						delta = 0;

						if (vrec < vrecMin) //VREC is below what is required to reach/maintain max charging rate
						{
							if (tempPM <= PM_GOOD_TEMP && tempCD <= COIL_GOOD_TEMP && voltageSetting < maxVoltageSetting)
							{
								delta = (vrecMin - vrec)/2 + 1;  
								if(delta>20)
								{
									delta = 20; //max step up of 0.2V
								}
								voltageSetting += delta*stepVoltage;
								if(voltageSetting > maxVoltageSetting)
								{
									voltageSetting = maxVoltageSetting;
								}
								setDCDC(voltageSetting);
								LOG_INF("increasing coil voltage: %d ", voltageSetting);
							}
							else //Temperature is not sufficiently low to step up coil voltage, or coil voltage is already at maximum
							{
								if(modeLED & LED_CHARGER) setLEDs(2,2,1); //Set warn LED
								charWarnCoupling = CG_TRIANGLE;
								if(tempPM > PM_GOOD_TEMP)
								{
									charWarnPMTemp = CG_TRIANGLE;
								}
								if(tempCD > COIL_GOOD_TEMP)
								{
									charWarnCoilTemp = CG_TRIANGLE;	
								}
								if(voltageSetting >= maxVoltageSetting)
								{
									charWarnCoilV = CG_TRIANGLE;	
								}
							}
						}
						else if (vrec > vrecMax) //VREC is excessive
						{
							if (voltageSetting > minVoltageSetting)
							{
								if (vrec < VREC_MAX_QUICK && madeChangeLastCycle) //skip this cycle, allow VREC to stabilize unless VREC is very high 
								{
									madeChangeLastCycle = false;
								}
								else
								{
									if (vrec > VREC_MAX_QUICK)
									{
										delta = -50; //step down 0.5V
									}
									else 
									{
										delta = -((vrec - vrecMax)/50 + 1);  
										if(delta<-5)
										{
											delta = -5; //max step down of 0.05
										}
									}
									voltageSetting += delta*stepVoltage; //delta is neg
									if(voltageSetting < minVoltageSetting)
									{
										voltageSetting = minVoltageSetting;
									}
									setDCDC(voltageSetting);
									madeChangeLastCycle = true;
									LOG_INF("decreasing coil voltage: %d ", voltageSetting);
								}
							}
						}

						
					}

					BavgV = (B1V+B2V+B3V)/3;
					BavgI = (B1I+B2I+B3I)/3;

					displayMode = DISPLAY_PMFEEDBACK;
					

					//If the target current should be adjusted:
					if (tempPM <= PM_GOOD_TEMP  && tempCD <= COIL_GOOD_TEMP && 
							target + TARGET_INCREMENT <= TARGET_MAX &&
							(k_uptime_get()-timeCheckTargetChange)/1000 >= TIME_MIN_TARGET_CHANGE)
					{
						target += TARGET_INCREMENT;
						target |= 0x80;
						timeCheckTargetChange = k_uptime_get();
						LOG_INF("PM Temp Warn, increasing charge target");
					}
					else if ((tempPM >= PM_WARN_TEMP || tempCD >= COIL_WARN_TEMP) &&  
							target - TARGET_INCREMENT >= TARGET_MIN &&
							(k_uptime_get()-timeCheckTargetChange)/1000 >= TIME_MIN_TARGET_CHANGE)
					{
						target -= TARGET_INCREMENT;
						target |= 0x80;
						timeCheckTargetChange = k_uptime_get();
						LOG_INF("PM Temp Warn, decreasing charge target");
					}

				} 
			}

		}
		if(displayMode)
		{
			LOG_INF("Update Display %d", displayMode);

			
			sprintf(str, "%1c %1c%2d.%1d%1c%1c%1d.%02dV %1d.%02dA",
						CG_COIL, charWarnCoilTemp, tempCD/10, tempCD%10, CG_DEGREEC, 
						charWarnCoilV, voltageCD/1000, (voltageCD%1000)/10, 
						currentCD/1000, (currentCD%1000)/10);

			switch(displayMode)
			{
				case DISPLAY_WAITINGFORBOOT:
					timeSince = (k_uptime_get()-timeCheckStart)/1000;
					sprintf(&str[20], " Place coil         "\
								 	  "     over implant!  "\
								 	  " %2ds remaining...   ", TIME_OUT_APP-timeSince);		
					break;

				case DISPLAY_BOOT_RADIO_MATCH:
					if (timeCheckBootloader != 0)	
					{
						timeSince = (k_uptime_get()-timeCheckBootloader)/1000;
						sprintf(&str[20], "                     "\
										"   Found Implant    "\
										" Wait up to %2ds...   ", TIME_TO_BOOT-timeSince);		
					}				
					break;

				case DISPLAY_BOOT_RADIO_MISMATCH:
					if (timeCheckBootloader != 0)	
					{
						timeSince = (k_uptime_get()-timeCheckBootloader)/1000;
						sprintf(&str[20], "  Found Implant     "\
										" (App Radio Changed)    "\
										" Wait up to %2ds...  ", TIME_TO_BOOT-timeSince);	
					}
					break;

				case DISPLAY_SERIAL_REV:
					sprintf(&str[20], "                    "\
								 	  "Found PM#%4d v%04d"\
								 	  "                    ", serialPM, revPM);
					break;
				
				case DISPLAY_SET_CLOCK:
					sprintf(&str[20], "                    "\
								 	  "Found PM#%4d v%04d"\   
								 	  "   Setting clock..    ",	serialPM); //keep PM rev and serial# visible
					break;

				case DISPLAY_RADIO_WARN:
					sprintf(&str[20], "                    "\
								 	  "   No response      "\
								 	  "    from implant!   ");
					break;

				case DISPLAY_PMFEEDBACK:
					setIconRSSI(radioIndicator);
					setIconVoltage(BavgV);
					sprintf(&str[20], "%1c %1c%2d.%1d%1c            "\
								 	"%1c %1c%2d.%1dV  %3d/%3d   "\
								 	"%1c  %4dmV  %4dmA  %1c", 
							CG_PERSON, charWarnPMTemp, tempPM/10, tempPM%10, CG_DEGREEC, 
							CG_LIGHTNING, charWarnCoupling,vrec/10, vrec%10, cmd, disptarg,
							CG_BATTERY, BavgV, BavgI/10, CG_RADIO);
					// sprintf(str, "coil: %1c%2d.%1dC  %1c%1d.%02dVimpl: %1c%2d.%1dC   %1d.%02dAVrec: %1c%2d.%1dV %3d/%3dBatt: %4dmV%4d.%1dmA", 
					// 	charWarnCoilTemp, tempCD/10, tempCD%10, charWarnCoilV, voltageCD/1000, (voltageCD%1000)/10,
					// 	charWarnPMTemp, tempPM/10, tempPM%10, currentCD/1000, (currentCD%1000)/10,
					// 	charWarnCoupling,vrec/10, vrec%10, cmd, target,
					// 	BavgV, BavgI/10, abs(BavgI%10));

					break;

				case DISPLAY_NO_FEEDBACK:
					sprintf(&str[20], "     USE CAUTION!   "\
									 "   Coil ON without  "\
								 	 "   implant feedback ");
					break;
			} 
			OutputDisplay(0, str, "","","",80,0,0,0);
		}

		delayCycle = (uint16_t) (k_uptime_get() - timeCycle);
		LOG_INF("DelayCycle: %dms", delayCycle);
		if(delayCycle < TIME_CYLCE_LED_ON)
		{
			k_msleep(TIME_CYLCE_LED_ON - delayCycle);
		}
		if(modeLED & LED_CHARGER) setLEDs(2,0,2);  //turn off green LED so it blinks
		k_msleep(TIME_CYLCE_LED_OFF);

		if (displayMode == DISPLAY_SERIAL_REV || DISPLAY_BOOT_RADIO_MISMATCH)
		{
			k_msleep(500); //delay an extra 0.5s to make reading screen easier
		}
		
		if(playsound)
		{
			switch(playsound)
			{
				case PLAYSOUND_COUPLE:
					audioList.len = sizeof(coupleSoundNotes)/sizeof(coupleSoundNotes[0]);;
					memcpy(&audioList.notePeriod[0],coupleSoundNotes, sizeof(coupleSoundNotes));
					memcpy(&audioList.noteTime[0],coupleSoundTimes, sizeof(coupleSoundTimes));
					break;
				case PLAYSOUND_UNCOUPLE:
					audioList.len = sizeof(uncoupleSoundNotes)/sizeof(uncoupleSoundNotes[0]);;
					memcpy(&audioList.notePeriod[0],uncoupleSoundNotes, sizeof(uncoupleSoundNotes));
					memcpy(&audioList.noteTime[0],uncoupleSoundTimes, sizeof(uncoupleSoundTimes));
					break;
				default:
					audioList.len = 0;	
			}
			if(audioList.len > 0)
			{	
				while(k_msgq_put(&audio_msgq, &audioList, K_NO_WAIT) != 0)
				{
					/* message queue is full: purge old data & try again */
					LOG_INF("Purging Audio MsgQ");
					k_msgq_purge(&audio_msgq);
				}
			}
			playsound = PLAYSOUND_NONE;
		}
		
		feedWatchdog();

	} //end WHILE loop



	switch(chargingError)
	{
		case CHARGING_ERROR_NONE:
			OutputDisplay(1, "","      CHARGING      ", "       Stopped      ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_RADIO:
			OutputDisplay(1, "","       ERROR        ", "   Radio Timed Out  ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_STALE:
			OutputDisplay(1, "","       ERROR        ", " Stale Charging Data","",0, 20,20,0);
			break;
		case CHARGING_ERROR_NO_PM:
			OutputDisplay(1, "","       ERROR        ", "     No PM Found    ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_NO_APP:
			OutputDisplay(1, "","       ERROR        ", "  No PM App Found   ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_PMTEMP:
			OutputDisplay(1, "","       ERROR        ", "  PM Temperature    ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_CHARGERTEMP:
			OutputDisplay(1, "","       ERROR        ", "  Coil Temperature  ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_CHARGERTEMP_OPEN:
			OutputDisplay(1, "","       ERROR        ", "   Coil Temp Open   ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_PMTEMP_OPEN:
			OutputDisplay(1, "","       ERROR        ", "    PM Temp Open    ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_COUPLING:
			OutputDisplay(1, "","       ERROR        ", "    Poor Coupling   ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_SYSCURRENT:
			OutputDisplay(1, "","       ERROR        ", "   System Current   ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_CDCURRENT:
			OutputDisplay(1, "","       ERROR        ", " Coil Drive Current ","",0, 20,20,0);
			break;
		case CHARGING_ERROR_CDVOLTAGE:
			OutputDisplay(1, "","       ERROR        ", " Coil Drive Voltage ","",0, 20,20,0);
			break;

	}	

	//Turn off coil and exit.  chargingMode will still be CHARGER_MODE_CHARGE or CHARGER_MODE_CHARGE_NOFEEDBACK in the case of an error
	if(chargingError == CHARGING_ERROR_NONE) 
	{ 
		if(modeLED & LED_CHARGER) setLEDs(0,0,0);
		//playsound = PLAYSOUND_CHARGEFULL;
	}
	else
	{
		if(modeLED & LED_CHARGER) setLEDs(1,0,0);
		playsound = PLAYSOUND_ERROR;
		
	}

	switch(playsound) //JML: Add finished charging sound
	{
		case PLAYSOUND_ERROR:
			audioList.len =  sizeof(alertSoundNotes)/sizeof(alertSoundNotes[0]);
			memcpy(&audioList.notePeriod[0],alertSoundNotes, sizeof(alertSoundNotes));
			memcpy(&audioList.noteTime[0],alertSoundTimes, sizeof(alertSoundTimes));
			break;	
		case PLAYSOUND_CHARGEFULL:
			audioList.len = 0;
			break;
		default:
			audioList.len = 0;
	}
	if(audioList.len > 0)
	{	
		while(k_msgq_put(&audio_msgq, &audioList, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging Audio MsgQ");
			k_msgq_purge(&audio_msgq);
		}
	}
	

	stopDCDC();
	stopCoilDrive();
	
	//pause 1s on current screen.  If there was an error.  Leave error screen until charging mode is changed
	do {
		k_msleep(1000);
	} while(chargerMode == CHARGER_MODE_CHARGE || chargerMode == CHARGER_MODE_CHARGE_NOFEEDBACK); 

	return;

}



void powerDownPM(void)
{

	OutputDisplay(1, "","   POWERING DOWN    ", "       Implant      ","",0, 20,20,0);
	k_msleep(1000);

	uint8_t data = 0x0B;
 	//SDO write action 0x0B (script-based power down) 7:1F53:04, resp is uint8_t (0 if successful)
	if(WriteSDO(7, 0x1f53, 4, 1, &data, 1))
	{
		//PM got action command to turn off
		LOG_INF("PM script action : Power Off confirmed");
		OutputDisplay(1, "","    POWER DOWN      ", "  Confirmed-Script  ","",0, 20,20,0);
	}
	else
	{
		LOG_INF("PM script action : Power Off unconfirmed");
	} 
	k_msleep(1000);
	

	if(NMT(7, 0x9E, 0, 0))
	{
		//PM got action command to turn off
		LOG_INF("PM NMT : Power Off confirmed");
		OutputDisplay(1, "","    POWER DOWN      ", "  Confirmed-NMT     ","",0, 20,20,0);
	}
	else
	{
		LOG_INF("PM NMT : Power Off unconfirmed");
	} 

	k_msleep(1000);

}



void testNMT(void)
{
	if(NMT(7, 0x95, 0, 0))
	{
		LOG_INF("NMT Successful");
	}
	else{
		LOG_INF("NMT Error");
	}
}


void testReadSDO(void)
{
	uint8_t resp[4];
	uint8_t n;

	n = ReadSDO(7, 0x1f53, 1, 4, resp);

	LOG_HEXDUMP_INF(resp, n, "SDO Read");
}

void testWriteSDO(void)
{
	uint8_t data[4] = {1,2,3,4};
	

	if( WriteSDO(7, 0x1f53, 1, sizeof(data), data, sizeof(data)) )
	{
		LOG_INF("Write Successful");
	}
	else{
		LOG_INF("Write Error");
	}
}



//returns 0 if fails to read, 1 otherwise
uint8_t getAppRadioFromPMBoot(uint8_t* addrAP, uint8_t* addrPM, uint8_t* chan, uint8_t* power)
{
	int err;
	struct medRadio_type medRadio;
	medRadio.len = 1;
	medRadio.buf[0] = 0x17;
	medRadio.source = SOURCE_CHARGER;
	struct cmdHandler_type cmdHandler;

	LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "TX MedRadio Packet");
	while(k_msgq_put(&imp_req_msgq, &medRadio, K_NO_WAIT) != 0)
	{
		/* message queue is full: purge old data & try again */
		LOG_INF("Purging MsgQ");
		k_msgq_purge(&imp_req_msgq);
	}
	//wait for a response
	err = k_msgq_get(&charger_resp_msgq, &cmdHandler, K_MSEC(getTaskTimeoutForMedRadio()));  
	LOG_HEXDUMP_INF(cmdHandler.buf, cmdHandler.len, "CMDHANDLER resp");
	if(err == 0 && cmdHandler.len == (7 + NON_PAYLOAD_RESP_BYTES)  && cmdHandler.buf[NON_PAYLOAD_RESP_BYTES + 0] == 0x27)
	{	

		LOG_INF("****Got Radio Settings");
		*addrPM = cmdHandler.buf[NON_PAYLOAD_RESP_BYTES + 1];
		*addrAP = cmdHandler.buf[NON_PAYLOAD_RESP_BYTES + 2];
		*chan   = cmdHandler.buf[NON_PAYLOAD_RESP_BYTES + 3];
		*power  = cmdHandler.buf[NON_PAYLOAD_RESP_BYTES + 4];
		return 1;
	}
	else
	{
		
		return 0;
	}


}

//returns 0 if fails to read, otherwise return length of read buffer
uint8_t ReadSDO(uint8_t node, uint16_t index, uint8_t subindex, uint8_t numSubIndices, uint8_t* resp)
{
	uint8_t data[4] = {index & 0xFF, index >> 8,  subindex, numSubIndices};
	uint8_t protocol;
	int8_t len;

	if(numSubIndices > 1)
	{
		protocol = 0x30;
	}
	else
	{
		protocol = 0x24;
	}
	len = transmit(node, data, sizeof(data), 0, protocol, resp); //waits for response or timeout	
	if(len < 1)
	{
		return 0;
	}	
	else 
	{
		return len;
	}

}

//returns 0 if fails to read, returns 1 if write successfully acknowledged (PM sends response: 0)
uint8_t WriteSDO(uint8_t node, uint16_t index, uint8_t subindex, uint8_t numSubIndices, uint8_t* writeData, uint8_t sizeBytes)
{
	uint8_t data[54] = {index & 0xFF, index >> 8,  subindex, 0, 
							0,0,0,0,0,0,0,0,0,0,  
							0,0,0,0,0,0,0,0,0,0,  
							0,0,0,0,0,0,0,0,0,0, 
							0,0,0,0,0,0,0,0,0,0, 
							0,0,0,0,0,0,0,0,0,0};
	uint8_t resp[4];
	uint8_t protocol;
	int8_t len;

	if (sizeBytes > 50)
	{
		return 0;
	}
	if (numSubIndices>1)
	{
		switch(sizeBytes/numSubIndices)
		{
			case 1: data[3] = numSubIndices; break;
			case 2: data[3] = numSubIndices + BIT6; break;
			case 4: data[3] = numSubIndices + BIT7; break;
			default: return 2;
		}
	}
	memcpy(&data[4], writeData, sizeBytes );
	if(numSubIndices > 1)
	{
		protocol = 0xB0;
	}
	else
	{
		protocol = 0xA4;
	}
	len = transmit(node, data, sizeBytes+4, 0, protocol, resp); //waits for response or timeout	
	if(len == 1 && resp[0] == 0)
	{
		return 1;
	}	
	else
	{
		return 0;
	}
}


//returns 0 if fails to read, returns 1 if write successfully acknowledged (PM echoes cmd)
uint8_t NMT(uint8_t node, uint8_t cmd, uint8_t param1, uint8_t param2)
{
	uint8_t data[7] = {0,0,0,0,0,0,0};
	uint8_t resp[1];
	int8_t len;

	data[3] = 3;
	data[4] = cmd;
	data[5] = param1;
	data[6] = param2;
	len = transmit(node, data, sizeof(data), 0, 0x34, resp); //waits for response or timeout	
	if(len == 1 && resp[0] == cmd)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//returns negative for error, otherwise length of response
int8_t transmit(uint8_t node, uint8_t* data, uint8_t len, uint8_t counter, uint8_t protocol, uint8_t* resp)
{
	struct medRadio_type medRadio;
	struct cmdHandler_type cmdHandler;
	int err;

	uint8_t *medRadioResp;
	uint8_t medRadioRespLength = 0;
	cmdHandler.len = 0;

	medRadio.len = len + 4;
	medRadio.buf[0] = protocol;
	medRadio.buf[1] = counter;
	medRadio.buf[2] = 1; //netID
	medRadio.buf[3] = node;
	memcpy(&medRadio.buf[4], data, len);

	medRadio.source = SOURCE_CHARGER;


	LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "TX MedRadio Packet");
	while(k_msgq_put(&imp_req_msgq, &medRadio, K_NO_WAIT) != 0)
	{
		/* message queue is full: purge old data & try again */
		LOG_INF("Purging MsgQ");
		k_msgq_purge(&imp_req_msgq);
	}
	//wait for a response
	err = k_msgq_get(&charger_resp_msgq, &cmdHandler, K_MSEC(getTaskTimeoutForMedRadio()));  
	if(err || cmdHandler.len < NON_PAYLOAD_RESP_BYTES + 2 )
	{
		return -1; //no valid response 
	}
	else
	{
		LOG_HEXDUMP_INF(&cmdHandler.buf[NON_PAYLOAD_RESP_BYTES],  cmdHandler.len - NON_PAYLOAD_RESP_BYTES - 2, "RX MedRadio in Charger Thread");
		medRadioResp = &cmdHandler.buf[NON_PAYLOAD_RESP_BYTES];
		medRadioRespLength = cmdHandler.len - NON_PAYLOAD_RESP_BYTES - 2; 
	}

	for (uint8_t i=0; i<DATA_INDEX-1; i++)
	{
		if(medRadioResp[i] != medRadio.buf[i])
		{
			return -2; //response does not match request
			LOG_HEXDUMP_INF(medRadioResp, medRadioRespLength, "Resp");
			LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "Req");
		}
	}

	LOG_INF("Length of response after sizebyte%d", medRadioRespLength-DATA_INDEX );
	memcpy(resp, &medRadioResp[DATA_INDEX], medRadioRespLength-DATA_INDEX); 

	return medRadioRespLength-DATA_INDEX;

}




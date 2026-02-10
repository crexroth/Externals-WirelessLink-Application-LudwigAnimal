/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>


#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>



//#include <zephyr/drivers/sensor.h>


#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

// /* Power Management */
// #include <zephyr/pm/pm.h>
// #include <zephyr/pm/device.h>
// #include <zephyr/pm/policy.h>

#include <zephyr/sys/poweroff.h>

#include <hal/nrf_gpio.h>

/*Analog-to-Digital*/
#include <zephyr/drivers/adc.h>

/*I2C Display*/


#include "cmdhandler.h"
#include "cc1101radio.h"
#include "charger.h"
#include "wlgpio.h"
#include "spi.h"
#include "imu.h"






// /* Prevent deep sleep (system off) from being entered on long timeouts
//  * or `K_FOREVER` due to the default residency policy.
//  *
//  * This has to be done before anything tries to sleep, which means
//  * before the threading system starts up between PRE_KERNEL_2 and
//  * POST_KERNEL.  Do it at the start of PRE_KERNEL_2.
//  */
// static int disable_ds_1(void)
// {

// 	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
// 	return 0;
// }

// SYS_INIT(disable_ds_1, PRE_KERNEL_2, 0);

//#define LOG_MODULE_NAME app
LOG_MODULE_REGISTER(app);

#define IMP_STACKSIZE 8192 //4096 //CONFIG_BT_NUS_THREAD_STACK_SIZE
#define IMP_PRIORITY 7

#define SENSOR_POLL_INTERVAL 100 //ms
#define SENSOR_STACKSIZE 1024 //bytes
#define SENSOR_PRIORITY 9

#define COIL_STACKSIZE 4096 //bytes Needs space for SD Card functions
#define COIL_PRIORITY 8

#define CHARGER_STACKSIZE 4096 //bytes Needs space for SD Card functions
#define CHARGER_PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

//#define RUN_STATUS_LED DK_LED1
//#define RUN_LED_BLINK_INTERVAL 1000
//#define CON_STATUS_LED DK_LED3


#define RED_LED		DK_LED1
#define GREEN_LED   DK_LED2
#define BLUE_LED    DK_LED3


#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK
#define KEY_ACTION1		   DK_BTN3_MSK
#define KEY_ACTION2	       DK_BTN4_MSK


#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 500 //time in microseconds
//#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));

//If using Dynamic name
// static struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, ( BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// };

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, ( BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif


#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256

#define EXTRALONG_BUTTON_PUSH 10000  //in ms
#define LONG_BUTTON_PUSH      2000   //in ms 


uint8_t rx_buf[RX_BUF_SIZE] = {0};
uint8_t tx_buf[TX_BUF_SIZE] = {0};

static bool button1_held = false;
static bool pairing_mode = false;
static bool isAdvertising = false;
static bool isReady = false;
static uint8_t bond_addr_list[CONFIG_BT_MAX_PAIRED][BT_ADDR_SIZE] = {0};


//See BT Fundamental Lesson 2, Ex 3
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);

static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static struct bt_gatt_exchange_params exchange_params;

static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}


//See BT Fundamentals Lesson 5, Ex 1
uint8_t erasebonds(void)
{
	//JML TODO: make sure this isn't called while in a connection
	int err= bt_unpair(BT_ID_DEFAULT,BT_ADDR_LE_ANY);
	if (err) {
		LOG_INF("Cannot delete bond (err: %d)\n", err);
	} else	{
		LOG_INF("Bond deleted succesfully \n");
	}	
	return err;
}

//To use a device name changed at runtime (e.g. to include serial number in advertising name)
//JML: can't figure out how to include the Complete name instead of shortened name
//#define BT_LE_ADV_CONN_ACCEPT_LIST \
//	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_USE_NAME, \
//			BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

//See BT Fundamentals Lesson 5, Ex 2, Filter accept list so only previiously paired devices may connect
#define BT_LE_ADV_CONN_ACCEPT_LIST \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN, \
			BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)			

static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data)
{
	char addr[BT_ADDR_LE_STR_LEN];

	int *bond_cnt = user_data;

	if ((*bond_cnt) < 0) {
		return;
	}

	int err = bt_le_filter_accept_list_add(&info->addr);

	memcpy(&bond_addr_list[*bond_cnt][0], &info->addr.a, sizeof(info->addr.a));

	bt_addr_le_to_str(&info->addr, addr, sizeof(addr));
	LOG_INF("Added following peer to accept list #%d: %s\n", *bond_cnt, addr);
	if (err) {
		LOG_INF("Cannot add peer to filter accept list (err: %d)\n", err);
		(*bond_cnt) = -EIO;
	} else {
		(*bond_cnt)++;
	}
}

/*  Define the function to loop through the bond list */
static int setup_accept_list(uint8_t local_id)
{
	int err = bt_le_filter_accept_list_clear();

	if (err) {
		LOG_INF("Cannot clear accept list (err: %d)\n", err);
		return err;
	}

	int bond_cnt = 0;

	bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);

	return bond_cnt;
}

static void start_adv_work_handler(struct k_work *work)
{
	int err = 0;
	/* Advertise without using Accept List when pairing_mode is set to true */
	if (pairing_mode==true) {
		err = bt_le_filter_accept_list_clear();
		if (err) {
			LOG_INF("Cannot clear accept list (err: %d)\n", err);
		} else {
			LOG_INF("Accept list cleared succesfully");
		}
		
		err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd,
				      ARRAY_SIZE(sd));
		if (err) {
			LOG_INF("Advertising failed to start (err %d)\n", err);
			return;
		}
		LOG_INF("Advertising successfully started\n");
		return;
	}

	/* Start advertising with the Accept List */
	int allowed_cnt = setup_accept_list(BT_ID_DEFAULT);
	
	if (allowed_cnt < 0) {
		LOG_INF("Acceptlist setup failed (err:%d)\n", allowed_cnt);
	} else {
		if (allowed_cnt == 0) {
			LOG_INF("Advertising with no Accept list \n");
			pairing_mode=true;
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		} else {
			LOG_INF("Advertising with Accept list \n");
			LOG_INF("Acceptlist setup number  = %d \n", allowed_cnt);
			err = bt_le_adv_start(BT_LE_ADV_CONN_ACCEPT_LIST, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		}
		if (err) {
			LOG_INF("Advertising failed to start (err %d)\n", err);
			return;
		}
		LOG_INF("Advertising successfully started\n");
	}

}
K_WORK_DEFINE(start_adv_work, start_adv_work_handler); //this replaces static struct definition and init


static void stop_adv_work_handler(struct k_work *work)
{
    ble_stop_advertising();
}
K_WORK_DEFINE(stop_adv_work, stop_adv_work_handler);



#define ADV_TIMEOUT 120 //2 minutes
static void adv_timeout_action(struct k_timer *dummy)
{
    LOG_INF("Advertising Timeout");
	//We can't call ble_stop_advertising directly because this is called by an ISR
	k_work_submit(&stop_adv_work); 
}

K_TIMER_DEFINE(adv_timer, adv_timeout_action, NULL);



uint8_t get_bonded_devices(uint8_t *buf)
{
	int num_bonds = 0;
	ble_stop_advertising();	//stop advertising becuase we can't setup accept list if we are using it.	

	num_bonds = setup_accept_list(BT_ID_DEFAULT);

	if(buf != NULL)
	{
		memcpy(buf, bond_addr_list, sizeof(bond_addr_list));
	}
	return (uint8_t) num_bonds;
}

//we can't use delays in functions ghere because they are called by ISRs, so we use the cmdhandler mechanism to initiate the actions by message queue
void btn1_extralongpress_action(struct k_timer *dummy)
{
	struct cmdHandler_type cmdHandler; 

	//if button2 is also held down (after button1), then device will enter serial recovery when it comes out of reset
	LOG_INF("Button1 extra long press: reset WL");
	cmdHandler.len = 3;
	cmdHandler.route = ROUTE_LOCAL;
	cmdHandler.buf[0] = 0xFF;
	cmdHandler.buf[1] = 0x23; //reset WL
	cmdHandler.buf[2] = cmdHandler.len;
	while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
	{
		/* message queue is full: purge old data & try again */
		LOG_INF("Purging CmdReq MsgQ");
		k_msgq_purge(&cmd_req_msgq);
	}

}

void btn2_extralongpress_action(struct k_timer *dummy)
{
	struct cmdHandler_type cmdHandler; 

	//if button1 is also held down (after button2), then device will erase bonds 
	if(button1_held)
	{
		LOG_INF("Button2 extra long press with Button1: delete bonds");
		cmdHandler.len = 3;
		cmdHandler.route = ROUTE_LOCAL;
		cmdHandler.buf[0] = 0xFF;
		cmdHandler.buf[1] = 0x22; //delete bonds
		cmdHandler.buf[2] = cmdHandler.len;
		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
	}
	else
	{
		LOG_INF("Button2 extra long press: enable pairing");
	
		cmdHandler.len = 3;
		cmdHandler.route = ROUTE_LOCAL;
		cmdHandler.buf[0] = 0xFF;
		cmdHandler.buf[1] = 0x1F; //start open pairing mode
		cmdHandler.buf[2] = cmdHandler.len;
		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
	}
}


void btn1_longpress_action(struct k_timer *dummy)
{
	//this is called by an ISR and cannot have any sleeps
    struct cmdHandler_type cmdHandler; 

	LOG_INF("Button1 long press");
	//long press action
	if(action[BUTTON_1_LONG].len > 0){
		LOG_INF("Button1 long press action started");
		cmdHandler.len = action[BUTTON_1_LONG].len;
		cmdHandler.route = ROUTE_LOCAL;
		memcpy(cmdHandler.buf, action[BUTTON_1_LONG].buf, action[BUTTON_1_LONG].len);
		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
	}
	else {
		LOG_INF("No Button1 long press action enabled");
	}

}

void btn2_longpress_action(struct k_timer *dummy)
{
    //this is called by an ISR and cannot have any sleeps
    struct cmdHandler_type cmdHandler; 

	LOG_INF("Button2 long press");
	//long press action
	if(action[BUTTON_2_LONG].len > 0){
		LOG_INF("Button2 long press action started");
		cmdHandler.len = action[BUTTON_2_LONG].len;
		cmdHandler.route = ROUTE_LOCAL;
		memcpy(cmdHandler.buf, action[BUTTON_2_LONG].buf, action[BUTTON_2_LONG].len);
		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
	}
	else {
		LOG_INF("No Button2 long press action enabled");
	}
}

K_TIMER_DEFINE(btn1_timer_extra, btn1_extralongpress_action, NULL);
K_TIMER_DEFINE(btn2_timer_extra, btn2_extralongpress_action, NULL);
K_TIMER_DEFINE(btn1_timer, btn1_longpress_action, NULL);
K_TIMER_DEFINE(btn2_timer, btn2_longpress_action, NULL);

// void sensor_thread(void);

// K_THREAD_DEFINE(sensor_thread_id, SENSOR_STACKSIZE, sensor_thread, NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0); 

//static const uint8_t action3Msg[] = { 0xB0, 0x00, 0x01, 0x07, 0x20, 0x30,  0x07, 0x02, 0x00, 0x00, 0x00}; //3020.7-8


/*ADC definitions and includes*/
/*ADC used to measure WL battery voltage*/
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf[8];
	int16_t sum;
static struct adc_sequence_options seqoptions;
static struct adc_sequence sequence;

int init_adc(void)
{
	int err;

	seqoptions.extra_samplings = 7;

	sequence.buffer = &adc_buf;
		/* buffer size in bytes, not number of samples */
	sequence.buffer_size = sizeof(adc_buf);
	sequence.options = &seqoptions;
		//Optional
		//.calibrate = true

	if (!device_is_ready((&adc_channel)->dev)) {
		LOG_ERR("ADC controller devivce %s not ready", adc_channel.dev->name);
		return 0;
	}

	/*  Setup the ADC channel */
	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)", 0, err);
		return 0;
	}

	/*Initialize the ADC sequence */
	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("Could not initalize sequnce");
		return 0;
	}

	return 1;
}

int16_t get_adc_sample(void)
{
	int val_mv;
	int err;
	int count = 0;

		/* Read a sample from the ADC */
		err = adc_read(adc_channel.dev, &sequence);
		if (err < 0) {
			LOG_ERR("Could not read (%d)", err);
		}

		LOG_INF("%d %d %d %d %d %d %d %d", adc_buf[0], adc_buf[1],adc_buf[2],adc_buf[3],adc_buf[4], adc_buf[5],adc_buf[6],adc_buf[7]);
		sum = 0;
		for( int i=0; i<8; i++)
		{
			sum+=adc_buf[i];
			
		}
		val_mv = (int)sum/8;
		
		LOG_INF("ADC reading[%u]: %s, channel %d: Raw: %d", count++, adc_channel.dev->name,
			adc_channel.channel_id, val_mv);

		/* STEP 6 - Convert raw value to mV*/
		err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
		/* conversion to mV may not be supported, skip if not */
		val_mv = (val_mv*14)/5;  //account for 2.8x resistor divider
		if (err < 0) {
					LOG_WRN(" (value in mV not available)\n");
			} else {
				LOG_INF(" = %d mV", val_mv);
				}
		return(val_mv);
}


//**************
static struct nvs_fs fs;


#define NVS_PARTITION		user_storage
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)




void power_down_sensor(void);







//JML TODO: IN USB (CDC) mode, the app does not continue initializing until the port is opened at the host end

//eventually move CRC and SYNC byte into this function
void uart_send(uint8_t* packet, uint8_t len)
{
	if(len<sizeof(tx_buf))
	{
		memcpy(tx_buf, packet, len);
		int err;	
		LOG_HEXDUMP_INF(tx_buf, tx_buf[2], "UART TX Buffer");

		err = uart_tx(uart, tx_buf, tx_buf[2], SYS_FOREVER_MS);
		if (err) {
			LOG_INF("UART write failed: UART TX FIFO not implemented");
		}
	}
	else{
		//error checking?
	}
}

void ble_send(uint8_t* packet, uint8_t len)
{

	bt_nus_send(NULL, packet, len);

}

//UART_RX_READY completes every time there is new data
//The UART0 console  on the nRF5340DK does not appear to keep up when logging 
//is also active at 115200.  In UART mode, this callback can be called with each character received.  
//
//In USB (CDC) mode, this gets called with more data at once.  
//data.rx.buf is the RX buffer (points to rx_buf or a location specified in uart_rx_enable)
//evt->data.rx.offset is the index into the buffer of the first byte recieved in this interrupt
//evt->data.rx.len is the number of bytes received in last interrupt (usually 1 or 2, when not using CDC)
//Once a completer valid packet is received or an invalid packet is detected, uart_rx_disable is called 
//This triggers UART_RX_BUF_RELEASED.  If any data is still being received UART_RX_READY will be triggered first.
//When the buffer is released we handle the contents of the packet.  Next UART_RX_DISABLED is automatically
//triggered.  We reenable UART (buffer is reset so offset is back to zero) and reinitialize packet variables 
//The buffer contents always therefore contain a single packet (or invalid packet) starting at zero index
//PKT format
//0xFF, cmd, len (payload+header), payload
//Should add checksum 
//
// TODO: UART TX buffer should be used if uart_tx fails 

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev); //should all instances of uart below be changed to dev?  and tis line removed?


	static bool disable_req;

	static uint16_t len = 0;
	static uint8_t cmd = 0;
	static uint16_t pkt_len = PKT_LEN_UNDEFINED;

	static bool look_for_start = true;

	bool end_of_packet = false;

	enableDebug0(1); //JMLDEBUG
	switch (evt->type) {
	case UART_TX_DONE:
		LOG_INF("UART_TX_DONE");
		break;

	case UART_RX_RDY:
		LOG_INF("offset %d, evtlen %d, len %d, pkt_len %d, cmd %d", evt->data.rx.offset, evt->data.rx.len, len, pkt_len, cmd);
		LOG_HEXDUMP_INF(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len, "UART_RX_RDY");

		if (look_for_start) {
			cmd = 0;
			len = 0;
			
			if(evt->data.rx.buf[0]==0xFF){
				enableDebug1(1); //JMLDEBUG
				LOG_INF("Found Packet Header");
				look_for_start = false;
			} 
			else {
				LOG_INF("Bad Packet Header");
				end_of_packet = true;
				pkt_len = PKT_LEN_UNDEFINED;
				look_for_start = true;
			}			
		} 

		if (!look_for_start){
			len += evt->data.rx.len;

			if(len >= MIN_PKT_LEN && pkt_len == PKT_LEN_UNDEFINED){
				
				cmd = evt->data.rx.buf[1];
				pkt_len = evt->data.rx.buf[2];

				if (pkt_len > MAX_PKT_LEN || pkt_len < MIN_PKT_LEN ){ //bad packet: pkt len too long or too short
					LOG_INF("Bad Packet Length");
					pkt_len = PKT_LEN_UNDEFINED;
					look_for_start = true;
					end_of_packet = true;
				}
			}
			if(len >= pkt_len && len >= MIN_PKT_LEN ){ //reached end of packet
				//TODO: add checksum here
				LOG_INF("Reached end of Packet");
				enableDebug1(0); //JMLDEBUG
				end_of_packet = true;
			}
		}
		if (disable_req) {
			return;
		}
		if(end_of_packet){
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_INF("UART_RX_DISABLED");
		//reinitialize stuff
		disable_req = false;
		pkt_len = PKT_LEN_UNDEFINED;
		look_for_start = true;

		uart_rx_enable(uart, rx_buf, sizeof(rx_buf), UART_WAIT_FOR_RX ); 

		break;

	case UART_RX_BUF_REQUEST:
		LOG_INF("UART_RX_BUF_REQUEST");
		break;

	case UART_RX_BUF_RELEASED:
		LOG_INF("UART_RX_BUF_RELEASED");
		
		//command_handler(&evt->data.rx.buf[1], pkt_len-1, ROUTE_UART);
		if(pkt_len == PKT_LEN_UNDEFINED)
		{
			LOG_INF("don't release to cmdhandler");
			break;
		}
					
		struct cmdHandler_type cmdHandler;
		cmdHandler.route = ROUTE_UART;
		cmdHandler.len = pkt_len; 
		if (cmdHandler.len > sizeof(cmdHandler.buf)){
			LOG_WRN("Max command length exceeded");
			break;
		} 
		memcpy(&cmdHandler.buf[0], &evt->data.rx.buf[0], pkt_len);
		
		

		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
		
		break;

	case UART_TX_ABORTED:
		LOG_INF("UART_TX_ABORTED");
		break;

	default:
		break;
	}
	enableDebug0(0); //JMLDEBUG
	
}


static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}


static int uart_init(void)
{
	int err;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	
	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	 if (err) {
	// 	k_free(rx);
	 	LOG_ERR("Cannot initialize UART callback: %d", err);
	 	return err;
	 }

	//JML: This is unused.  We don't want to wait for port being opened because we may just use BLE or button
	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}



	err = uart_rx_enable(uart, rx_buf, sizeof(rx_buf), 50);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
	}

	return err;
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	//stop advertising since we are only supporting 1 connection
	ble_stop_advertising();

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	

	#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
		LOG_INF("Connected %s, (Security required for NUS.  Pair if not already bonded)", addr);	
		//isReady will occur once level 4 security is reached
	#else
		LOG_INF("Connected (Seurity Disabled on NUS!) %s", addr);	
		isReady = true;
	#endif

	//Connection info (code from BLE fundamentals Lesson 3, Ex 2)
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}
	LOG_INF("Connection parameters: raw interval(x1.25ms) %d, latency %d intervals, rawtimeout(x10ms) %d", info.le.interval, info.le.latency, info.le.timeout);

	k_sleep(K_MSEC(1000));  // Delay added to avoid link layer collisions.
	update_data_length(conn);
	update_mtu(conn);

}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	isReady = false;
	ble_start_advertising();

}

//code from BLE Fundamentals Lesson 3, Ex 2
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated: raw interval(x1.25ms) %d, latency %d intervals, raw timeout(x10ms) %d", interval, latency, timeout);
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void on_security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
	if(level >= 4)
	{
		isReady = true;
	}
	else 
	 {
		isReady = false;
	 }
}
#endif

void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len; 
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = on_connected,
	.disconnected = on_disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = on_security_changed,
#endif
	//code from BLE Fundamentals Lesson 3, Ex 2
	.le_param_updated = on_le_param_updated,
	.le_data_len_updated = on_le_data_len_updated,
};

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static uint32_t pairing_passkey = 0;

uint32_t getpasskey(void)
{
	LOG_INF("Passkey in getter: %06u", pairing_passkey);
	return pairing_passkey;
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Display Passkey %s: %06u", addr, passkey);
	pairing_passkey = passkey;
	#if WL_IN_CHARGER
		displayPairingKey();
	#endif
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s. Should DIsconnect here", addr);

	//JML TODO: disconnect here
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);

	//JML TODO: disconnect here
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	//JML: this is nonessential >>
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	LOG_INF("Received data from: %s", addr);
	//<<JML

	struct cmdHandler_type cmdHandler;
	cmdHandler.route = ROUTE_BLE;
	cmdHandler.len = len; 
	memcpy(&cmdHandler.buf[0], data, len);
	
	while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
	{
		/* message queue is full: purge old data & try again */
		LOG_INF("Purging CmdReq MsgQ");
		k_msgq_purge(&cmd_req_msgq);
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}


#if WL_IN_CHARGER
	K_THREAD_DEFINE(charger_thread_id, CHARGER_STACKSIZE, charger_thread, NULL, NULL, NULL, CHARGER_PRIORITY, 0, 0);
	K_THREAD_DEFINE(coil_reqresp_thread_id, COIL_STACKSIZE, coil_reqresp_thread, NULL, NULL, NULL, COIL_PRIORITY, 0, -1); //only needed with smartcoil
#else 
	K_THREAD_DEFINE(coil_reqresp_thread_id, COIL_STACKSIZE, coil_reqresp_thread, NULL, NULL, NULL, COIL_PRIORITY, 0, -1); //don't start this thread yet
	K_THREAD_DEFINE(charger_thread_id, CHARGER_STACKSIZE, charger_thread, NULL, NULL, NULL, CHARGER_PRIORITY, 0, -1);
#endif

//void enterLowPower_function(struct k_work *work_tem)
void setLowPower(void)
{
	//JML Note: This crashes if coil_reqresp_thread_id is defined above but not otherwise
	//not sure what in this task is a problem

	setBuzzerPeriod(2273, 200);
	setBuzzerPeriod(4546, 200);
	setBuzzerPeriod(9092, 200);
	while(dk_get_buttons()>0)
	{
		LOG_INF("Button still held");
		k_msleep(100);
	}

	LOG_INF("Entering Low Power");
	k_msleep(100);
	
	powerDownRadio();
	//power_down_sensor();
	
	dk_set_leds(0);  //turn off Red, Green, and Blue LED enable
	enable3V3(0);
	
	// /* Above we disabled entry to deep sleep based on duration of
	//  * controlled delay.  Here we need to override that, then
	//  * force entry to deep sleep on any delay.
	//  */
	// pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

	// /* Now we need to go sleep. This will let the idle thread runs and
	//  * the pm subsystem will use the forced state. To confirm that the
	//  * forced state is used, lets set the same timeout used previously.
	//  */

	sys_poweroff();
// 	while(true)
// 	{
// 		k_sleep(K_SECONDS(2));
// 	} 
}




//NOTE: this function is an ISR triggered function and cannot call sleep.  
//All real work needs to be offloaded with a work queue or message queue to another thread
void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttonsOn = button_state & has_changed;
	uint32_t buttonsOff = (~button_state) & has_changed;
 	//LOG_INF("button_state : %08X", button_state);
	//LOG_INF("has_changed  : %08X", has_changed);
	//LOG_INF("buttonsON : %08X", buttonsOn);
	//LOG_INF("buttonsOFF: %08X", buttonsOff);

	struct cmdHandler_type cmdHandler; 

	cmdHandler.len = 0;

	//JML: This allows a button press to authorize the pairing request. Currently not used
	//instead the passkey is read via USB
	// #ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	// if (auth_conn) {
	// 	if (buttonsOn & KEY_PASSKEY_ACCEPT) {
	// 		LOG_INF("Passkey accepted");
	// 		num_comp_reply(true);
	// 	}

	// 	if (buttonsOn & KEY_PASSKEY_REJECT) {
	// 		LOG_INF("Passkey rejected");
	// 		num_comp_reply(false);
	// 	}
	// }
	// #endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	if (buttonsOn & DK_BTN1_MSK ){
		k_timer_start(&btn1_timer, K_MSEC(LONG_BUTTON_PUSH), K_NO_WAIT);
		k_timer_start(&btn1_timer_extra, K_MSEC(EXTRALONG_BUTTON_PUSH), K_NO_WAIT);
		//longPress1Start = k_uptime_get_32();
		//LOG_INF("Button1 press started %08X", longPress1Start);
		button1_held = true;
	}

	if (buttonsOn & DK_BTN2_MSK ){
		k_timer_start(&btn2_timer, K_MSEC(LONG_BUTTON_PUSH), K_NO_WAIT);
		k_timer_start(&btn2_timer_extra, K_MSEC(EXTRALONG_BUTTON_PUSH), K_NO_WAIT);
		//longPress2Start = k_uptime_get_32();
		//LOG_INF("Button2 press started %08X", longPress2Start);
	}


	if (buttonsOff & DK_BTN1_MSK ){ 
		
		button1_held = false;
		if(k_timer_status_get(&btn1_timer)==0)
		{
			//long press timer has not elapsed
			k_timer_stop(&btn1_timer); //stop the timer to prevent long press action
			k_timer_stop(&btn1_timer_extra); //stop the timer to prevent long press action
			LOG_INF("Button1 short press release");
		
			if(action[BUTTON_1_SHORT].len > 0){
				LOG_INF("Button1 short sction started");
				cmdHandler.len = action[BUTTON_1_SHORT].len;
				memcpy(cmdHandler.buf, action[BUTTON_1_SHORT].buf, action[BUTTON_1_SHORT].len);
			} else {
				LOG_INF("No Button1 short press action enabled");
			}
		}
	}

	
	if (buttonsOff & DK_BTN2_MSK ){
		
		if(k_timer_status_get(&btn2_timer)==0)
		{
			//long press timer has not elapsed
			k_timer_stop(&btn2_timer);
			k_timer_stop(&btn2_timer_extra);
			LOG_INF("Button2 short press release");
		
			if(action[BUTTON_2_SHORT].len > 0){
				LOG_INF("Button2 short sction started");
				cmdHandler.len = action[BUTTON_2_SHORT].len;
				memcpy(cmdHandler.buf, action[BUTTON_2_SHORT].buf, action[BUTTON_2_SHORT].len);
			} else {
				LOG_INF("No Button2 short press action enabled");
			}
		}
	}

	if(cmdHandler.len > 0){ 
		cmdHandler.route = ROUTE_LOCAL;
		while(k_msgq_put(&cmd_req_msgq, &cmdHandler, K_NO_WAIT) != 0)
		{
			/* message queue is full: purge old data & try again */
			LOG_INF("Purging CmdReq MsgQ");
			k_msgq_purge(&cmd_req_msgq);
		}
	}
	

}


static void configure_gpio(void)
{
	int err;
	


	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}


	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	init_wl_gpio();

	//JML: not sure why system is waking up without this enabled for either button press
	/* Configure to generate PORT event (wakeup) on button 1 press. */
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_NODELABEL(button0), gpios),
			   NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_NODELABEL(button0), gpios),
			       NRF_GPIO_PIN_SENSE_LOW);


}


void setLEDs( uint8_t red, uint8_t green, uint8_t blue)
{
	//1 turns LED on, 0 turns off, other value leaves LED as is

	if (red == 1) {dk_set_led_on(RED_LED);}
	if (red == 0) {dk_set_led_off(RED_LED);}

	if (green == 1) {dk_set_led_on(GREEN_LED);}
	if (green == 0) {dk_set_led_off(GREEN_LED);}
	
	if (blue == 1) {dk_set_led_on(BLUE_LED);}
	if (blue == 0) {dk_set_led_off(BLUE_LED);}
}






static int flash_init()
{
	int err = 0;
	struct flash_pages_info info;


	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_WRN("Flash device %s is not ready\n", fs.flash_device->name);
		return -ENODEV;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	err = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (err) {
		LOG_WRN("Unable to get page info\n");
		return err;
	}


	fs.sector_size = info.size;
	fs.sector_count = 2U;
	err =nvs_mount(&fs);
	if (err) {
		LOG_WRN("Flash Mount failed\n");
		return err;
	}
	return 0;	
}



int saved_settings_write(uint16_t id, const void* buf, size_t len)
{
	int err;
	err = nvs_delete(&fs, id);
	if (err == 0) {
		LOG_INF("Item at id %d was deleted\n",id);
	}else{
		LOG_INF("Item at id %d  was not deleted", id);
	}

	err = nvs_write(&fs, id, buf, len);
	if (err == 0) {
		LOG_INF("Item at id %d already exists, delete first\n",id);
	}else if (err < 0){
		LOG_INF("Item at id %d  was not found\n", id);
	} else{
		LOG_INF("Item at id %d  was written:\n", id);
		LOG_HEXDUMP_INF(buf, len, "Data:");
	}
	return err;
}

int saved_settings_read(uint16_t id, const void* buf, size_t len)
{

	int err = nvs_read(&fs, id, buf, len);
	if (err > 0) {
		LOG_INF("Item at id %d was found.",id);
		LOG_INF("%d Bytes read\n",err);
		LOG_HEXDUMP_INF(buf, len, "Data:");
	}else{
		LOG_INF("Item at id %d  was not found\n", id);
	}
	return err;
}


void ble_start_advertising_pairing_mode(void)
{
	//stop any ongoing advertising first
	ble_stop_advertising();
	
	LOG_INF("BLE-Start Advertising in Pairing Mode");
	pairing_mode = true;
	k_work_submit(&start_adv_work);

	isAdvertising = true;
	k_timer_start(&adv_timer, K_SECONDS(ADV_TIMEOUT), K_NO_WAIT);
}


void ble_start_advertising(void){
	
	pairing_mode = false;
	LOG_INF("BLE-Start Advertising");
	k_work_submit(&start_adv_work);

	isAdvertising = true;
	k_timer_start(&adv_timer, K_SECONDS(ADV_TIMEOUT), K_NO_WAIT);
}

void ble_stop_advertising(void){
	LOG_INF("BLE-Stop Advertising");

	k_timer_stop(&adv_timer); //if the timer is still running (i.e. this wasn't called by timeout handler), then stop it

	int err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("Advertising failed to stop(err %d)", err);
		return;
	}
	isAdvertising = false;
}



int main(void)
{
	int err = 0;
	//char *device_name = "WL_1234";

	

	LOG_INF("Entered Main");
	configure_gpio();
	enable3V3( !(modeLED & LED_AUDIO_OFF) );

	err = uart_init();
	if (err) {
		error();
	}


	
	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_WRN("Failed to register authorization callbacks.\n");
			return;
		}
		
		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_WRN("Failed to register authorization info callbacks.\n");
			return;
		}
		LOG_INF("Bluetooth Authorization Callbacks registered");
	}





	err = bt_enable(NULL);
	if (err) {
		error();
	}
	LOG_INF("Bluetooth initialized");
	
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	
	// err = bt_set_name(device_name);
    // if (err) {
    //     // Handle error if setting the name failed
    //     LOG_ERR("Failed to set Bluetooth device name: %d\n", err);
    // } else {
    //     LOG_INF("Bluetooth device name set to: %s\n", device_name);
    // }



	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize Nordic UART service (err: %d)", err);
		return;
	}

	k_sem_give(&ble_init_ok);

	init_adc();
	spi_init();
	initIMU();
	LOG_INF("SPI initialized");

	err = flash_init();
	if (err){
	 	LOG_ERR("Flash not initialized. Will not be able to store or read saved settings!(err: %d)", err);
	} else {
		LOG_INF("Flash initialized");
	}

	
	initRadioConfig();
	LOG_INF("MedRadio initialized");

	k_sleep(K_MSEC(100)); //pauses for logging to catch up


	setBuzzerPeriod(9092, 200);
	dk_set_led_on(BLUE_LED);
	k_sleep(K_MSEC(300)); 
	dk_set_led_off(BLUE_LED);
	
	setBuzzerPeriod(4546, 200);
	dk_set_led_on(GREEN_LED);
	k_sleep(K_MSEC(300)); 
	dk_set_led_off(GREEN_LED);

	setBuzzerPeriod(2273, 200);
	dk_set_led_on(RED_LED);
	k_sleep(K_MSEC(300)); 
	dk_set_led_off(RED_LED);

	
	ble_start_advertising();
	
	k_sleep(K_MSEC(100)); //pauses for logging to catch up

	for (uint8_t i=0; i<MAX_ACTIONS; i++){ 
		err = saved_settings_read(ACTION_SETTINGS_ID+i, &action[i], sizeof(action[i]));
		if(err > 0)
		{
			LOG_INF("Action settings found in flash, %d bytes read", err);

		} else{
			LOG_INF("No action settings found in flash %d", err);
				if( i==0 || i==1)
				{
					action[i].len = 0;
					action[i].source = 0;
				} else {
					action[i].buf[0] = 0xFF;
					action[i].buf[1] = 0x43;
                    action[i].buf[2] = 0x03;
                    action[i].len= 3;
					action[i].source= 0;

				}
		}
	}

	//Just blink LED infinitely, everything else happens in separate threads
	//uint8_t cnt = 0;
	for (;;) {
		if(modeLED & BLUE_LED_BLE_ADV && (isAdvertising ||  isReady )) {dk_set_led_on(BLUE_LED);}
		k_sleep(K_MSEC(100));
		if(!isReady ){
			if(modeLED & BLUE_LED_BLE_ADV){dk_set_led_off(BLUE_LED);}
			
			if(isAdvertising && pairing_mode){k_sleep(K_MSEC(100));}  //fast blink in pairing mode
			else {k_sleep(K_MSEC(900));}  //slow blink otherwise
		} 
	}
}



void implant_reqresp_thread(void)
{
	/* Don't go any further until MedRadio and BLE are initialized */
	k_sem_take(&medradio_init_ok, K_FOREVER);
	k_sem_take(&ble_init_ok, K_FOREVER);

	LOG_INF("Starting implant request-response thread");

	struct medRadio_type medRadio; 
	const uint8_t *bufPtr = &medRadio.buf;
	struct cmdHandler_type cmdHandler; 
	uint16_t timeout;
	uint8_t source = 0;
	uint8_t response_len, response_type;
	int err = 0;

	for (;;) {
		response_len = 0, response_type = 0;

		/* Wait indefinitely for request coming from BLE or UART or locally */
		err = k_msgq_get(&imp_req_msgq, &medRadio, K_MSEC(100)); //wait up to 100ms, then turn off LEDs
		if(modeLED & RED_LED_RADIO_ERROR){dk_set_led_off(RED_LED); }
			
		if(modeLED & GREEN_LED_RADIO_RESPONSE){dk_set_led_off(GREEN_LED); }
			
		if(modeLED & GREEN_LED_RADIO_RESPONSE || modeLED & RED_LED_RADIO_ERROR)
		{
			dk_set_led_off(BLUE_LED); //turn off blue so green and red look correct
		}
		if (err < 0){  //If no message, wait indefinitely
			k_msgq_get(&imp_req_msgq, &medRadio, K_FOREVER);
		}

		source = medRadio.source;
		timeout = getMedRadioTimeout(); 
		LOG_INF("Sending out MedRadio (Request Route %d) using Timeout: %d", source, timeout);
		k_msgq_purge(&imp_resp_msgq); //purge any incoming messages responses prior to sending new request
		sendRadioPacket(bufPtr, medRadio.len);  
		medRadio.len = 0;
		/* Wait (not indefinitely) for message to be sent to implant*/
		err = k_sem_take(&medradio_sent, K_MSEC(timeout)); //JML should this be a shorter timeout
		if (err == 0)
		{
			/* Wait (not indefinitely) for response from implant*/
			err = k_msgq_get(&imp_resp_msgq, &medRadio, K_MSEC(timeout));
		}
				
		if (err!=0 || medRadio.len < 2 ){ 
			if(modeLED & RED_LED_RADIO_ERROR){dk_set_led_on(RED_LED); }
			//JML TODO: send an appropriate timeout response)
			LOG_INF("MedRadio Timeout");
			idleMedRadio();

			cmdHandler.len = NON_PAYLOAD_RESP_BYTES;
			cmdHandler.buf[INDEX_RESP_SYNC ] = SYNC_BYTE;
			cmdHandler.buf[INDEX_RESP_TYPE] = RESP_MEDRADIO_TIMEOUT;
			cmdHandler.buf[INDEX_RESP_LEN] = cmdHandler.len;
		} else {
			if(modeLED & GREEN_LED_RADIO_RESPONSE){ dk_set_led_on(GREEN_LED); }
			LOG_INF("MedRadio Receive back in Implant thread, len=%d", medRadio.len);
			LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "Response");
			//scrap addr, len bytes from radio msg, maintain RSSI,LQI
			cmdHandler.len = medRadio.len + NON_PAYLOAD_RESP_BYTES - 2; 
			cmdHandler.buf[INDEX_RESP_SYNC ] = SYNC_BYTE;
			cmdHandler.buf[INDEX_RESP_TYPE] = RESP_SUCCESS;
			cmdHandler.buf[INDEX_RESP_LEN] = cmdHandler.len;
			memcpy(&cmdHandler.buf[INDEX_RESP_PAYLOAD], &medRadio.buf[2], medRadio.len-2);
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
				while(k_msgq_put(&charger_resp_msgq, &cmdHandler, K_NO_WAIT) != 0)
				{
					/* message queue is full: purge old data & try again */
					LOG_INF("Purging ChargerResp msg_q");
					k_msgq_purge(&charger_resp_msgq);
				}
				LOG_INF("ChargerResponse k_msg_q_put complete");
				break;
			default:
				LOG_INF("Undefined Source");

		}
		
		


		

		
	}
}

//currently, 65*10*2 = 1300bytes for MSGQ
K_THREAD_DEFINE(implant_reqresp_thread_id, IMP_STACKSIZE, implant_reqresp_thread, NULL, NULL, NULL, IMP_PRIORITY, 0, 0);




////////  IMU Sensorn: move to own file?


// static const struct device *sensor = DEVICE_DT_GET(DT_INST(0,st_ism330is));

// void enable_sensor(uint8_t enable)
// {
// 	struct sensor_value odr_attr;
// 	int err = 0;
// 	if (enable)
// 	{
// 		odr_attr.val1 = 26; 
// 		k_thread_resume(sensor_thread_id);
// 	}
// 	else {
// 		odr_attr.val1 = 0; 
// 		k_thread_suspend(sensor_thread_id);
// 	}

// 	odr_attr.val2 = 0;
// 	err = sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
// 			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
// 	if (err != 0) {
// 		LOG_WRN("Cannot set sampling frequency for accelerometer.\n");
// 		return;
// 	}
// }

// void power_down_sensor(void)
// {
// 	enable_sensor(0);
// }



// void sensor_thread(void)
// {
// 	LOG_INF("Starting sensor thread");

// 	//Config IMU Sensor

// 	if(!device_is_ready(sensor))
// 	{
// 		LOG_INF("Could not get sensor device");
// 		return;
// 	}	

// 	int err = 0;
// 	struct sensor_value accel[3];
// 	struct sensor_value gyro[3];
// 	struct sensor_value odr_attr;
// 	float temp;
// 	uint8_t command[3];
// 	static uint8_t prevcommand[3];
// 	uint8_t i;
// 	odr_attr.val1 = 0; 
// 	odr_attr.val2 = 0;

// 	//JML: don't think this is setting accel in Low power mode
// 	err = sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
// 			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
// 	if (err != 0) {
// 		LOG_WRN("Cannot set sampling frequency for accelerometer.\n");
// 		return;
// 	}

// 	// odr_attr.val1 = 26; 
// 	// odr_attr.val2 = 0;
// 	// err = sensor_attr_set(sensor, SENSOR_CHAN_GYRO_XYZ,
// 	// 		SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
// 	// if (err != 0) {
// 	// 	LOG_WRN("Cannot set sampling frequency for gyro.\n");
// 	// 	return;
// 	// }

// 	k_sleep(K_MSEC(3000)); //JML pause for test in main thread

// 	k_thread_suspend(sensor_thread_id);

// 	for (;;) {
// 		err = sensor_sample_fetch(sensor);
// 		if (err == 0) {
// 			err = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
// 			if (err == 0){
// 				// LOG_INF("accel X %d.%d Y %d.%d Z %d.%d\n", 
// 				// 	accel[0].val1,accel[0].val2,
// 				// 	accel[1].val1,accel[1].val2,
// 				// 	accel[2].val1,accel[2].val2);
// 				for(i=0; i<3; i++)
// 				{
// 					temp = (accel[i].val1 + accel[i].val2/1000000)*255/9.8; 
// 					if(temp > 255)
// 						temp = 255;
// 					else if (temp < 10) //arbitrary threshold
// 						temp = 0;	
// 					command[i] = temp;	
// 				}
				
// 			}else{
// 				LOG_INF("Sensor channel get accel failed: %d\n", err);
// 			}

// 			// err = sensor_channel_get(sensor, SENSOR_CHAN_GYRO_XYZ,gyro);
// 			// if (err == 0){
// 			// 	// LOG_INF("gyro  X %d.%d Y %d.%d Z %d.%d\n", 
// 			// 	// 	gyro[0].val1,gyro[0].val2,
// 			// 	// 	gyro[1].val1,gyro[1].val2,
// 			// 	// 	gyro[2].val1,gyro[2].val2);

// 			// }else{
// 			// 	LOG_INF("Sensor channel get gyro failed: %d\n", err);
// 			// }
// 		} else{
// 			LOG_INF("Sensor fetch failed: %d\n", err);
// 		}

// 		if(prevcommand[0] != command[0] || prevcommand[1] != command[1] || prevcommand[2] != command[2] ){  
// 			//JML TODO: this should call accelerometer action, rather than doing direct MedRadio action 
// 		 	struct medRadio_type medRadio;
// 		 	medRadio.source = SOURCE_SENSOR;

// 			medRadio.len = sizeof(action3Msg); 
// 			memcpy(medRadio.buf, action3Msg, medRadio.len);
// 			for(i=0; i<2; i++)
// 			{
// 				medRadio.buf[i+8]=command[i];
// 				prevcommand[i] = command[i];
// 			}

// 		 	if(medRadio.len > MAX_MEDRADIO_PAYLOAD)
// 		 	{
// 		 		LOG_INF("Packet too long for MedRadio");
// 		 		return;
// 		 	}
		
// 		 	LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "TX MedRadio Packet");
// 		 	k_msgq_put(&imp_req_msgq, &medRadio, K_NO_WAIT);
// 		}


// 		// if(abs(gyro[2].val1*57) > 90 ){  //rotate arund Z axis faster than 90 deg/s 
// 		// 	struct medRadio_type medRadio;
// 		// 	medRadio.route = ROUTE_LOCAL;

// 		// 	if(gyro[2].val1 > 0) //+Z
// 		// 	{
// 		// 		medRadio.len = sizeof(action1Msg); 
// 		// 		memcpy(medRadio.buf, action1Msg, medRadio.len);
// 		// 		LOG_INF("Rotation Threshold Speed Met - Network On");
// 		// 	}
// 		// 	if(gyro[2].val1 < 0)  //-Z
// 		// 	{
// 		// 		medRadio.len = sizeof(action2Msg); 
// 		// 		memcpy(medRadio.buf, action2Msg, medRadio.len);
// 		// 		LOG_INF("Rotation Threshold Speed Met - Network Off");
// 		// 	}

// 		// 	if(medRadio.len > MAX_MEDRADIO_PAYLOAD)
// 		// 	{
// 		// 		LOG_INF("Packet too long for MedRadio");
// 		// 		return;
// 		// 	}
		
// 		// 	//clear MedRadio local status LEDs before sending out message
// 		// 	dk_set_leds_state(DK_NO_LEDS_MSK, DK_LED3_MSK|DK_LED4_MSK);
// 		// 	LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "TX MedRadio Packet");
// 		// 	k_msgq_put(&imp_req_msgq, &medRadio, K_NO_WAIT);
// 		// }


// 		k_sleep(K_MSEC(SENSOR_POLL_INTERVAL));
// 	}
// }





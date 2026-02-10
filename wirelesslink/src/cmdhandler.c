#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include "cmdhandler.h"
#include "charger.h"
#include "wlgpio.h"
#include "cc1101radio.h"
#include "audio.h"

#include "uart_async_adapter.h"

#include <zephyr/logging/log.h>

#include "imu.h"


#define LOG_MODULE_NAME cmdhandler
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Packet format
// header                                         payload
// SYNC   CMD     LEN                              ...
// 0xFF seebelow  lengthOfPacket(up to 255)    up to 252 bytes
// multiple bytes are presented Little Endian



//                                   CMD       bytesToWL  bytesFromWL  
#define GET_PRODUCT_ID              (0x20) //    0         4           Returns HW and SW revision numbers 

//WIRELESS LINK
#define WL_ENTER_DFU_MODE           (0x21) //  Not yet implemented.  Hold button1 during reset to enter serial recovery
#define WL_ERASE_BONDS              (0x22) //    0         1         returns erasebonds result
#define WL_RESET                    (0x23) //    0         0            Reset WL
#define WL_START_PAIRING            (0x1F) //    0         0           Start advertising, allowing new devices
#define WL_GET_PASSKEY              (0x1E) //    0         4           uint32 for 6 digit number
#define WL_GET_BONDS                (0x1D) //    0         31          Returns number of existing bonds from 0-5, and the 6-byte addresses of up to 5 paired devices
//These commands allow accelerating long flash read/write commands with PM
//WL Image is currently only 512 bytes 
#define WL_WRITE_IMAGE              (0x24) //    4+N       0           Send 4 Address Bytes followed by N (up to 248 for USB, 237 for BLE ) bytes to write to WL image
#define WL_READ_IMAGE               (0x25) //    5         N           Send 4 Address Bytes and N (up to 252 for USB, 241 for BLE).  Returns N bytes of WL inage starting from addr 
#define WL_PMBOOT_WRITE             (0x26) //    7         0           Send 4 addr bytes + 2 pgsize bytes + 1 sector byte.  Copies WL image to PM flash
#define WL_PMBOOT_READ              (0x27) //    6         0           Send 4 addr bytes + 2 size bytes.  Copies PM flash to WL image
#define WL_PMFILE_READ              (0x28) //    7         0           Send 4 addr bytes + 2 size bytes + 1 file byte.  Copies PM file data to WL image
#define WL_PMSCRIPT_WRITE           (0x29) //   Not yet implemented


#define WL_SET_BUTTON_ACTION        (0x30) //    1+N       0           Send 1 action index + N packet bytes to execute. Action index (0=button1 short, 1=button2 short, 2=button1 long, 3=button2 long) 
#define WL_GET_BUTTON_ACTION        (0x31) //    1         N           Read back for above
#define WL_SET_BUTTON_ACTION_TEMP   (0x32) //  Not yet implemented      temporary version of above (not saved in flash)
#define WL_SET_IMU_MODE             (0x33) //    1         0           Currently enables or disables accel task.  Sets one of the predetermined IMU modes (accel, gyro, accel+gyro, accel+gyro+ispu) 
#define WL_GET_IMU_MODE             (0x34) //  Not yet implemented.     Read back for above
#define WL_SET_IMU_MODE_TEMP        (0x35) //  Not yet implemented.    temporary version of above (not saved in flash)
#define WL_GET_IMU_DATA             (0x36) //  Not yet implemented.     Reads available IMU data
#define WL_SET_IMU_ACTION           (0x37) //  Not yet implemented.    Sets the IMU condition and payload that will be sent to implant if condition is met
#define WL_GET_IMU_ACTION           (0x38) //  Not yet implemented.    Read back for above
#define WL_SET_IMU_ACTION_TEMP      (0x39) //  Not yet implemented.    temporary version of above (not saved in flash)
#define WL_GET_IMU_ACCEL            (0x3A) //  Not yet implemented.    Read an IMU register directly (Low Level)
#define WL_GET_IMU_GYRO             (0x3B) //  Not yet implemented.    Write an IMU register directly (Low Level)
#define WL_SET_LED_MODE             (0x3C) //    1         0           Set one of the predetermined LED modes: BLUE_LED_BLE_ADV=BIT0, GREEN_LED_RADIO_RESPONSE=BIT1, RED_LED_RADIO_ERROR=BIT2, LED_CHARGER=BIT3, LED_MANUAL=BIT6, LED_AUDIO_OFF=BIT7
#define WL_GET_LED_MODE             (0x3D) //    0         1            Read back for above
#define WL_SET_LED_MODE_TEMP        (0x3C) //  Not yet implemented. 
#define WL_SET_LEDS                 (0x3F) //    3         0           Force the LEDs to a specific state.  LED Mode must be LED_MANUAL.  Red, Green, Blue
#define WL_GET_LEDS                 (0x40) //  Not yet implemented.    Read the current LED state
#define WL_GET_CHARGE_STATUS        (0x41) //    0         1           Read MCP73833 charging status.  BIT0=PowerGood (on USB), BIT1=STAT1 (charging), BIT2=STAT2 (done charging)
#define WL_SET_BUZZER_PERIOD        (0x42) //   N*4+1      1           Send 1 NumberOfTones(N) byte  + N*(2bytes note period + 2bytes note length) (in us).  Returns N
#define WL_ENTER_LOW_POWER          (0x43) //    0         0           Enter Low Power mode
#define WL_BLE_START_ADV            (0x44) //    0         0           Start BLE advertising again.  Device will advertise for first 30s after startup automatically
#define WL_BLE_STOP_ADV             (0x45) //    0         0           Stop advertising
#define WL_BLE_GET_ADC              (0x46) //    0         2           Returns 2 bytes (battery voltage in mV)

//ACCESS POINT                       CMD
#define AP_SEND_RECV_MSG            (0x47) //    N         M+2         Send the implant a message and await response (or timeout).  Include everything to send over radio except address.  Returns radio response except address (appends RSSI/CRC/LQI).    
#define AP_SET_RADIO_SETTINGS	    (0x48) //    7         7           Write the MedRadio settings: AP/PM addresses and fixed channel, WOR settings and timeouts (saved to flash)
#define AP_GET_RADIO_SETTINGS  	    (0x49) //    0         7           localAddr, RemoteAddr, Chan, TXpower, WORInt, RXtimeout, Retries
#define AP_SET_RADIO_SETTINGS_TEMP  (0x4A) //    7         7           Write the MedRadio settings: AP/PM addresses and fixed channel, WOR settings and timeouts (temporary)
#define AP_CLEAR_CHANNEL_SEARCH     (0x4B) //    1         2           Send dwell time (1 byte) in 10ms increments (1=10ms, 250=25s) Returns clearest channel + RSSI (int8_t)
#define AP_SET_SESSION_TIME         (0x4C) //    1         0           Set time since session lasts (0 to disable Clear Channel Search), 5 for MedRadio protocol
#define AP_GET_SESSION_TIME_LEFT    (0x4D) //    0         2           Get time since last MedRadio RX 
#define AP_SET_SESSION_MAINTAIN     (0x4E) //    1         0           1=maintain (if sessiontime>0) 0, don't maintaitn Not yet implemented.    Read back for above
#define AP_ENCRYPTION               (0x4F) //    1          0          Enable/disable encryption
#define AP_RESTORE_RADIO            (0x50) //    1          0          0=Bootloader, 1=App

//COIL DRIVE
#define CD_SET_PERIOD               (0x52) //   4         0           Set coil period in ns (nominally 3.5kHz for NNP)     
#define CD_GET_PERIOD               (0x53) //   0         4           Set coil period in ns (nominally 3.5kHz for NNP)  
#define CD_START_DRIVE              (0x54) //   0         0           Turn on Coil (if DCDC on)    
#define CD_STOP_DRIVE               (0x55) //   0         0           Turn off Coil (if DCDC on)  
#define CD_GET_COIL_POWER           (0x56) //   0         4           Read coil drive power 2 current bytes (int16, mA) + 2 voltage bytes (uint16, mV) 
#define CD_GET_CHARGER_POWER        (0x57) //   0         4           Read system power     2 current bytes (int16, mA) + 2 voltage bytes (uint16, mV)  Read DC voltage input of coil drive   
#define CD_GET_THERMISTOR           (0x58) //   0         2           Read Thermistor (not for MCU coil)  2 bytes (in 0.1degC)
#define CD_GET_COIL_CONNECT         (0x59) //   Not yet implemented.  Read coil connection status (not for MCU coil) 
#define CD_GET_SERIAL               (0x5E) //   Not yet implemented 
#define CD_SEND_SERIAL              (0x5F) //   Not yet implemented  
#define CD_START_DCDC               (0x62) //   0         0           Start DCDC converter (supplies Coil Drive)    
#define CD_STOP_DCDC                (0x63) //   0         0           Stop DCDC converter (supplies Coil Drive)          
#define CD_SET_DCDC_VOLT            (0x64) //   2         0           Set DCDC voltage: 2 bytes (uint16, mV)  
#define CD_KEEP_COIL_ON             (0x65) //   0         0           Feeds the watchdog timer to keep the coil on.  Ohterwise reset will occur 1 min after turning coil on  
#define DISPLAY_SET                 (0x6D) //    1+4+N                clear screen, len1, [str1 ...], len2, [str2 ...], len3, [str3 ...], len4, [str4 ...]

#define CD_GET_RTC                  (0x70) //   0         7           sec, min, hour, weekday, day, month, year
#define CD_SET_RTC                  (0x71) //   7         0           sec, min, hour, weekday, day, month, year

#define CD_GET_PARAMS              (0x72)
#define CD_SET_PARAMS             (0x73)

//SMART COIL (coil UART)
//commands forward cmd and payload on the smartcoil and back
#define COIL_GET_THERMISTOR_1       (0x78) 
#define COIL_GET_THERMISTOR_2       (0x79) 
#define COIL_GET_PRODUCT_ID         (0x80) 
#define COIL_ACCEL_OFF              (0x7A)     
#define COIL_ACCEL_ON               (0x7B)     
#define COIL_GET_ACCEL              (0x7C)       
#define COIL_GET_PRODUCT_ID         (0x80)   
#define COIL_SET_LED                (0x81)           

//These SD card commands are for testing and may be removed
#define SD_WRITE                    (0x90) //   1+N      1          1 lenToWrite (N) byte + N data bytes (up to 251). Returns 1 if successful, 0 otherwsie
#define SD_READ                     (0x91) //    5       N          4 addr bytes + 1 lenToRead (N) byte.  Returns N bytes (up to 252)
#define SD_INIT                     (0x92) //    0       0          Init SD Card
#define SD_DIR                      (0x93) //    0       0          Dir info visible in debug terminal only
#define SD_GET_LOG_LENGTH           (0x94) //    0       4          4 length byte (uint32)
#define SD_FLUSH_LOG                (0x95) //    0       1          Returns 1 if successful, 0 otherwsie

#define CHG_GET_CHARGEMODE          (0xA0) //    0       1         
#define CHG_SET_CHARGEMODE          (0xA1) //    1       0

//for testing only
#define PURGE_MSG_QUEUES            (0xF0) 

//These PM OD commands are for testing and will be removed
#define TEST_READ_SDO               (0xF1)
#define TEST_WRITE_SDO              (0xF2)
#define TEST_NMT                    (0xF3)


//PKT format
//0xFF, cmd, len (payload+header), payload  
//Should add checksum 

#if WL_IN_CHARGER
    uint8_t modeLED = LED_DEFAULT_MODE_CHARGER;
#else
    uint8_t modeLED = LED_DEFAULT_MODE_WL;
#endif

struct action_type action[MAX_ACTIONS];

K_MSGQ_DEFINE(cmd_req_msgq, sizeof(struct cmdHandler_type), MSG_DEPTH, 4);
K_MSGQ_DEFINE(cmd_resp_msgq, sizeof(struct cmdHandler_type), MSG_DEPTH, 4);


//uint8_t *packet, uint8_t len, uint8_t route)

#define PMBOOT_FIRST_PKT_BYTES 48 //512%PM_PKT_BYTES and <=60-1-7=52.   originally 12
#define PMBOOT_PKT_BYTES       58 //<= 60-1=59 //originally 50
#define PM_PAGE_SIZE          512
#define PMFILE_PKT_BYTES       48 //maximum bytes to read at once wiht NNP file read command 60-10byte header-2byte size
uint8_t image[512]; 

#define DISP_STR_LEN 80

void purge_msg_queues(void);


void command_handler_thread(void)
{
    static uint8_t response[MAX_CMD_RESPONSE_LEN]; 
    uint8_t cmd_payload_len = 0;
    uint8_t response_len, response_type;
    uint8_t actionindex = 0;
    uint32_t addr;
    uint8_t len, route;
    uint32_t result;
    uint8_t buf[8]; //temporary storage buffer
    
    int err;

    static struct cmdHandler_type cmdhandler; 
    struct medRadio_type medRadio;
    struct smartCoil_type smartCoil;

    bool lowpower = false;


                uint32_t k[4] = {0x1EA098D4, 0x8FAECA4B, 0x2BBCF0DA, 0xFA12E8E4};
                uint32_t p[16];
                uint8_t b;
                int8_t n;

    for (;;) {
        medRadio.len = 0;
        response_len = 0;
        response_type = 0;
        lowpower = false;
        
        k_msgq_get(&cmd_req_msgq, &cmdhandler, K_FOREVER);
        //enableDebug0(1); //JMLDEBUG
        route = cmdhandler.route;
        if (cmdhandler.buf[INDEX_CMD_SYNC] !=  SYNC_BYTE)
        {
            LOG_INF("Comand Handler First Byte is not Sync Byte");
            continue;
        }
        LOG_INF("Command Handler Length: 0x%02X", cmdhandler.len);
        if (cmdhandler.len != cmdhandler.buf[INDEX_CMD_LEN])
        {
            LOG_INF("Comand Handler Bad Length");
            continue;
        }
        cmd_payload_len = cmdhandler.len - NON_PAYLOAD_RESP_BYTES;
        
        LOG_HEXDUMP_INF(cmdhandler.buf, cmdhandler.len, "Command Handler");
        switch(cmdhandler.buf[INDEX_CMD_COMMAND]) //CMD
        {
            case GET_PRODUCT_ID:
                if(cmd_payload_len == 0)
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 4;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = (uint8_t) (SW_REV);
                    response[INDEX_RESP_PAYLOAD + 1] = (uint8_t) (SW_REV >> 8);
                    response[INDEX_RESP_PAYLOAD + 2] = (uint8_t) (HW_REV);
                    response[INDEX_RESP_PAYLOAD + 3] = (uint8_t) (HW_REV >> 8);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
           
            case WL_START_PAIRING:
                if(cmd_payload_len == 0)
                {
                    ble_start_advertising_pairing_mode();
                    response_type = RESP_SUCCESS;
                    response_len = NON_PAYLOAD_RESP_BYTES;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case WL_GET_PASSKEY:
                if(cmd_payload_len == 0)
                {
                    result = getpasskey();
                    response_type = RESP_SUCCESS;
                    memcpy(&response[INDEX_RESP_PAYLOAD], &result, sizeof(result));
                    response_len = NON_PAYLOAD_RESP_BYTES + 4;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            case WL_GET_BONDS:
                if(cmd_payload_len == 0)
                {
                    response[INDEX_RESP_PAYLOAD] = get_bonded_devices(&response[INDEX_RESP_PAYLOAD+1]);
                    response_type = RESP_SUCCESS;
                    response_len = NON_PAYLOAD_RESP_BYTES + 31;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            case WL_ERASE_BONDS:
                if(cmd_payload_len == 0)
                {
                    response[INDEX_RESP_PAYLOAD] = erasebonds();
                    response_type = RESP_SUCCESS;
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case WL_RESET:
                if(cmd_payload_len == 0)
                {
                    resetWL();
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case WL_WRITE_IMAGE:
                if(cmd_payload_len >= 4)
                {
                    memcpy(&addr, &cmdhandler.buf[INDEX_CMD_PAYLOAD], 4);
                    len = cmd_payload_len - 4; //data length after subtracting off 4 address bytes 

                    if (addr + len <= sizeof(image)) 
                    {
                        memcpy(&image[addr], &cmdhandler.buf[INDEX_CMD_PAYLOAD + 4], len);
                        response_len = NON_PAYLOAD_RESP_BYTES;
                        response_type = RESP_SUCCESS;
                        break;
                    }
                }
                //invalid parameters
                response_len = NON_PAYLOAD_RESP_BYTES;
                response_type = RESP_INVALID_PARAMETERS;
                break;
                

            case WL_READ_IMAGE:
                if(cmd_payload_len == 5) //4 addr bytes + 1 len byte
                {   
                    memcpy(&addr, &cmdhandler.buf[INDEX_CMD_PAYLOAD], 4);
                    len = cmdhandler.buf[INDEX_CMD_PAYLOAD+4];

                    if (addr + len <= sizeof(image) && len <= sizeof(response)-NON_PAYLOAD_RESP_BYTES)
                    {
                        response_len = NON_PAYLOAD_RESP_BYTES + len;
                        response_type = RESP_SUCCESS;
                        memcpy(&response[INDEX_RESP_PAYLOAD], &image[addr], len);
                        break;
                    }
                }
                //invalid parameters
                response_len = NON_PAYLOAD_RESP_BYTES;
                response_type = RESP_INVALID_PARAMETERS;    
                break;

            case WL_PMBOOT_WRITE:
                LOG_INF("PM BOOT WRITE");
                if(cmd_payload_len == 7  ) //4 addr bytes + 2 pgsize bytes + 1 sector byte
                {   
                    pmboot_write_supervisor(&cmdhandler.buf[INDEX_CMD_PAYLOAD], NULL, 0, &medRadio.buf[0], &medRadio.len);
                    response_len = AWAITING_RESPONSE;
                    response_type =  RESP_LOCAL_PMBOOT_WRITE;
                    //response is sent when final PM write is complete
                }
                else if(cmd_payload_len == 0  ) //get status
                {   
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = pmboot_write_supervisor(NULL, NULL, 0, NULL, NULL);
                }
                else {
                    //invalid parameters
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }   
                break;

            case WL_PMBOOT_READ:
                if(cmd_payload_len == 6  ) //4 addr bytes + 2 size bytes 
                {   
                    pmboot_read_supervisor(&cmdhandler.buf[INDEX_CMD_PAYLOAD], NULL, 0, &medRadio.buf[0], &medRadio.len);
                    response_len = AWAITING_RESPONSE;
                    response_type =  RESP_LOCAL_PMBOOT_READ;
                    //response is sent when final PM read is complete
                }
                else if(cmd_payload_len == 0  ) //get status
                {   
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = pmboot_read_supervisor(NULL, NULL, 0, NULL, NULL);
                }
                else {
                    //invalid parameters
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }   
                break;

            case WL_PMFILE_READ:
                if(cmd_payload_len == 7  ) //4 addr bytes + 2 size bytes + 1 file byte
                {   
                    pmfile_read_supervisor(&cmdhandler.buf[2], NULL, 0, &medRadio.buf[0], &medRadio.len);
                    response_len = AWAITING_RESPONSE;
                    response_type =  RESP_LOCAL_PMFILE_READ;
                    //response is sent when final PM read is complete
                }
                else if(cmd_payload_len == 0 ) //get status
                {   
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = pmfile_read_supervisor(NULL, NULL, 0, NULL, NULL);
                }
                else {
                    //invalid parameters
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }   
                break;
            
            case WL_PMSCRIPT_WRITE:
                 response_len = NON_PAYLOAD_RESP_BYTES;
                 response_type = RESP_INVALID_PARAMETERS;  
                 LOG_INF("Not yet implemented");


                break;
                
            case AP_SEND_RECV_MSG:
                if(cmd_payload_len > MAX_MEDRADIO_PAYLOAD)
                {
                    LOG_INF("Packet too long for MedRadio");
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                    break;
                }
                if(cmd_payload_len < MIN_MEDRADIO_PAYLOAD)
                {
                    LOG_INF("Packet too short for MedRadio");
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                    break;
                }
                
                response_len = AWAITING_RESPONSE;
                medRadio.len = cmd_payload_len; //only the payload 
                memcpy(&medRadio.buf[0], &cmdhandler.buf[INDEX_CMD_PAYLOAD], medRadio.len); //copy cmdhandler buf to medRadio buf
                break;

            case AP_SET_RADIO_SETTINGS:
            case AP_SET_RADIO_SETTINGS_TEMP:
                if(cmd_payload_len != 7)
                {
                    //bad number of settings
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                    break;
                }
    
                updateMedRadioLocalAddress( cmdhandler.buf[INDEX_CMD_PAYLOAD + 0]);
                updateMedRadioRemoteAddress(cmdhandler.buf[INDEX_CMD_PAYLOAD + 1]);
                updateMedRadioChannel(      cmdhandler.buf[INDEX_CMD_PAYLOAD + 2]);
                updateMedRadioTXPower(      cmdhandler.buf[INDEX_CMD_PAYLOAD + 3]);
                updateMedRadioWORInterval(  cmdhandler.buf[INDEX_CMD_PAYLOAD + 4]);
                updateMedRadioRXTimeout(    cmdhandler.buf[INDEX_CMD_PAYLOAD + 5]);
                updateMedRadioRetries(      cmdhandler.buf[INDEX_CMD_PAYLOAD + 6]);

                if (cmdhandler.buf[INDEX_CMD_COMMAND]==AP_SET_RADIO_SETTINGS){
                    copyRadioSettings(&buf[0]);
                    saved_settings_write(RADIO_SETTINGS_ID, buf, 7);
                } 
                
                response_len = NON_PAYLOAD_RESP_BYTES + 7;
                response_type = RESP_SUCCESS;
                copyRadioSettings(&response[INDEX_RESP_PAYLOAD]);
                break;
        
            case AP_GET_RADIO_SETTINGS:
                if(cmd_payload_len == 0 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 7;
                    response_type = RESP_SUCCESS;
                    copyRadioSettings(&response[INDEX_RESP_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;
            
            case AP_CLEAR_CHANNEL_SEARCH:
                if(cmd_payload_len == 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 21;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = clearChannelSearch(cmdhandler.buf[INDEX_CMD_PAYLOAD], &response[INDEX_RESP_PAYLOAD + 1],  &response[INDEX_RESP_PAYLOAD + 11]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;

            case AP_SET_SESSION_TIME:
                 if(cmd_payload_len == 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    updateMedRadioSessionTime(cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;     
            
            case AP_GET_SESSION_TIME_LEFT:
                if(cmd_payload_len == 0 )
               {
                   response_len = NON_PAYLOAD_RESP_BYTES + 2;
                   response_type = RESP_SUCCESS;
                   result = getTimeRemainingInSession();
                   response[INDEX_RESP_PAYLOAD + 0] = result & 0xFF;
                   response[INDEX_RESP_PAYLOAD + 1] = result >> 8;
               }
               else
               {
                   response_len = NON_PAYLOAD_RESP_BYTES;
                   response_type = RESP_INVALID_PARAMETERS;   
               }
               break;    
            case AP_SET_SESSION_MAINTAIN:
                if(cmd_payload_len == 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    updateMedRadioMaintain(cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;

            case AP_ENCRYPTION:
                if(cmd_payload_len == 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    updateMedRadioEncryption(cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;

             case AP_RESTORE_RADIO:
                if(cmd_payload_len == 1 )
                {
                    if(cmdhandler.buf[INDEX_CMD_PAYLOAD]==0){
                        //switch to fixed PM bootloader settings
                        loadRadioSettingsForPMBoot();
                    }
                    else {
                        //load app settings from flash
                        loadRadioSettingsFromFlash();
                    }
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;

            case WL_SET_IMU_MODE:
                if(cmd_payload_len == 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    //enable_sensor(cmdhandler.buf[INDEX_CMD_PAYLOAD] );
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;   
                }
                break;

            case WL_SET_BUTTON_ACTION:
                if(cmd_payload_len < 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                    break;
                }
                actionindex = cmdhandler.buf[INDEX_CMD_PAYLOAD];
                LOG_INF("Action %d", actionindex);
                LOG_INF("Length %d", cmd_payload_len - 1);
                if (actionindex > MAX_ACTIONS-1) {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                } else {
                    action[actionindex].len = cmd_payload_len - 1;
                    memcpy(action[actionindex].buf, &cmdhandler.buf[INDEX_CMD_PAYLOAD + 1], action[actionindex].len); //copy cmdhandler buf to action
                    LOG_HEXDUMP_INF(action[actionindex].buf,action[actionindex].len, "Action");
                    response_len = NON_PAYLOAD_RESP_BYTES + 2;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = actionindex;
                    response[INDEX_RESP_PAYLOAD + 1] = saved_settings_write(ACTION_SETTINGS_ID+actionindex, &action[actionindex],  sizeof(action[actionindex]));
                }
                break;

            case WL_GET_BUTTON_ACTION:
                if(cmd_payload_len < 1 || cmd_payload_len > 2 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                    break;
                }
                actionindex = cmdhandler.buf[INDEX_CMD_PAYLOAD];
                LOG_INF("Action %d", actionindex);
                if (actionindex > MAX_ACTIONS-1) {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                } else {
                    int err;
                    if(cmd_payload_len == 2 && cmdhandler.buf[INDEX_CMD_PAYLOAD + 1] == 1){
                        // this reloads from flash
                        err = saved_settings_read(ACTION_SETTINGS_ID+actionindex, &action[actionindex], sizeof(action[actionindex]));
                        if(err > 0)
                        {
                            LOG_HEXDUMP_INF(action[actionindex].buf, action[actionindex].len, "Action Read:");	

                        } else{
                            LOG_INF("No action settings found in flash %0X", err);
                        }
                    }
                    //
                    response_len = NON_PAYLOAD_RESP_BYTES + action[actionindex].len + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = actionindex;
                    memcpy(&response[INDEX_RESP_PAYLOAD + 1], action[actionindex].buf, action[actionindex].len);
                }
                break;

            case WL_SET_LED_MODE: 
                if(cmd_payload_len == 1 )
                {
                    modeLED = cmdhandler.buf[INDEX_CMD_PAYLOAD];
                    enable3V3( !(modeLED & LED_AUDIO_OFF));

                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else{
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            
            case WL_GET_LED_MODE: 
                if(cmd_payload_len == 0 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = modeLED;
                }
                else{
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

             case WL_SET_LEDS: 
                if(cmd_payload_len == 3 && (modeLED & LED_MANUAL))
                {
                    setLEDs(cmdhandler.buf[INDEX_CMD_PAYLOAD], cmdhandler.buf[INDEX_CMD_PAYLOAD+1], cmdhandler.buf[INDEX_CMD_PAYLOAD+2]);
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
 
                break;
            
            case WL_GET_LEDS: //JML TODO: NOT YET IMPLEMENTED
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
            
                break;

            case WL_ENTER_LOW_POWER: 
                if(cmd_payload_len == 0 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;

                    lowpower = true;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            
            case WL_BLE_START_ADV:
                if(cmd_payload_len == 0 )
                {
                    ble_start_advertising();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case WL_BLE_STOP_ADV:
                if(cmd_payload_len == 0 )
                {
                    ble_stop_advertising();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case WL_BLE_GET_ADC:
                if(cmd_payload_len == 0 )
                {
                    result = get_adc_sample();
                    response_len = NON_PAYLOAD_RESP_BYTES + 2;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = (uint8_t)(result);
                    response[INDEX_RESP_PAYLOAD + 1] = (uint8_t)(result>>8);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;    

            case WL_GET_CHARGE_STATUS:
                if(cmd_payload_len == 0 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD] = getChargingStatus();
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            
            case WL_SET_BUZZER_PERIOD:
                if(cmd_payload_len < 1 )
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                    break;
                }

                struct audioList_type audioList;
                audioList.len = cmdhandler.buf[INDEX_CMD_PAYLOAD];
                if (audioList.len > MAX_AUDIO_NOTES || cmd_payload_len-1 != audioList.len*4)
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                    break;
                }

                for(uint8_t i=0; i<audioList.len; i++)
                {
                    audioList.notePeriod[i] = ((uint16_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 1 + i*4] << 8 ) + cmdhandler.buf[INDEX_CMD_PAYLOAD + 2 + i*4];
                    audioList.noteTime[i]   = ((uint16_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 3 + i*4] << 8 ) + cmdhandler.buf[INDEX_CMD_PAYLOAD + 4 + i*4];
                    
                }
                while(k_msgq_put(&audio_msgq, &audioList, K_NO_WAIT) != 0)
                {
                    /* message queue is full: purge old data & try again */
                    LOG_INF("Purging Audio MsgQ");
                    k_msgq_purge(&audio_msgq);
                }

                response_len = NON_PAYLOAD_RESP_BYTES + 1;
                response_type = RESP_SUCCESS;
                response[INDEX_RESP_PAYLOAD] = audioList.len;
                break;
            
            case CD_SET_RTC:
                if(cmd_payload_len == 7)  //sec, min, hour, weekday, day, month, year
                {
                    writeRTC(&cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case CD_GET_RTC:
                if(cmd_payload_len == 0)  
                {
                    readRTC(&response[INDEX_RESP_PAYLOAD]);
                    response_len = NON_PAYLOAD_RESP_BYTES + 7; //sec, min, hour, weekday, day, month, year
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case CD_START_DCDC:
                if(cmd_payload_len == 0)  
                {
                    startDCDC();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case CD_STOP_DCDC:
                if(cmd_payload_len == 0)  
                {
                    stopDCDC();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case CD_SET_DCDC_VOLT:
                if(cmd_payload_len == 2)  
                {
                    setDCDC(((uint16_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 1]<<8)+(uint16_t)cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;  

            case CD_START_DRIVE:
                if(cmd_payload_len == 0)  
                {
                    startCoilDrive();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;

            case CD_STOP_DRIVE:
                if(cmd_payload_len == 0)  
                {            
                    stopCoilDrive();
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;

            case CD_SET_PERIOD:
                if(cmd_payload_len == 4)  
                {
                    result = setCoilPeriod (((uint32_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 3]<<24) +
                                            ((uint32_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 2]<<16) +
                                            ((uint32_t)cmdhandler.buf[INDEX_CMD_PAYLOAD + 1]<< 8) +
                                            (uint32_t)cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                    response_len = NON_PAYLOAD_RESP_BYTES + 4;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = (uint8_t) (result & 0xFF);
                    response[INDEX_RESP_PAYLOAD + 1] = (uint8_t) (result >> 8);
                    response[INDEX_RESP_PAYLOAD + 2] = (uint8_t) (result >> 16);
                    response[INDEX_RESP_PAYLOAD + 3] = (uint8_t) (result >> 24);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;  

            case CD_GET_PERIOD:
                if(cmd_payload_len == 0)  
                {
                    result = getCoilPeriod ();
                    response_len = NON_PAYLOAD_RESP_BYTES + 4;
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = (uint8_t) (result & 0xFF);
                    response[INDEX_RESP_PAYLOAD + 1] = (uint8_t) (result >> 8);
                    response[INDEX_RESP_PAYLOAD + 2] = (uint8_t) (result >> 16);
                    response[INDEX_RESP_PAYLOAD + 3] = (uint8_t) (result >> 24);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;  

            case CD_GET_THERMISTOR:
                if(cmd_payload_len == 0)  
                {
                    result = getThermistor();
                    response_len = NON_PAYLOAD_RESP_BYTES + 2; 
                    response_type = RESP_SUCCESS;
                    response[INDEX_RESP_PAYLOAD + 0] = (uint8_t) (result & 0xFF);
                    response[INDEX_RESP_PAYLOAD + 1] = (uint8_t) (result >> 8);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;

            case CD_GET_COIL_POWER:
                if(cmd_payload_len == 0)  
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 4; 
                    response_type = RESP_SUCCESS;
                    getCDPower(&response[INDEX_RESP_PAYLOAD], &response[INDEX_RESP_PAYLOAD+2]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;         
            
            case CD_GET_CHARGER_POWER:   
                if(cmd_payload_len == 0)  
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 4;
                    response_type = RESP_SUCCESS;
                    getSysPower(&response[INDEX_RESP_PAYLOAD], &response[INDEX_RESP_PAYLOAD]+2);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break; 

            case CD_KEEP_COIL_ON:
                if(cmd_payload_len == 0)  
                {
                    
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                    keepCoilOn();
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }   
                break;

            case DISPLAY_SET:
                if(cmd_payload_len < 2) 
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }

                //clear, len1, [str1 ...], len2, [str2 ...], len3, [str3 ...], len4, [str4 ...]
                char line1[DISP_STR_LEN] = "                                                                                ";
                char line2[DISP_STR_LEN] = "                                                                                ";
                char line3[DISP_STR_LEN] = "                                                                                ";
                char line4[DISP_STR_LEN] = "                                                                                ";
                uint8_t cursor = INDEX_CMD_PAYLOAD+1, len1 = 0, len2=0, len3=0, len4=0;

                len1 = cmdhandler.buf[cursor];
                if(len1 <= sizeof(line1))
                {
                    memcpy(&line1[0], &cmdhandler.buf[cursor+1], len1); 
                    cursor += len1 + 1; 
                    len2 = cmdhandler.buf[cursor];
                    if(len2 <= sizeof(line2))
                    {
                        memcpy(&line2[0], &cmdhandler.buf[cursor+1], len2); 
                        cursor += len2 + 1;
                        len3 = cmdhandler.buf[cursor];
                        if(len3 <= sizeof(line3))
                        {
                            memcpy(&line3[0], &cmdhandler.buf[cursor+1], len3); 
                            cursor += len3 + 1;
                            len4 = cmdhandler.buf[cursor];
                            if(len4 <= sizeof(line4))
                            {
                                memcpy(&line4[0], &cmdhandler.buf[cursor+1], len4); 
                                cursor += len4 + 1;

                                OutputDisplay(cmdhandler.buf[INDEX_CMD_PAYLOAD], line1, line2, line3, line4, len1, len2, len3, len4);
                                response_len = NON_PAYLOAD_RESP_BYTES;
                                response_type = RESP_SUCCESS;
                                break;
                            }
                        }
                    }
                }
                //invalid parameters response
                response_len = NON_PAYLOAD_RESP_BYTES;
                response_type = RESP_INVALID_PARAMETERS;
                break;
                
            case SD_WRITE:
                response[INDEX_RESP_PAYLOAD] = logToFile(&cmdhandler.buf[INDEX_CMD_PAYLOAD+1], cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                response_len = NON_PAYLOAD_RESP_BYTES + 1;
                response_type = RESP_SUCCESS;
                break;

            case SD_READ:
                memcpy(&addr, &cmdhandler.buf[INDEX_CMD_PAYLOAD], 4);
                if(readFromFile(&response[INDEX_RESP_PAYLOAD], cmdhandler.buf[INDEX_CMD_PAYLOAD+4], addr))
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + cmdhandler.buf[INDEX_CMD_PAYLOAD+4];
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_SUCCESS;
                }
                break;
            
            case SD_INIT:
                initSD();
                response_len = NON_PAYLOAD_RESP_BYTES;
                response_type = RESP_SUCCESS;
                break;

            case SD_DIR:
                dirSD();
                response_len = NON_PAYLOAD_RESP_BYTES;
                response_type = RESP_SUCCESS;
                break;

            case SD_GET_LOG_LENGTH:
                addr = getLogLength();
                response_len = NON_PAYLOAD_RESP_BYTES + 4;
                response_type = RESP_SUCCESS;
                memcpy(&response[INDEX_RESP_PAYLOAD], &addr, sizeof(addr));
                break;
            
            case SD_FLUSH_LOG:
                response[INDEX_RESP_PAYLOAD] = flushLog();
                response_len = NON_PAYLOAD_RESP_BYTES + 1;
                response_type = RESP_SUCCESS;
                break;

            case COIL_GET_THERMISTOR_1:
            case COIL_GET_THERMISTOR_2:
            case COIL_GET_PRODUCT_ID:
            case COIL_ACCEL_OFF:              
            case COIL_ACCEL_ON:            
            case COIL_GET_ACCEL:   
            case COIL_SET_LED:             
                
                response_len = AWAITING_RESPONSE;
                smartCoil.source = SOURCE_CMDHANDLER;
                smartCoil.len = cmdhandler.len; 
                memcpy(&smartCoil.buf[0], &cmdhandler.buf[0], smartCoil.len); //copy cmdhandler buf to smartCoil buf
                
                LOG_HEXDUMP_INF(smartCoil.buf, smartCoil.len, "SmartCoil req from cmdhandler");

                while(k_msgq_put(&coil_req_msgq, &smartCoil, K_NO_WAIT) != 0)
                {
                    /* message queue is full: purge old data & try again */
                    LOG_INF("Purging MsgQ");
                    k_msgq_purge(&coil_req_msgq);
                }
                break;

            case CHG_GET_CHARGEMODE:
                response[INDEX_RESP_PAYLOAD] = getChargeMode();
                response_len = NON_PAYLOAD_RESP_BYTES + 1;
                response_type = RESP_SUCCESS;
                break;

            case CHG_SET_CHARGEMODE:
                response[INDEX_RESP_PAYLOAD] = setChargeMode(cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                response_len = NON_PAYLOAD_RESP_BYTES + 1;
                response_type = RESP_SUCCESS;
                break;

            // case CHG_METAL_DETECT:
            //     response[INDEX_RESP_PAYLOAD] = chargerStartStopMetalDetect();
            //     response_len = NON_PAYLOAD_RESP_BYTES + 1;
            //     response_type = RESP_SUCCESS;
            //     break;

            case CD_SET_PARAMS:
                if(cmd_payload_len == 6) 
                {
                    setChargerParams( &cmdhandler.buf[INDEX_CMD_PAYLOAD]);
                    saved_settings_write(COIL_SETTINGS_ID, &cmdhandler.buf[INDEX_CMD_PAYLOAD], 6);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                
            break;

            case CD_GET_PARAMS:
                if(cmd_payload_len == 0) 
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + getChargerParams(&response[INDEX_RESP_PAYLOAD]);
                    response_type = RESP_SUCCESS;
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
            break;

            case PURGE_MSG_QUEUES:
                purge_msg_queues();
                break;

            case TEST_READ_SDO:
                testReadSDO();
                break;
            
            case TEST_WRITE_SDO:
                testWriteSDO();
                break;

            case TEST_NMT:
                testNMT();
                break;

            case WL_GET_IMU_ACCEL:
                if(cmd_payload_len == 0) 
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 6;
                    response_type = RESP_SUCCESS;
                    getAccel(&response[INDEX_RESP_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
            case WL_GET_IMU_GYRO:
                if(cmd_payload_len == 0) 
                {
                    response_len = NON_PAYLOAD_RESP_BYTES + 6;
                    response_type = RESP_SUCCESS;
                    getGyro(&response[INDEX_RESP_PAYLOAD]);
                }
                else
                {
                    response_len = NON_PAYLOAD_RESP_BYTES;
                    response_type = RESP_INVALID_PARAMETERS;
                }
                break;
        }




        while(response_len == AWAITING_RESPONSE)
        {
            if(medRadio.len > 0) //we have an implant request to send
            {
                medRadio.source = SOURCE_CMDHANDLER;
                LOG_HEXDUMP_INF(medRadio.buf, medRadio.len, "TX MedRadio Packet");
                while(k_msgq_put(&imp_req_msgq, &medRadio, K_NO_WAIT) != 0)
                {
                    /* message queue is full: purge old data & try again */
                    LOG_INF("Purging ImpReq MsgQ");
                    k_msgq_purge(&imp_req_msgq);
                }
            }
            //wait for response from Implant Req/Resp Thread or Coil Req/Resp Thread.  
            //Since there is no timeout here, make sure the tasks expected to push the message do.
            k_msgq_get(&cmd_resp_msgq, &cmdhandler, K_FOREVER);

            if(response_type == RESP_LOCAL_PMBOOT_WRITE || response_type == RESP_LOCAL_PMBOOT_READ || response_type == RESP_LOCAL_PMFILE_READ)
            {
                //scrap CRC/LQI bytes and the nonpayload bytes  
                if (cmdhandler.len < NON_PAYLOAD_RESP_BYTES + 2)
                {
                    len = 0;  
                }
                else
                {
                    len = cmdhandler.len - NON_PAYLOAD_RESP_BYTES - 2;
                }
                switch(response_type)
                {
                    case RESP_LOCAL_PMBOOT_WRITE:
                        err = pmboot_write_supervisor(NULL, &cmdhandler.buf[INDEX_RESP_PAYLOAD], len, &medRadio.buf[0], &medRadio.len); 
                        break;
                    case RESP_LOCAL_PMBOOT_READ:
                        err = pmboot_read_supervisor(NULL, &cmdhandler.buf[INDEX_RESP_PAYLOAD], len, &medRadio.buf[0], &medRadio.len); 
                        break;
                    case RESP_LOCAL_PMFILE_READ:
                        err = pmfile_read_supervisor(NULL, &cmdhandler.buf[INDEX_RESP_PAYLOAD], len, &medRadio.buf[0], &medRadio.len); 
                        break;

                }


                if(err == 0 || err == 0xFF){ //local task completed either with sucess or failed retry attempts
                    LOG_INF("Done with local process %02X", err);

                    response_type = RESP_SUCCESS;
                    response_len = NON_PAYLOAD_RESP_BYTES + 1;
                    
                    if(err){
                        response[INDEX_RESP_PAYLOAD] = 0; //fail
                    }
                    else{
                        response[INDEX_RESP_PAYLOAD] = 1; //success
                    }
                    //message will be sent on USB/UART or BLE below
                }
                // otherwise go back to begining of loop and send out next implant request

            }
            else
            {
                memcpy(&response[0], &cmdhandler.buf[0], cmdhandler.len);  //copy msg to response
                response_type = RESP_SUCCESS;
                response_len = cmdhandler.len;
            }

        } 

        if (response_len > AWAITING_RESPONSE)
        {
            response[INDEX_RESP_SYNC] = SYNC_BYTE;
            response[INDEX_RESP_TYPE] = response_type;
            response[INDEX_RESP_LEN] = response_len;

            LOG_HEXDUMP_INF(response, response_len, "Response to Host:");

            switch(route)
            {
                case ROUTE_UART:
                    uart_send(response, response_len);
                    break;
                
                case ROUTE_BLE:
                    ble_send(response, response_len);
                    break;
                
                default:
                    LOG_INF("command response route %d not yet implemented", cmdhandler.route);
                
            }
        }

        if(lowpower){
            setLowPower();
        }

    }


}

/* This function is first called by the cmdhandler with pmboot settings
   In this case resp is null.  
   Follow up calls will be made with resp containing the payload of the implant response to the previous request
   req and req_len are outputs.  They provide information for the next radio request
  */
uint8_t pmboot_write_supervisor(uint8_t *pmboot, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len)
{
    static uint8_t pkt = 0, errcnt=0;
    static uint8_t header[7];  //address of beginning of page (4 bytes), page size (2 bytes, always 512), sector (1 byte)
    

    if(pmboot != NULL)
    {
        pkt = 1;
        memcpy(header, pmboot, sizeof(header)); //stores the address info in case we need to retry
        errcnt = 0 ;
    }
    else if (resp_len > 0 || resp != NULL)
    {
        if(pkt==0)
        {
            LOG_INF("radio resp but pkt=0");
        }
        else if(pkt == 1)
        {
            if(resp_len == 2 && resp[0] == 0x20 && resp[1] == 1)
            {
                pkt++;
                errcnt = 0 ;
            }
            else
            {
                LOG_HEXDUMP_INF(resp, resp_len, "First PMBOOT pkt");
                errcnt++;
            }
        }
        else
        {
            if(resp_len == 2 && resp[0] == 0x21 && resp[1] == 1)
            {
                pkt++;
                errcnt = 0 ;
            }
            else if(pkt == (PM_PAGE_SIZE-PMBOOT_FIRST_PKT_BYTES)/PMBOOT_PKT_BYTES + 1 && resp_len == 2 && resp[0] == 0x21 && resp[1] == 0)
            {
                //failed to write image, need to start back at begininning of image
                LOG_INF("pmboot_write_supervisor: PM failed to write image on last packet, restarting image");
                pkt = 1;
                errcnt++; //JML Note: could have infinite loop here where last packet always fails but other packets restore errcnt to 0.  Add second error counter?
            }
            else{
                LOG_INF("Other PMBOOT pkt %d", pkt);
                LOG_HEXDUMP_INF(resp, resp_len, " ");
                errcnt++;
            }
        }
    }
    else
    {
        LOG_INF("pmboot_write_supervisor called with no data");
        errcnt++;
    }
    if(errcnt>5){
        LOG_INF("exceeded 5 retries");
        return 0xFF;
    
    }

    LOG_INF("PMBOOT write pkt# %d, err# %d following receive", pkt, errcnt);
    
    if((PMBOOT_FIRST_PKT_BYTES + (pkt-2)*PMBOOT_PKT_BYTES) >= PM_PAGE_SIZE){ 
        pkt = 0; //done
        *req_len = 0;
    }
    else 
    {
        if(pkt == 1)
        {
            *req_len = 1+sizeof(header)+PMBOOT_FIRST_PKT_BYTES; 
            req[0]=0x10;
            memcpy(&req[1], header, sizeof(header)); 
            memcpy(&req[1+sizeof(header)], &image[0], PMBOOT_FIRST_PKT_BYTES); 
        }
        else
        {
            *req_len = 1+PMBOOT_PKT_BYTES; 
            req[0]=0x11;
            memcpy(&req[1], &image[PMBOOT_FIRST_PKT_BYTES + (pkt-2)*PMBOOT_PKT_BYTES], PMBOOT_PKT_BYTES); 
        }
    }
    return pkt;
}


uint8_t pmboot_read_supervisor(uint8_t *pmboot, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len )
{
    static uint16_t image_addr = 0; //address into stored image
    static uint8_t pkt = 0, errcnt=0;
    static uint8_t header[6];  //address (4 bytes), size (2 bytes)
    uint32_t address; //address in PM flash


    if(pmboot != NULL)
    {
        pkt = 1;
        image_addr = 0;
        memcpy(header, pmboot, sizeof(header)); //stores the address info in case we need to retry
        errcnt = 0 ;
    }
    else if (resp_len > 0 || resp != NULL)
    {
        if(pkt==0)
        {
            LOG_INF("radio resp but pkt=0");
        }
        else 
        {
            if(resp_len >= 1 && resp[0] == 0x23 )
            {
                if(image_addr + resp_len-1 <= PM_PAGE_SIZE)
                {
                    LOG_INF("Good Read PMBOOT pkt %d addr%d resp_len %d", pkt, image_addr, resp_len);
                    //JML TODO?:  resp_len-1 should be PMBOOT_PKT_BYTES or PMBOOT_FIRST_PKT_BYTES
                    memcpy(&image[image_addr], &resp[1], resp_len-1);
                    image_addr+=resp_len-1;
                    pkt++;
                    errcnt = 0 ;
                }
                else{
                    LOG_INF("Bad Read PMBOOT pkt %d addr%d resp_len %d", pkt, image_addr, resp_len);
                    LOG_HEXDUMP_INF(resp, resp_len, " ");
                    errcnt++;
                }
                
            }
            else
            {
                LOG_HEXDUMP_INF(resp, resp_len, "Read PMBOOT pkt");
                errcnt++;
            }
        }        
    }
    else
    {
        LOG_INF("pmboot_read_supervisor called with no data");
        errcnt++;
    }
    if(errcnt>5){
        LOG_INF("exceeded 5 retries");
        return 0xFF;
    
    }

     LOG_INF("PMBOOT read pkt# %d, err# %d following receive", pkt, errcnt);
     
    if((PMBOOT_FIRST_PKT_BYTES + (pkt-2)*PMBOOT_PKT_BYTES) >= PM_PAGE_SIZE){
        pkt = 0; //done
        *req_len = 0;
    }
    else
    {
        *req_len = 1+sizeof(header); 
        req[0]=0x13;
        address = ((uint32_t)header[0]<<24) + ((uint32_t)header[1]<<16) + ((uint32_t)header[2]<<8) + ((uint32_t)header[3]);
        address += image_addr;
       
        req[1] = (uint8_t) (address>>24);
        req[2] = (uint8_t) (address>>16);
        req[3] = (uint8_t) (address>>8);
        req[4] = (uint8_t) (address);
        req[5] = 0;
        if(pkt == 1)
        {
            req[6] = PMBOOT_FIRST_PKT_BYTES;
        }
        else{
            req[6] = PMBOOT_PKT_BYTES;
        }
    }


    return pkt;
}

uint8_t pmfile_read_supervisor(uint8_t * pmfile, uint8_t * resp, uint8_t resp_len, uint8_t * req, uint8_t * req_len )
{
    static uint16_t image_addr = 0;  //address into stored image
    static uint8_t pkt = 0, errcnt=0;
    static uint8_t header[7]; //address (4 bytes), size (up to image size), file (1 byte)
    uint32_t address; //address of PM file (remote flash address - starting address for file)
    uint16_t len; 

    if(pmfile != NULL)
    {
        pkt = 1;
        image_addr = 0;
        memcpy(header, pmfile, sizeof(header)); //stores the address info in case we need to retry
        errcnt = 0 ;
    }
    else if (resp_len > 0 || resp != NULL)
    {
        if(pkt==0)
        {
            LOG_INF("radio resp but pkt=0");
        }
        else 
        {
            if(resp_len >= 10 && resp[0] == 0x26 )
            {
                if(image_addr + resp_len-1 <= sizeof(image))
                {
                    memcpy(&image[image_addr], &resp[10], resp_len-10);
                    image_addr+=resp_len-1;
                    pkt++;
                    errcnt = 0 ;
                }
                else{
                    LOG_INF("Bad Read PMFILE pkt %d addr%d resp_len %d", pkt, image_addr, resp_len);
                    LOG_HEXDUMP_INF(resp, resp_len, " ");
                    errcnt++;
                }
                
            }
            else
            {
                LOG_HEXDUMP_INF(resp, resp_len, "Read PMFILE pkt");
                errcnt++;
            }
        }        
    }
    else
    {
        LOG_INF("pmfile_read_supervisor called with no data");
        errcnt++;
    }
    if(errcnt>5){
        LOG_INF("exceeded 5 retries");
        return 0xFF;
    
    }

     LOG_INF("PMFILE read pkt# %d, err# %d following receive", pkt, errcnt);
    if((PMBOOT_FIRST_PKT_BYTES + (pkt-1)*PMBOOT_PKT_BYTES) > PM_PAGE_SIZE){
        pkt = 0; //done
        *req_len = 0;
    }
    else
    {
        len = ((uint16_t)header[6]<<8)+(uint16_t)header[5];

        *req_len = 14; 
        req[0]=0x26;
        req[1]=0xD0 + header[7];
        req[2]=1;
        req[3]=7;
        req[4]=0;
        req[5]=0;
        req[6]=0;
        req[7]=6;
        
        address = ((uint32_t)header[3]<<24) + ((uint32_t)header[2]<<16) + ((uint32_t)header[1]<<8) + ((uint32_t)header[0]);
        address += image_addr;
       
        req[8] = (uint8_t) (address);
        req[9] = (uint8_t) (address>>8);
        req[10] = (uint8_t) (address>>16);
        req[11] = (uint8_t) (address>>24);
        req[12] = 0;
        if (image_addr + PMFILE_PKT_BYTES <= len)
        {
            req[13] = PMFILE_PKT_BYTES;
        }
        else
        {
            req[13] = len - image_addr;
        }

    }
    return pkt;
}


#define CMD_STACKSIZE 8192
#define CMD_PRIORITY 6

K_THREAD_DEFINE(command_handler_thread_id, CMD_STACKSIZE, command_handler_thread, NULL, NULL, NULL, CMD_PRIORITY, 0, 0);

void resetWL(void){
    NVIC_SystemReset();

    // // errata - Tested this workaround. Be aware of the register address:
    // // - 0x50005000 for secure firmware
    // // - 0x40005000 for non-secure firmware
    // *(volatile uint32_t *) 0x40005618ul = 1ul;
    // NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
    // k_msleep(5); // Wait for at least five microseconds
    // NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Hold << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
    // k_msleep(1); // Wait for at least one microsecond
    // NRF_RESET->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
    // *(volatile uint32_t *) 0x40005618ul = 0ul;
}

void purge_msg_queues(void){
    k_msgq_purge(&imp_req_msgq);
    k_msgq_purge(&imp_resp_msgq);
    k_msgq_purge(&cmd_req_msgq);
    k_msgq_purge(&cmd_resp_msgq);
    k_msgq_purge(&coil_req_msgq);
    k_msgq_purge(&coil_resp_msgq);
}
#include <zephyr/drivers/gpio.h>
#include "wlgpio.h"
#include "cc1101radio.h"

#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h> //for k_msleep
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>


#define LOG_MODULE_NAME wlgpio
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


/*Port0*/
#define WL0PIN_DCDC_EN     2 
#define WL0PIN_RST_DISP    3 
#define WL0PIN_GDO0        6 
#define WL0PIN_BUZZ_EN     7
#define WL0PIN_BUCK_EN     12
#define WL0PIN_GDO2        19 
#define WL0PIN_WIFI_EN     31

/*Port1*/
#define WL1PIN_3V3_EN   0
#define WL1PIN_DEBUG0   4
#define WL1PIN_DEBUG1   5
#define WL1PIN_CD_EN    6
#define WL1PIN_PG       9
#define WL1PIN_STAT2    10
#define WL1PIN_STAT1    11
#define WL1PIN_IMUINT   12



static const struct device *gpiodev0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *gpiodev1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/*Define a variable of type static struct gpio_callback */
static struct gpio_callback radio_int_cb_data;

//NEW
/*Define the callback function */
void radio_int_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	radio_interrupt();
}

void init_wl_gpio(void)
{
    int err;

    if (!device_is_ready(gpiodev0)) {
		LOG_ERR("GPIOdev0 device not available");
	}
    if (!device_is_ready(gpiodev1)) {
		LOG_ERR("GPIOdev1 device not available");
	}

    //Charging Status inputs
	/* Configure the interrupt on the button's pin falling edge */
	err = gpio_pin_configure(gpiodev1, WL1PIN_PG, GPIO_INPUT | GPIO_PULL_UP );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_PG as input (err: %d)", err);
	}
	err = gpio_pin_configure(gpiodev1, WL1PIN_STAT2 , GPIO_INPUT | GPIO_PULL_UP );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_STAT2 as input (err: %d)", err);
	}
	err = gpio_pin_configure(gpiodev1, WL1PIN_STAT1, GPIO_INPUT | GPIO_PULL_UP );
	if (err) {
		LOG_ERR("Cannot configure GPIO1.11 as input (err: %d)", err);
	}

	//ISM330IS interrupt lines 
	err = gpio_pin_configure(gpiodev1, WL1PIN_IMUINT, GPIO_INPUT  );
	if (err) {
		LOG_ERR("Cannot configure  WL1PIN_IMUINT as input (err: %d)", err);
	}
	
    //3V3_LED ENABLE
	err = gpio_pin_configure(gpiodev1, WL1PIN_3V3_EN, GPIO_OUTPUT_ACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_3V3_EN as output (err: %d)", err);
	}

	//CoilDrive ENABLE
	err = gpio_pin_configure(gpiodev1, WL1PIN_CD_EN, GPIO_OUTPUT_INACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_CD_EN as output (err: %d)", err);
	}

	err = gpio_pin_configure(gpiodev1, WL1PIN_DEBUG0, GPIO_OUTPUT_INACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_DEBUG0 as output (err: %d)", err);
	}
	
	err = gpio_pin_configure(gpiodev1, WL1PIN_DEBUG1, GPIO_OUTPUT_INACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL1PIN_DEBUG1 as output (err: %d)", err);
	}

    /* This has a big impact on current consumption when operating at 3.3V */
	err = gpio_pin_configure(gpiodev0, WL0PIN_WIFI_EN, GPIO_OUTPUT_INACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_WIFI_EN as output (err: %d)", err);
	}

	/* P0.7 low to turn off buzzer enable*/
	err = gpio_pin_configure(gpiodev0, WL0PIN_BUZZ_EN, GPIO_OUTPUT_INACTIVE );
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_BUZZ_EN as output (err: %d)", err);
	}

	/* P0.3 (nRESET) high to use LCD Display */
	err = gpio_pin_configure(gpiodev0, WL0PIN_RST_DISP, GPIO_OUTPUT_INACTIVE );
	
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_RST_DISP as output low (err: %d)", err);
	}


	err = gpio_pin_configure(gpiodev0, WL0PIN_DCDC_EN, GPIO_OUTPUT_INACTIVE );
	
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_DCDC_EN as output low (err: %d)", err);
	}
	enableDCDC(1);

    //CC1101Radio configure inputs and interrupts
    err = gpio_pin_configure(gpiodev0, WL0PIN_GDO0, GPIO_INPUT );
	
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_GDO0 as input (err: %d)", err);
	}

	
	err = gpio_pin_configure(gpiodev0, WL0PIN_GDO2, GPIO_INPUT);
	
	if (err) {
		LOG_ERR("Cannot configure WL0PIN_GDO2 as input (err: %d)", err);
		return;
	}

}

void configureRadioInterrupt(void)
{
    int err; 
    /* Configure the interrupt on the falling edge */
	err = gpio_pin_interrupt_configure(gpiodev0, WL0PIN_GDO0, GPIO_INT_EDGE_TO_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure WL0PIN_GDO0 interrupt (err: %d)", err);
        return;
    }
	
    gpio_init_callback(&radio_int_cb_data, radio_int_cb, BIT(WL0PIN_GDO0)); 	
	gpio_add_callback(gpiodev0, &radio_int_cb_data);

	/* Initialize the work item to allow offloadin*/
	//k_work_init(&isr_work.work, isr_offload_function);	
}

//enables #V# LDO required to use LEDs and Audio Buzzer
void enable3V3(bool enable){
	gpio_pin_set(gpiodev1, WL1PIN_3V3_EN, enable);
}

void enableWiFi(bool enable){
	gpio_pin_set(gpiodev0, WL0PIN_WIFI_EN , enable);
}

void enableDCDC(bool enable){
	gpio_pin_set(gpiodev0, WL0PIN_DCDC_EN, enable);
}

void enableDisplay(bool enable){
	gpio_pin_set(gpiodev0, WL0PIN_RST_DISP, enable);
}

void enableCoilDrive(bool enable){
	gpio_pin_set(gpiodev1, WL1PIN_CD_EN, enable );
}

void enableDebug0(bool enable){
	gpio_pin_set(gpiodev1, WL1PIN_DEBUG0, enable );
}

void enableDebug1(bool enable){
	gpio_pin_set(gpiodev1, WL1PIN_DEBUG1, enable );
}


uint8_t getChargingStatus(void)
{
	int val;
	uint8_t stat = 0;

	val = gpio_pin_get(gpiodev1, WL1PIN_PG ); //PG
	if (val<0) {
		LOG_ERR("Cannot read GPIO1.09 (PG) (err: %d)", val);
		return 0xF1;
	} else {
		stat |= val;
	}
	val = gpio_pin_get(gpiodev1, WL1PIN_STAT1 ); //STAT1
	if (val<0) {
		LOG_ERR("Cannot read GPIO1.11 (STAT1) (err: %d)", val);
		return 0xF2;
	} else {
		stat |= (val<<1);
	}

	val = gpio_pin_get(gpiodev1, WL1PIN_STAT2 ); //STAT2
	if (val<0) {
		LOG_ERR("Cannot read GPIO1.10 (STAT2) (err: %d)", val);
		return 0xF3;
	} else {
		stat |= (val<<2);
	}
	return stat;
}








#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h> //for k_msleep
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "audio.h"

#define LOG_MODULE_NAME audio
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define AUDIO_STACKSIZE 1024 //bytes
#define AUDIO_PRIORITY 9

static const struct pwm_dt_spec pwm_buzz = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));  //JML TODO: change alias to something more descriptive
K_MSGQ_DEFINE(audio_msgq, sizeof(struct audioList_type), 3, 4);

////////  AUDIO

//period in us, timeon in ms
uint16_t setBuzzerPeriod(uint16_t period, uint16_t timeon){

	uint32_t period_ns = ((uint32_t)period) *1000;
	int err = pwm_set_dt(&pwm_buzz, period_ns, period_ns >> 1);
	if (err) {
		LOG_WRN("Error %d: failed to set PWM pulse width/period\n", err);
		return 0;
	}
	//LOG_INF("Setting Buzzer Period");
	if (timeon > 0)
	{
		k_msleep(timeon);
		err = pwm_set_dt(&pwm_buzz, 0, 0);
		if (err) {
			LOG_WRN("Error %d: failed to disable PWM\n", err);
			return 0;
		}
	}
	return period;

}

void audio_thread(void)
{
	LOG_INF("Starting audio thread");

	if (!device_is_ready(pwm_buzz.dev)) {
	LOG_WRN("Error: PWM device %s is not ready. No audio supported\n",
			pwm_buzz.dev->name);
			return;
	}

	struct audioList_type audioList; 
	for (;;) {
		/* Wait indefinitely for request coming from BLE or UART or locally */
		k_msgq_get(&audio_msgq, &audioList, K_FOREVER);
        for (uint8_t i = 0; i< audioList.len; i++){
            setBuzzerPeriod(audioList.notePeriod[i], audioList.noteTime[i]); //TODO: handle an error response?
        }
        
	}

}

K_THREAD_DEFINE(audio_thread_id, AUDIO_STACKSIZE, audio_thread, NULL, NULL, NULL, AUDIO_PRIORITY, 0, 0);
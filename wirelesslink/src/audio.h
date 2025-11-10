#ifndef _AUDIO_H
#define _AUDIO_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define MAX_AUDIO_NOTES 16 //must be divisible by 4 and should not exceed 60
//JML: does this need the __aligned(4) attribute
struct audioList_type {
   	uint16_t notePeriod[MAX_AUDIO_NOTES]; 
	uint16_t noteTime[MAX_AUDIO_NOTES];
    uint8_t len;
    uint8_t unused1;
    uint8_t unused2;
    uint8_t unused3;
};

extern struct k_msgq audio_msgq;

#endif
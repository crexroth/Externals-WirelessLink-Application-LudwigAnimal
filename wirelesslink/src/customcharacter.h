#ifndef _CUSTOMCHARACTER_H
#define _CUSTOMCHARACTER_H


#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define CUSTOMCHAR_RADIO0   {   0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000  }

#define CUSTOMCHAR_RADIO1   {   0b00000,\
                                0b00000,\
                                0b00000,\
                                0b00000,\
                                0b10000,\
                                0b10000,\
                                0b10000,\
                                0b10000 }

#define CUSTOMCHAR_RADIO2   {   0b00000,\
                                0b00000,\
                                0b00100,\
                                0b00100,\
                                0b10100,\
                                0b10100,\
                                0b10100,\
                                0b10100  }

#define CUSTOMCHAR_RADIO3   {   0b00001,\
                                0b00001,\
                                0b00101,\
                                0b00101,\
                                0b10101,\
                                0b10101,\
                                0b10101,\
                                0b10101 }      
                        
#define CUSTOMCHAR_PERSON   {   0b11000,\
                                0b11000,\
                                0b10000,\
                                0b11100,\
                                0b10000,\
                                0b11110,\
                                0b00010,\
                                0b00011 }    

#define CUSTOMCHAR_COIL  {      0b00000,\
                                0b00000,\
                                0b01110,\
                                0b10001,\
                                0b10101,\
                                0b10001,\
                                0b01110,\
                                0b00000 }   

#define CUSTOMCHAR_LIGHTNING {   0b00010,\
                                0b00110,\
                                0b01100,\
                                0b11100,\
                                0b00111,\
                                0b00110,\
                                0b01100,\
                                0b01000 }    
                                
#define CUSTOMCHAR_DEGREEC  {   0b10000,\
                                0b00110,\
                                0b01001,\
                                0b01000,\
                                0b01000,\
                                0b01001,\
                                0b00110,\
                                0b00000 }    
                                
#define CUSTOMCHAR_BATTERY0 {   0b01110,\
                                0b11011,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b11111 } 

#define CUSTOMCHAR_BATTERY1 {   0b01110,\
                                0b11011,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b11111,\
                                0b11111 } 

#define CUSTOMCHAR_BATTERY2 {   0b01110,\
                                0b11011,\
                                0b10001,\
                                0b10001,\
                                0b10001,\
                                0b11111,\
                                0b11111,\
                                0b11111 } 

#define CUSTOMCHAR_BATTERY3 {   0b01110,\
                                0b11011,\
                                0b10001,\
                                0b10001,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111 } 
    
#define CUSTOMCHAR_BATTERY4 {   0b01110,\
                                0b11011,\
                                0b10001,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111 } 

#define CUSTOMCHAR_BATTERY5 {   0b01110,\
                                0b11011,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111 }      

#define CUSTOMCHAR_BATTERY6 {   0b01110,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111,\
                                0b11111 }                                   


#define CUSTOMCHAR_BIGSTAR  {   0b00000,\
                                0b00000,\
                                0b00100,\
                                0b00100,\
                                0b01110,\
                                0b11111,\
                                0b01110,\
                                0b00100 } 

 #define CUSTOMCHAR_LILSTAR  {  0b00000,\
                                0b01000,\
                                0b01000,\
                                0b11100,\
                                0b01000,\
                                0b01000,\
                                0b00000,\
                                0b00000 }   

#define CUSTOMCHAR_DOUBLEI {    0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01010,\
                                0b00000 }    

#define CUSTOMCHAR_TUNER    {   0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01010,\
                                0b01110,\
                                0b00100,\
                                0b00100,\
                                0b00100 } 
    
#define CUSTOMCHAR_POWER     {  0b00000,\
                                0b00100,\
                                0b10101,\
                                0b10101,\
                                0b10001,\
                                0b10001,\
                                0b01110,\
                                0b00000 }   

#define CUSTOMCHAR_BT        {  0b00100,\
                                0b10110,\
                                0b01101,\
                                0b00110,\
                                0b01101,\
                                0b10110,\
                                0b00100,\
                                0b00000 }         

 #define CUSTOMCHAR_BTPAIR    { 0b00100,\
                                0b10110,\
                                0b01101,\
                                0b10110,\
                                0b01101,\
                                0b10110,\
                                0b00100,\
                                0b00000 }     
                                 
#endif
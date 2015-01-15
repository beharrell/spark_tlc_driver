/*
TlcDriver/tlc_driver for Spark Core is released under the BSD 2-Clause License:

Copyright (c) 2014, Dub Ink, Justin "Goose" Gaussoin
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


Major Credit to:

Alex Leone <acleone ~AT~ gmail.com>
http://alex.kathack.com/codes/tlc5940arduino/index.html

for some of the structure and proper bit shifting
 */

#ifndef __TLC_DRIVER_H
#define __TLC_DRIVER_H

#define GSCLK_COUNT 4096 //Resolution of the Greyscale clock for TLC5940, could be 255 but better to have higher resolution
#define GSCLK_FREQ  5000000 //Run at 5Mhz, fast enough there is no hesitation for updates to the TLC5940
#define BLANK_COUNT 768 //Enough time for a rising blank pulse to give room for the xlat pulse to latch in new data


#define PIN_GSCLK   GPIO_Pin_0 //TIM2 Channel 1, GPIO Port A, Pin 0, A0, TIM2 is attached

#define PIN_BLANK   GPIO_Pin_6 //TIM4 Channel 1, GPIo Port B, Pin 6, D1, TIM4 is attached

#define PIN_XLAT    GPIO_Pin_7 //GPIO Port B, Pin 7, D0

#define PIN_SIN   	GPIO_Pin_7 //GPIO Port A, Pin 7, Spark MOSI pin, A5
#define PIN_MISO    GPIO_Pin_6 //GPIO Port A, Pin 6, Spark MISO pin, A4
#define PIN_SCLK 	GPIO_Pin_5 //GPIO Port A, Pin 5, Spark SCLK, A3
#define PIN_SS      GPIO_Pin_4 //GPIO Port A, Pin 4, Spark SS, A2

#include "application.h"


extern "C" void TIM4_IRQHandler();



#ifdef __cplusplus
extern "C" {
#endif



class TlcDriver
{
    public:
        static uint8_t needsXLAT;
        static const uint8_t NUM_TLCS = 1;
        uint8_t gsData[NUM_TLCS * 24];
        void init();
        void clear();
        void set(uint8_t channel, uint16_t value);
        void update();

    private:
        void initGPIO();
        void initGSTimer();
        void initBlankTimer();
        void initSPI();
        void writeData(uint8_t data);



};

#ifdef __cplusplus
}
#endif





#endif
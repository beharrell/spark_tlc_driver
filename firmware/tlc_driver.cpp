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
#include "tlc_driver.h"

extern "C" void TIM4_IRQHandler()
{
    if (TIM_GetFlagStatus(TIM4,TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        
        if (TIM4->CR1 & TIM_CR1_DIR) {
            
            if (TlcDriver::needsXLAT) {
                delayMicroseconds(2);
                GPIO_WriteBit(GPIOB, PIN_XLAT, Bit_SET);
                delayMicroseconds(1);
                GPIO_WriteBit(GPIOB, PIN_XLAT, Bit_RESET);
                TlcDriver::needsXLAT=0;
            }   
        }
    }
}

uint8_t TlcDriver::needsXLAT = 0;

void TlcDriver::init()
{
	needsXLAT = 0;
    initGPIO();
    initGSTimer();
    initBlankTimer();
    initSPI();
}


void TlcDriver::writeData(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
}


void TlcDriver::clear()
{
	uint8_t value = 0;
	uint8_t firstByte = value >> 4;
    uint8_t secondByte = (value << 4) | (value >> 8);
    uint8_t *p = gsData;
    while (p < gsData + NUM_TLCS * 24) {
        *p++ = firstByte;
        *p++ = secondByte;
        *p++ = (uint8_t)value;
    }
}

void TlcDriver::set(uint8_t channel, uint16_t value)
{
	uint8_t index8 = (NUM_TLCS * 16 - 1) - channel;
    uint8_t *index12p = gsData + ((((uint16_t)index8) * 3) >> 1);
    if (index8 & 1) { // starts in the middle
                      // first 4 bits intact | 4 top bits of value
        *index12p = (*index12p & 0xF0) | (value >> 8);
                      // 8 lower bits of value
        *(++index12p) = value & 0xFF;
    } else { // starts clean
                      // 8 upper bits of value
        *(index12p++) = value >> 4;
                      // 4 lower bits of value | last 4 bits intact
        *index12p = ((uint8_t)(value << 4)) | (*index12p & 0xF);
    }
}

void TlcDriver::update()
{
	if (needsXLAT) {
        return;
    }
    uint8_t *p = gsData;
    while (p < gsData + NUM_TLCS * 24) {
        writeData(*p++);
        writeData(*p++);
        writeData(*p++);
    }
    needsXLAT = 1;
}

void TlcDriver::initGPIO()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Pin = PIN_GSCLK; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Pin = PIN_BLANK; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = PIN_XLAT; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void TlcDriver::initGSTimer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    uint16_t GSCLK_Prescaler = (uint16_t) ((SystemCoreClock / GSCLK_FREQ) - 1);

    TIM_BaseInitStructure.TIM_Period = 1;
    TIM_BaseInitStructure.TIM_Prescaler = GSCLK_Prescaler;
    TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_Cmd(TIM2, ENABLE);
}

void TlcDriver::initBlankTimer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_BaseInitStructure.TIM_Period = GSCLK_COUNT + BLANK_COUNT;
    TIM_BaseInitStructure.TIM_Prescaler = 0;
    TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = BLANK_COUNT;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); 
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

    TIM_SelectInputTrigger(TIM4, TIM_TS_ITR1);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TlcDriver::initSPI()
{
	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    GPIO_InitStructure.GPIO_Pin = PIN_SCLK | PIN_SIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CRCPolynomial = 0;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

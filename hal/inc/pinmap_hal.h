/**
 ******************************************************************************
 * @file    pinmap_hal.h
 * @authors Satish Nair, Brett Walach
 * @version V1.0.0
 * @date    13-Sept-2014
 * @brief
 ******************************************************************************
  Copyright (c) 2013-2015 Particle Industries, Inc.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PINMAP_HAL_H
#define __PINMAP_HAL_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    
/* Exported types ------------------------------------------------------------*/

typedef uint16_t pin_t;

typedef enum PinMode {
  INPUT,
  OUTPUT,
  INPUT_PULLUP,
  INPUT_PULLDOWN,
  AF_OUTPUT_PUSHPULL, //Used internally for Alternate Function Output PushPull(TIM, UART, SPI etc)
  AF_OUTPUT_DRAIN,    //Used internally for Alternate Function Output Drain(I2C etc). External pullup resistors required.
  AN_INPUT,           //Used internally for ADC Input
  AN_OUTPUT,          //Used internally for DAC Output
  PIN_MODE_NONE=0xFF
} PinMode;

typedef enum {
    PF_NONE,
    PF_DIO,
    PF_TIMER,
    PF_ADC,
  PF_DAC
} PinFunction;

PinFunction HAL_Validate_Pin_Function(pin_t pin, PinFunction pinFunction);

typedef struct STM32_Pin_Info  STM32_Pin_Info;

STM32_Pin_Info* HAL_Pin_Map(void);

/* Exported macros -----------------------------------------------------------*/

/*
* Pin mapping. Borrowed from Wiring
*/
#if PLATFORM_ID!=3

#define TOTAL_PINS 21
#define TOTAL_ANALOG_PINS 8
#define FIRST_ANALOG_PIN 10

#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7

// todo - this is corev1 specific, needs to go in a conditional define

#define LED1 LED_USER

#define A0 10
#define A1 11
#define A2 12
#define A3 13
#define A4 14
#define A5 15
#define A6 16

// WKP pin is also an ADC on Photon
#define A7 17

// RX and TX pins are also ADCs on Photon
#define A8 18
#define A9 19

#define RX 18
#define TX 19

#define BTN 20

// WKP pin on Photon
#define WKP 17

// Timer pins

#define TIMER2_CH1 10
#define TIMER2_CH2 11
#define TIMER2_CH3 18
#define TIMER2_CH4 19

#define TIMER3_CH1 14
#define TIMER3_CH2 15
#define TIMER3_CH3 16
#define TIMER3_CH4 17

#define TIMER4_CH1 1
#define TIMER4_CH2 0

// SPI pins

#define SS   12
#define SCK  13
#define MISO 14
#define MOSI 15

// I2C pins

#define SDA  0
#define SCL  1

// DAC pins on Photon
#define DAC1 16
#define DAC2 13

#else
const pin_t TOTAL_PINS = 21;
const pin_t TOTAL_ANALOG_PINS = 8;
const pin_t FIRST_ANALOG_PIN = 10;


const pin_t D0 = 0;
const pin_t D1 = 1;
const pin_t D2 = 2;
const pin_t D3 = 3;
const pin_t D4 = 4;
const pin_t D5 = 5;
const pin_t D6 = 6;
const pin_t D7 = 7;

#if PLATFORM_ID<3
const pin_t LED1 = LED_USER;
#endif

const pin_t A0 = 10;
const pin_t A1 = 11;
const pin_t A2 = 12;
const pin_t A3 = 13;
const pin_t A4 = 14;
const pin_t A5 = 15;
const pin_t A6 = 16;

// WKP pin is also an ADC on Photon
const pin_t A7 = 17;

// RX and TX pins are also ADCs on Photon
const pin_t A8 = 18;
const pin_t A9 = 19;

const pin_t RX = 18;
const pin_t TX = 19;

const pin_t BTN = 20;

// WKP pin on Photon
const pin_t WKP = 17;

// Timer pins

const pin_t TIMER2_CH1 = 10;
const pin_t TIMER2_CH2 = 11;
const pin_t TIMER2_CH3 = 18;
const pin_t TIMER2_CH4 = 19;

const pin_t TIMER3_CH1 = 14;
const pin_t TIMER3_CH2 = 15;
const pin_t TIMER3_CH3 = 16;
const pin_t TIMER3_CH4 = 17;

const pin_t TIMER4_CH1 = 1;
const pin_t TIMER4_CH2 = 0;

// SPI pins

const pin_t SS   = 12;
const pin_t SCK  = 13;
const pin_t MISO = 14;
const pin_t MOSI = 15;

// I2C pins

const pin_t SDA  = 0;
const pin_t SCL  = 1;

// DAC pins on Photon
const pin_t DAC1 = 16;
const pin_t DAC2 = 13;

#endif

#define TIM_PWM_COUNTER_CLOCK_FREQ 24000000 //TIM Counter clock = 24MHz
#define TIM_PWM_FREQ 500 //500Hz

#define SERVO_TIM_PWM_COUNTER_CLOCK 1000000 //TIM Counter clock = 1MHz
#define SERVO_TIM_PWM_FREQ 50//50Hz                                                                                      //20ms = 50Hz

#define LSBFIRST 0
#define MSBFIRST 1

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif  /* __PINMAP_HAL_H */

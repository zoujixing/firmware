/**
 ******************************************************************************
 * @file    main.h
 * @author  Satish Nair, Zachary Crockett, Zach Supalla and Mohit Bhoite
 * @version V1.0.0
 * @date    13-March-2013
 * @brief   Header for main.c module
 ******************************************************************************
  Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

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
#ifndef __MAIN_H
#define __MAIN_H

extern "C" {

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif
#include "hw_config.h"
#include "spark_wlan.h"
#ifdef __cplusplus
}
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/*
 * In Eclipse Project Properties -> C/C++ Build -> Settings -> Tool Settings
 * -> ARM Sourcery Windows GCC C/C++ Compiler -> Preprocessor -> Defined symbol (-D),
 * Add : "DFU_BUILD_ENABLE"
 *
 * In Eclipse Project Properties -> C/C++ Build -> Settings -> Tool Settings
 * -> ARM Sourcery Windows GCC C/C++ Linker -> General -> Script file (-T),
 * Browse & select linker file : "linker_stm32f10x_md_dfu.ld"
 */

#ifdef DFU_BUILD_ENABLE

/*
 * Use the JTAG IOs as standard GPIOs (D3 to D7)
 * Note that once the JTAG IOs are disabled, the connection with the host debugger
 * is lost and cannot be re-established as long as the JTAG IOs remain disabled.
 */
#define SWD_JTAG_DISABLE

/*
 * Use Independent Watchdog to force a system reset when a software error occurs
 * During JTAG program/debug, the Watchdog has to be disabled so that it does not
 * upset the debugger
 */
#define IWDG_RESET_ENABLE
#define TIMING_IWDG_RELOAD	1000 //1sec

#endif

//#undef SPARK_WLAN_ENABLE

/*
 * default status, works the way it does now, no need to define explicitly
 */
//#define RGB_NOTIFICATIONS_ON
/*
 * should prevent any of the Spark notifications from ever being shown on the LED;
 * if the user never takes manual control of the LED, it should never turn on.
 */
//#define RGB_NOTIFICATIONS_OFF
/*
 * keep all of the statuses except the 'breathing cyan' - this would be a good
 * power saver while still showing when important things are happening on the Core.
 */
//#define RGB_NOTIFICATIONS_CONNECTING_ONLY

#define USART_RX_DATA_SIZE			256

/* Exported functions ------------------------------------------------------- */
void Timing_Decrement(void);

void USB_USART_Init(uint32_t baudRate);
uint8_t USB_USART_Available_Data(void);
int32_t USB_USART_Receive_Data(void);
void USB_USART_Send_Data(uint8_t Data);
void Handle_USBAsynchXfer(void);
void Get_SerialNum(void);

}

extern void delay(unsigned long ms);

#define NVIC_INT_CTRL				*((__IO uint32_t *)0xE000ED04)
#define NVIC_PENDSVSET				(1UL << 28UL)	//0x10000000
#define NVIC_PENDSVCLR				(1UL << 27UL)	//0x08000000

#define PROCESS_STACK0_SIZE			0x800	/* Process stack0 size */
#define PROCESS_STACK1_SIZE			0x400	/* Process stack1 size */

#define PROCESS_STACK_NOT_ACTIVE	-1		/* Process stack not active */
#define PROCESS_STACK0_ACTIVE		0		/* Process stack0 active */
#define PROCESS_STACK1_ACTIVE		1		/* Process stack1 active */

__attribute__( ( always_inline ) ) __STATIC_INLINE void __SVC(void)
{
	__ASM volatile
	(
			"SVC 0				\n"
	);
}

#endif /* __MAIN_H */

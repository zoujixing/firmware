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
#define NVIC_PENDSVSET				(1 << 28)	//0x10000000

#define SP_PROCESS_SIZE             0x300  /* Process stack size */
#define SP_PROCESS                  0x02   /* Process stack */
#define SP_MAIN                     0x00   /* Main stack */
#define THREAD_MODE_PRIVILEGED      0x00   /* Thread mode has privileged access */
#define THREAD_MODE_UNPRIVILEGED    0x01   /* Thread mode has unprivileged access */

__attribute__( ( always_inline ) ) __STATIC_INLINE void __SVC(void)
{
	__ASM volatile ("svc 0");
}

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_SP(void)
{
	register uint32_t result;

	__ASM volatile (
			"tst lr, #4\t\n" /* Check EXC_RETURN[2] */
			"ite eq\t\n"
			"mrseq %0, msp\t\n"
			"mrsne %0, psp\t\n"
			: "=r" (result)
	);

	return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE void __toggle_SP(void)
{
	__ASM volatile (
			"EOR lr, lr, #4	\n"
			"BX lr	\n"
	);
}

#endif /* __MAIN_H */

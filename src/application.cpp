/**
 ******************************************************************************
 * @file    application.cpp
 * @authors  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    05-November-2013
 * @brief   Tinker application
 ******************************************************************************
  Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/  
#include "application.h"


#undef SPARK_WLAN_ENABLE

void IncrRegister(uint16_t addy);
void ReadRegister(uint16_t addy);


/* This function is called once at start up ----------------------------------*/
void setup()
{
    Serial.begin(9600);
    Serial.println("Hello!");

    Delay(2500);

//WORKS
    volatile uint16_t val = BKP_ReadBackupRegister(BKP_DR10);
    BKP_WriteBackupRegister(BKP_DR10, val + 1);
    Serial.println("Set Register "+String(BKP_DR10)+"to " + String(val + 1));



    val = BKP_ReadBackupRegister(BKP_DR1);
    if (val < 1000) {
        val = 1000;
    }
    else if (val >= 50000) {
        val = 1000;
    }

    BKP_WriteBackupRegister(BKP_DR1, val + 1);
    Serial.println("Set Register "+String(BKP_DR1)+"to " + String(val + 1));



//WORKS SORTA
//ReadRegister(BKP_DR1);
//IncrRegister(BKP_DR2);
//IncrRegister(BKP_DR3);
//IncrRegister(BKP_DR4);
//IncrRegister(BKP_DR5);
//IncrRegister(BKP_DR6);
//IncrRegister(BKP_DR7);
//IncrRegister(BKP_DR8);
//IncrRegister(BKP_DR9);
//#define BKP_DR10                          ((uint16_t)0x0028)



//DOESN"T WORK
//IncrRegister(BKP_DR11);
//IncrRegister(BKP_DR12);
//IncrRegister(BKP_DR13);
//IncrRegister(BKP_DR14);
//IncrRegister(BKP_DR15);
//IncrRegister(BKP_DR16);
//IncrRegister(BKP_DR17);
//IncrRegister(BKP_DR18);
//IncrRegister(BKP_DR19);
//IncrRegister(BKP_DR20);
//IncrRegister(BKP_DR21);
//IncrRegister(BKP_DR22);
//IncrRegister(BKP_DR23);
//IncrRegister(BKP_DR24);
//IncrRegister(BKP_DR25);
//IncrRegister(BKP_DR26);
//IncrRegister(BKP_DR27);
//IncrRegister(BKP_DR28);
//IncrRegister(BKP_DR29);
//IncrRegister(BKP_DR30);
//IncrRegister(BKP_DR31);
//IncrRegister(BKP_DR32);
//IncrRegister(BKP_DR33);
//IncrRegister(BKP_DR34);
//IncrRegister(BKP_DR35);
//IncrRegister(BKP_DR36);
//IncrRegister(BKP_DR37);
//IncrRegister(BKP_DR38);
//IncrRegister(BKP_DR39);
//IncrRegister(BKP_DR40);
//IncrRegister(BKP_DR41);
//IncrRegister(BKP_DR42);

//untested?





    Serial.println("sleeping...");
    Delay(5000);

    Serial.println("resetting...");
    Delay(1000);


    USB_Cable_Config(DISABLE);
    NVIC_SystemReset();
}

/* This function loops forever --------------------------------------------*/
void loop()
{
	//This will run in a loop
}

void IncrRegister(uint16_t addy) {
    volatile uint16_t val = BKP_ReadBackupRegister(addy);
    BKP_WriteBackupRegister(addy, val + 1);
    Serial.println("Set " + String(addy) + " Register to " + String(val + 1));
    Delay(250);
}
void ReadRegister(uint16_t addy) {
    volatile uint16_t val = BKP_ReadBackupRegister(addy);

    Serial.println("Read " + String(addy) + " is " + String(val));
    Delay(250);
}
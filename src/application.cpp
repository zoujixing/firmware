#include "application.h"
#include "string.h"
#include "handshake.h"

int lockDone = false;
int unlockDone = false;

/**
	Bootloader Lock / Unlock software:
	The purpose of this firmware is to unlock / lock the bootloader region of memory
	to help prevent accidental 'bricking' of a device for those without a JTAG programmer.

	Load this firmware, and use the following to lock / unlock the core:

	D0 - HIGH
	D1 - HIGH  -> LOCK -> Core responds with A0 HIGH

	D0 - LOW
	D1 - HIGH  -> UNLOCK -> Core responds with A1 HIGH
**/


void setup()
{
	//pinMode(D0, INPUT_PULLDOWN);			//both (d0,d2) high to lock
	//pinMode(D2, INPUT_PULLDOWN);			//only d1 high to unlock

	pinMode(A0, OUTPUT);		//locked indicator pin
	pinMode(A1, OUTPUT);		//unlocked indicator pin

	LED_SetRGBColor(RGB_COLOR_BLUE);
    LED_On(LED_RGB);

	digitalWrite(A0, LOW);
	digitalWrite(A1, LOW);
}

void loop()
{
	// set RGB	
	if (lockDone) {
		LED_SetRGBColor(RGB_COLOR_RED);
		LED_On(LED_RGB);
		
		//also delete any saved smart config profiles
		
		delay(1000);
		
	}
	else if (unlockDone) {
		LED_SetRGBColor(RGB_COLOR_GREEN);
		LED_On(LED_RGB);
		delay(1000);
	}
	else {
		LED_SetRGBColor(RGB_COLOR_BLUE);
		LED_On(LED_RGB);
	}

	//optionally reset the core
	if (lockDone || unlockDone) {

		//RESET INTO DFU MODE
		FLASH_OTA_Update_SysFlag = 0x0000;
		Save_SystemFlags();
		BKP_WriteBackupRegister(BKP_DR10, 0x0000);

		USB_Cable_Config(DISABLE);
		NVIC_SystemReset();	
	}
	
	
	

	//int inpin1 = digitalRead(D0);
	//int inpin2 = digitalRead(D2);
	
	int doLock = 1;

	//if (inpin1 && inpin2) {
	if (doLock) {
		//locking
		digitalWrite(A0, HIGH);
		FLASH_WriteProtection_Enable(BOOTLOADER_FLASH_PAGES); 
	
		lockDone = 1;
	}
	//if (!inpin1 && inpin2) {
	else {
	
		//unlocking
		digitalWrite(A1, HIGH);
		FLASH_WriteProtection_Disable(BOOTLOADER_FLASH_PAGES);
		
		unlockDone = 1;
	}

	delay(50);
}





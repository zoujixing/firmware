#include "application.h"
#include "string.h"
#include "handshake.h"



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
	pinMode(D0, INPUT);			//both (d0,d2) high to lock
	pinMode(D2, INPUT);			//only d1 high to unlock

	pinMode(A0, OUTPUT);		//locked indicator pin
	pinMode(A1, OUTPUT);		//unlocked indicator pin

	LED_SetRGBColor(RGB_COLOR_BLUE);
    LED_On(LED_RGB);

	//digitalWrite(A0, HIGH);
	//digitalWrite(A1, HIGH);
}

void loop()
{
	LED_SetRGBColor(RGB_COLOR_BLUE);
    LED_On(LED_RGB);


	int inpin1 = digitalRead(D0);
	int inpin2 = digitalRead(D2);

	if (inpin1 && inpin2) {
		//locking
		digitalWrite(A0, HIGH);
		FLASH_WriteProtection_Enable(BOOTLOADER_FLASH_PAGES); 
		
	}
	else if (!inpin1 && inpin2) {
		//unlocking
		digitalWrite(A1, HIGH);
		FLASH_WriteProtection_Disable(BOOTLOADER_FLASH_PAGES);
	}

	delay(50);
}





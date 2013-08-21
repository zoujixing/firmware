#include "hw_config.h"
#include "spark_utilities.h"
#include "application.h"
#include "spark_wiring.h"
#include <string.h>

/*
#include "main.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "sst25vf_spi.h"
#include "spark_utilities.h"
*/

void setup();
void loop();
void allOff();
void handlePinMessage(int pin);
void checkButton();
void handleRGBMessage(int pin);
void setRGBLED();

int state = 0;
int lastButton = 0;
int lastButtonState = 0;
uint32_t btnTime = 0;
uint32_t debounceDelay = 50;
uint32_t msgDelay = 100;




    //in theory, these pin mappings could change, so this seems safest
    //A0 = 10,A1 = 11,A2 = 12,A3 = 13,A4 = 14,A5 = 15,A6 = 16,A7 = 17
	//D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7
    //we aren't expecting D4-D7 to work on the programming rig because of the wiring

//0,1,2,3,4,5,6,7,8,9,:,;,<,=,>,?
int testPins[] = {
    10,11,12,13,14,15,16,17,	0,1,2 /*,3,4,5,6,7*/
};
int numPins = 11;
int pins_start = (int)'0';
int pins_end = (int)'?';
char buf[256];




int numTestColors = 7;
int rgb_start = (int)'A';
int rgb_end = (int)'I';

uint32_t RGBColor = 0;
uint32_t testColors[] = {
	0xFF0000, /*RGB_COLOR_RED*/	
	0x00FF00, /*RGB_COLOR_GREEN*/	
	0x0000FF, /*RGB_COLOR_BLUE*/	
	0xFFFF00, /*RGB_COLOR_YELLOW*/	
	0x00FFFF, /*RGB_COLOR_CYAN*/	
	0xFF00FF, /*RGB_COLOR_MAGENTA*/	
	0xFFFFFF /*RGB_COLOR_WHITE*/	
};





void setup()
{
    // light em up.
    int i;
    for(i=0;i<numPins;i++) {
        pinMode(testPins[i], OUTPUT);
    }

	//BUTTON_Init(BUTTON1, BUTTON_MODE_GPIO);
	pinMode(BTN, INPUT);
	
	LED_On(LED_RGB);
	
	Serial.begin(9600);	
	//digitalWrite(D0, HIGH);
}



void loop()
{
	if ((millis() % msgDelay) == 0) {
		//Serial.println("HEY\n");
		checkButton();
	}
	setRGBLED();
	
	if (Serial.available()) {
		int c = Serial.read();		
		//char retStr[11];
		//int retLen = 0;

		if ((c >= pins_start) && (c <= pins_end)) {
		//if ((c >= 0) && (c < numPins)) {
			//if we should receive a byte value in the range of '0'-'9'
			//lets assume they would like us to turn off all the pins, and turn on just that pin.
			
			handlePinMessage(c - pins_start);
						
			//retLen = itoa(c-pins_start, retStr);
			//retStr[retLen] = '\0';
			//Serial.println(strcat(strcat("OK PIN ", retStr), "\n"));
			Serial.println("OK PIN \n");
		}
		else if ((c >= rgb_start) && (c <= rgb_end)) {
			handleRGBMessage(c - rgb_start);
						
			//retLen = itoa(c-rgb_start, retStr);
			//retStr[retLen] = '\0';
			//Serial.println(strcat(strcat("OK LED ", retStr), "\n"));
			Serial.println("OK LED \n");
		}
		else {
			//other commands...?
			Serial.print("HUH\n");
		}
	}
	
	
}

void handlePinMessage(int pin) {
    if ((pin < 0) || (pin > numPins)) {
        //shouldn't get here.
		Serial.println("ERROR: Weird pin value\n");
		return;
    }

	allOff();
	digitalWrite(testPins[pin], HIGH);
}

void allOff() {
    //	digitalWrite(D1, LOW);
    //	...
    //	digitalWrite(D3, LOW);

    //iterate over the pins, setting them all LOW

    int i;
    for(i=0;i<numPins;i++) {
        digitalWrite(testPins[i], LOW);
    }
}

void handleRGBMessage(int idx) {
	if ((idx >= 0) && (idx < numTestColors)) {
		RGBColor = testColors[idx];
	}
	else {
		RGBColor = 0;
	}
}

void setRGBLED() {
	if (RGBColor > 0) {
		LED_SetRGBColor(RGBColor);
		LED_On(LED_RGB);
	}
}




void checkButton() {
/*
	if(BUTTON_GetDebouncedTime(BUTTON1) >= 100)
	{
		int val = digitalRead(BTN);
		Serial.println((val) ? "BTN DOWN\n" : "BTN UP\n");
		BUTTON_ResetDebouncedState(BUTTON1);
	}
	*/

	int val = digitalRead(BTN);
	//if (val != lastButton) {
		Serial.println((val) ? "BTN 1\n" : "BTN 0\n");
	//	btnTime = millis();
	
	//	lastButton = val;
	//}
	/*
	if ((millis() - btnTime) > debounceDelay) {
		Serial.println("BTN 2\n");
		if (lastButtonState != val) {
			Serial.println("BTN 3\n");
			Serial.println((val) ? "BTN DOWN\n" : "BTN UP\n");
			btnTime = 0;
			lastButtonState = val;
		}
	}
	*/
}


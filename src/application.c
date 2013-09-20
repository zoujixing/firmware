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
char *coreIdToHex(unsigned char *data, int length, char* buffer);

int state = 0;
int lastButton = 0;
int lastButtonState = 0;
uint32_t btnTime = 0;
uint32_t debounceDelay = 50;
uint32_t msgDelay = 2500;
uint32_t btnDelay = 100;
int msgCount = 0;
int sendAlives = 0;



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
	
	//always take over the light.
	Set_RGBUserMode(1);
	USERLED_SetRGBColor(0xFFFFFF);
	
	//BUTTON_Init(BUTTON1, BUTTON_MODE_GPIO);
	Serial.begin(9600);	
}



void loop()
{
	if ( sendAlives ) {
		if ((millis() % msgDelay) == 0) {	
			if (msgCount == 0) {
				Serial.println("ALIVE\n");
			}
			msgCount++;
		}
		else {
			msgCount = 0;
		}
	}
	
	if ((millis() % btnDelay) == 0) {
		checkButton();
		btnDelay = 100;
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
		else if (c == 'T') {
			char buffer[32];
			unsigned char size = itoa(millis(), buffer);
			Serial.print("The time is:");
			Serial.print(buffer);
			Serial.println(":");
		}
		else if (c == 'X') {
			sendAlives = 1;
		}
		else if (c == 'Z') {
			sendAlives = 0;
		}
		else if (c == 'M') {
			
			char buffer[25];
			unsigned char coreid[12];
			memcpy(coreid, (void *)ID1, 12);
			coreIdToHex(coreid, 12, buffer);

			Serial.print("ID:");
			Serial.print(buffer);
			Serial.println(":END");
		}
		else {
			//other commands...?

			unsigned char in[1];
			in[0] = c;

			char buffer[3];		
			coreIdToHex(in, 1, buffer);

			Serial.print("HUH: ");
			Serial.println(buffer);
		}
	}
}


char *coreIdToHex(unsigned char *data, int length, char* buffer) {	
	const char * hex = "0123456789ABCDEF";	
	int i=0, a=0;
	
	for(i=0;i<length;i++) {
		unsigned char c = data[i];
		buffer[a] = hex[(c>>4) & 0xF];
		buffer[a+1] = hex[(c) & 0xF];
		a+=2;
	}
	buffer[24] = 0;	//null
	return buffer;
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
		USERLED_Off(LED_RGB);
	}
}

void setRGBLED() {
	if (RGBColor > 0) {
		USERLED_SetRGBColor(RGBColor);
		USERLED_On(LED_RGB);
	}
	else {
		//USERLED_Off(LED_RGB);
	}
}




void checkButton() {
	if(BUTTON_GetDebouncedTime(BUTTON1) >= 100)
	{
		BUTTON_ResetDebouncedState(BUTTON1);
		Serial.println("BTN DOWN\n");
	}
}


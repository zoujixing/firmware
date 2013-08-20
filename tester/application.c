#include "application.h"
#include "spark_wiring.h"
//extern uint32_t millis();
//#include "hw_config.h"
//extern void BUTTON_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);

void setup();
void loop();
void allOff();
void handlePinMessage(int pin);
void checkButton();
void handleRGBMessage(int pin);

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

int rgb_start = (int)'A';
int rgb_end = (int)'C';

void setup()
{
    // light em up.
    int i;
    for(i=0;i<numPins;i++) {
        pinMode(testPins[i], OUTPUT);
    }

	//BUTTON_Init(BUTTON1, BUTTON_MODE_GPIO);
	pinMode(BTN, INPUT);
	
	Serial.begin(9600);	
	//digitalWrite(D0, HIGH);
}



void loop()
{
	if ((millis() % msgDelay) == 0) {
		//Serial.println("HEY\n");
		checkButton();
	}
	
	if (Serial.available()) {
		int c = Serial.read();
		
		if ((c >= pins_start) && (c <= pins_end)) {
		//if ((c >= 0) && (c < numPins)) {
			//if we should receive a byte value in the range of '0'-'9'
			//lets assume they would like us to turn off all the pins, and turn on just that pin.

			handlePinMessage(c - pins_start);
			Serial.println("OK\n");
		}
		else if ((c >= rgb_start) && (c <= rgb_end)) {
			handleRGBMessage(c - rgb_start);
			Serial.println("OK\n");
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

void handleRGBMessage(int pin) {
	;
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


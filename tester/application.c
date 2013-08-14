#include "application.h"

void setup();
void loop();

void allOff();
void handlePinMessage(int32_t pin);

int state = 0;
//int theDelay = 500;

int pinRange_start = (int)'0';
int pinRange_end = (int)'9';


void setup()
{
	// light em up.
	pinMode(D0, OUTPUT);
	pinMode(D1, OUTPUT);
	pinMode(D2, OUTPUT);
	pinMode(D3, OUTPUT);
	pinMode(D4, OUTPUT);
	pinMode(D5, OUTPUT);
	pinMode(D6, OUTPUT);	
	pinMode(D7, OUTPUT);
	
	//
	Serial.begin(9600);	
	digitalWrite(D0, HIGH);	
}


void loop()
{
	if (Serial.available()) {
		int32_t c = Serial.read();

		
		if ((c >= pinRange_start) && (c <= pinRange_end)) {
			//if we should receive a byte value in the range of '0'-'9'
			//lets assume they would like us to turn off all the pins, and turn on just that pin.			
			handlePinMessage(c);
			Serial.println("You got it.");
		}
		else {
			//other commands...
			Serial.println("I didn't understand that.");
		}
	}
}

void handlePinMessage(int32_t pin) {
	//D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7
	pin = pin - (int)pinRange_start;
	if ((pin < 0) || (pin > 7)) {
		//shouldn't get here.
		Serial.println("Uh oh, weird pin value came through");
		return;
	}
	
	allOff();
	digitalWrite(pin, HIGH);	
	//digitalWrite(D0, (state) ? HIGH : LOW);
	//state = (state) ? 0 : 1;
}

void allOff() {
	digitalWrite(D0, LOW);
	digitalWrite(D1, LOW);
	digitalWrite(D2, LOW);
	digitalWrite(D3, LOW);
	digitalWrite(D4, LOW);
	digitalWrite(D5, LOW);
	digitalWrite(D6, LOW);
	digitalWrite(D7, LOW);
}



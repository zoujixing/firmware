#include "application.h"

void setup();
void loop();

int state = 0;
int theDelay = 500;


void setup()
{
	// Serial Test
	pinMode(D0, OUTPUT);
	Serial.begin(9600);	
	digitalWrite(D0, HIGH);
	
	/*
	int i=0;
	for(i=0;i<1000;i++) {
		Serial.println("THE PAIN! OH THE PAIN OF IT ALL! ");
		
		digitalWrite(D0, (state) ? HIGH : LOW);	
		state = (state) ? 0 : 1;
		
		Delay(250);
	}
	*/	
}

void loop()
{
	//D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7
	


	Serial.println("I know right?");		
	digitalWrite(D0, (state) ? HIGH : LOW);	
	state = (state) ? 0 : 1;		
	Delay(theDelay);
	
	if (Serial.available()) {
		int32_t c = Serial.read();
		if (c == (int)'+') {
			theDelay = theDelay + 100;
		}
		else if (c == (int)'-') {
			theDelay = theDelay - 100;
		}
	}
}

void handlePinMessage(int32_t pin) {

}




#include "application.h"


void setup();
void loop();


void setup()
{
	// Serial Test
	Serial.begin(9600);
}

void loop()
{
	if(Serial.available())
	{
		Serial.write("a");
	}
	
	// runs repeatedly
	Delay(250);
}
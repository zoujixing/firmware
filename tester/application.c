#include "application.h"

void setup();
void loop();


void setup()
{

	// Serial Test
	Serial.begin(9600);
	//delay(500);

	// Serial print test
	Serial.print("Hello ");
	Serial.println("Spark");
}

void loop()
{
	// runs repeatedly
	//delay(500);

	// Serial loopback test: what is typed on serial console
	// using Hyperterminal/Putty should echo back on the console
	if(Serial.available())
	{
		//Serial.write("you said: ");
		Serial.write(Serial.read());
	} 
}
/*
void SystemInit() {
	main();
}

int main() {
	setup();
	while(1) {	
		loop();
	}
	return 0;
}
*/

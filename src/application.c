#include "application.h"

#define LEFT  A7
#define RIGHT D1

int i;

void setup()
{
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
	analogWrite(LEFT, 0);
	analogWrite(RIGHT, 0);
	delay(5000);

	// speed up forward
	for (i = 0; i < 250; ++i)
	{
		analogWrite(LEFT, i);
		analogWrite(RIGHT, i);
		delay(4);
	}

	// turn left
	for (i = 249; i >=0; --i)
	{
		analogWrite(LEFT, i);
		delay(2);
	}
	delay(750);

	// stutter forward
	analogWrite(LEFT, 0);
	analogWrite(RIGHT, 0);
	delay(500);
	analogWrite(LEFT, 250);
	analogWrite(RIGHT, 250);
	delay(500);
	analogWrite(LEFT, 0);
	analogWrite(RIGHT, 0);
	delay(500);
	analogWrite(LEFT, 250);
	analogWrite(RIGHT, 250);
	delay(1000);

	// spin right for a full second
	analogWrite(RIGHT, 0);
	delay(2500);

	// forward
	for (i = 0; i < 250; ++i)
	{
		analogWrite(RIGHT, i);
		delay(2);
	}
	delay(1500);

	// slow down and stop
	for (i = 255; i >= 0; --i)
	{
		analogWrite(LEFT, i);
		analogWrite(RIGHT, i);
		delay(20);
	}
}

void loop()
{
}

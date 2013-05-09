#include "application.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

const int redLED = A4;
const int greenLED = A5;
const int blueLED = A6;
const int whiteLED = A7;

int gammaCorrect(const int input, const int power)
{
	return (int)(pow(input / 256.0, power) * 256);
}

void writeGammaCorrectedColor(const char *color, const int pin, const int power)
{
	char colorString[3] = "00\0";
	int colorVal;

	colorString[0] = *color;
	colorString[1] = *(color + 1);
	colorVal = strtol(colorString, NULL, 16);
	analogWrite(pin, gammaCorrect(colorVal, power));
}

char userFunction(char *message)
{
	if (8 > strlen(message))
		return -1;

	writeGammaCorrectedColor(message, redLED, 2);
	writeGammaCorrectedColor(message + 2, greenLED, 1);
	writeGammaCorrectedColor(message + 4, blueLED, 4);
	writeGammaCorrectedColor(message + 6, whiteLED, 2);
	return 0;
}


void setup()  {
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);

  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(whiteLED, LOW);
}

void loop()
{
}

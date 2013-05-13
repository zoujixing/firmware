#include "application.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

const float FADE_MILLIS = 400.0;
const int redLED   = A4;
const int greenLED = A5;
const int blueLED  = A6;
const int whiteLED = A7;

uint32_t startedFadingTime;

typedef struct {
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	unsigned char white;
} color_t;

color_t fadingFromColor;
color_t fadingToColor;

unsigned char gammaCorrect(const int input, const int power)
{
	return (unsigned char)(pow(input / 256.0, power) * 256);
}

unsigned char gammaCorrectString(const char *color, const int power)
{
	char colorString[3] = "00\0";
	int colorVal;

	colorString[0] = *color;
	colorString[1] = *(color + 1);
	colorVal = strtol(colorString, NULL, 16);
	return gammaCorrect(colorVal, power);
}

char userFunction(char *message)
{
	if (8 > strlen(message))
		return 1;

	startedFadingTime = millis();

	fadingFromColor.red   = fadingToColor.red;
	fadingFromColor.green = fadingToColor.green;
	fadingFromColor.blue  = fadingToColor.blue;
	fadingFromColor.white = fadingToColor.white;

	fadingToColor.red   = gammaCorrectString(message, 2);
	fadingToColor.green = gammaCorrectString(message + 2, 1);
	fadingToColor.blue  = gammaCorrectString(message + 4, 4);
	fadingToColor.white = gammaCorrectString(message + 6, 2);

	return 0;
}

void setup()  {
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);

  fadingToColor.red   = 0;
  fadingToColor.green = 0;
  fadingToColor.blue  = 0;
  fadingToColor.white = 0;

  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(whiteLED, LOW);

  delay(2000);
  userFunction("ffffffff");
}

void loop()
{
	uint32_t beenFadingMilliseconds = millis() - startedFadingTime;
	if (beenFadingMilliseconds < FADE_MILLIS)
	{
		float fractionFaded = beenFadingMilliseconds / FADE_MILLIS;

		analogWrite(redLED,   (fadingToColor.red   - fadingFromColor.red)   * fractionFaded + fadingFromColor.red);
		analogWrite(greenLED, (fadingToColor.green - fadingFromColor.green) * fractionFaded + fadingFromColor.green);
		analogWrite(blueLED,  (fadingToColor.blue  - fadingFromColor.blue)  * fractionFaded + fadingFromColor.blue);
		analogWrite(whiteLED, (fadingToColor.white - fadingFromColor.white) * fractionFaded + fadingFromColor.white);
	}
}

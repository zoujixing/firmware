#include "application.h"
#include <stdlib.h>

#define LEFT  A7
#define RIGHT D1

void setup()
{
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
}

void loop()
{
}

char userFunction(char *message)
{
	long val = strtol(message + 1, NULL, 10);
	if ('L' == *message)
	{
		analogWrite(LEFT, val);
		return 1;
	}
	else if ('R' == *message)
	{
		analogWrite(RIGHT, val);
		return 2;
	}
	return 0;
}

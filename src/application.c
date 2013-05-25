#include "application.h"
#include <stdlib.h>
#include <string.h>

// see http://www.dfrobot.com/wiki/index.php?title=Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)

#define LEFT_PWM   A7 // Motor Shield E1, Arduino D6
#define RIGHT_PWM  D1 // Motor Shield E2, Arduino D5

#define LEFT_BACK  D4 // Motor Shield M1, Arduino D7
#define RIGHT_BACK D3 // Motor Shield M2, Arduino D4

#define FORWARD_BACK_MULTIPLIER  (1.5)
#define TURNING_MULTIPLIER       (1.5)

void setup()
{
	pinMode(LEFT_PWM, OUTPUT);
	pinMode(RIGHT_PWM, OUTPUT);
	pinMode(LEFT_BACK, OUTPUT);
	pinMode(RIGHT_BACK, OUTPUT);
}

void loop()
{
}

void drive(const char *command)
{
	long val = strtol(command + 3, NULL, 10);
	if (0 == strncmp(command, "fd", 2))
	{
		digitalWrite(LEFT_PWM, HIGH);
		digitalWrite(RIGHT_PWM, HIGH);
		digitalWrite(LEFT_BACK, LOW);
		digitalWrite(RIGHT_BACK, LOW);
		delay(val * FORWARD_BACK_MULTIPLIER);
	}
	else if (0 == strncmp(command, "bk", 2))
	{
		digitalWrite(LEFT_PWM, HIGH);
		digitalWrite(RIGHT_PWM, HIGH);
		digitalWrite(LEFT_BACK, HIGH);
		digitalWrite(RIGHT_BACK, HIGH);
		delay(val * FORWARD_BACK_MULTIPLIER);
	}
	else if (0 == strncmp(command, "rt", 2))
	{
		digitalWrite(LEFT_PWM, HIGH);
		digitalWrite(RIGHT_PWM, LOW);
		digitalWrite(LEFT_BACK, LOW);
		digitalWrite(RIGHT_BACK, LOW);
		delay(val * TURNING_MULTIPLIER);
	}
	else if (0 == strncmp(command, "lt", 2))
	{
		digitalWrite(LEFT_PWM, LOW);
		digitalWrite(RIGHT_PWM, HIGH);
		digitalWrite(LEFT_BACK, LOW);
		digitalWrite(RIGHT_BACK, LOW);
		delay(val * TURNING_MULTIPLIER);
	}
	digitalWrite(LEFT_PWM, LOW);
	digitalWrite(RIGHT_PWM, LOW);
	digitalWrite(LEFT_BACK, LOW);
	digitalWrite(RIGHT_BACK, LOW);
}

char userFunction(char *message)
{
	char command[7] = "\0\0\0\0\0\0";
	size_t length;
	char *comma = strchr(message, ',');
	while (NULL != comma)
	{
		length = comma - message;
		strncpy(command, message, length);
		command[length] = '\0';
		drive(command);
		message = comma + 1;
		comma = strchr(message, ',');
	}
	strncpy(command, message, 6);
	drive(command);
	return 0;
}

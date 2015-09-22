#include "application.h"

#include "Serial3/Serial3.h"
#include "core_hal.h"

SYSTEM_MODE(MANUAL);

#define now() millis()
uint32_t lastFlash = now();
String com = "";
#define PASS_GREEN() RGB.color(0,255,0)
#define FAIL_RED() RGB.color(255,0,0)
#define FAIL_BLUE() RGB.color(0,0,255)
#define RGB_OFF() RGB.color(0,0,0)

int8_t testUbloxIsReset();
void   clearUbloxBuffer();
int8_t sendATcommand(const char* ATcommand, const char* expected_answer1, system_tick_t timeout);
int8_t testATOK(system_tick_t timeout);

void setup()
{
	RGB.control(true);

	pinMode(PWR_UC, OUTPUT);
	pinMode(RESET_UC, OUTPUT);
	digitalWrite(PWR_UC, HIGH);
	digitalWrite(RESET_UC, HIGH);
	pinMode(RTS_UC, OUTPUT);
	digitalWrite(RTS_UC, LOW); // VERY IMPORTANT FOR CORRECT OPERATION!!
	pinMode(LVLOE_UC, OUTPUT);
	digitalWrite(LVLOE_UC, LOW); // VERY IMPORTANT FOR CORRECT OPERATION!!

	Serial.begin(9600);
	Serial3.begin(9600);

	Serial.println("\e[0;36mHi, I'm the Electron!");
	Serial.println("\e[0;36mType in \"AT\" and press enter to see an \"OK\" response.\e[0;35m");

	// TEST RGB LED
	FAIL_RED();
	delay(100);
	PASS_GREEN();
	delay(100);
	FAIL_BLUE();
	delay(100);
	RGB_OFF();
	delay(500);

	// Test for AK OK, power cycle until we see it
	//Serial.println("\e[0;32mTest if AT == OK...");
	while ( !testATOK(500) ) { FAIL_RED(); } PASS_GREEN();
	delay(2000);
}

void loop()
{
	if ( Serial.available() ) {
		char c = Serial.read();
		com += c;
		Serial.write(c); //echo input
		if (c == '\r') {
			Serial3.print(com);
			com = "";
		}
	}

	if (Serial3.available()) {
		char c = Serial3.read();
		Serial.write(c);
	}
}

void clearUbloxBuffer() {
  while (Serial3.available()) {
	Serial3.read();
  }
}

int8_t sendATcommand(const char* ATcommand, const char* expected_answer1, system_tick_t timeout) {
	uint8_t x = 0, rv = 0;
	char response[100];
	system_tick_t previous;

	memset(response, '\0', 100);
	delay(100);

	clearUbloxBuffer();

	Serial3.println(ATcommand);

	previous = now();
	do {
		if (Serial3.available() != 0) {
			response[x] = Serial3.read();
			x++;

			// check whether we've received the expected answer
			if (strstr(response, expected_answer1) != NULL) {
				rv = 1;
			}
		}
	} while ((rv == 0) && ((now() - previous) < timeout));

	//Serial.print("\e[0;35m");
	//Serial.println(response);
	return rv;
}

int8_t testATOK(system_tick_t timeout) {
	int8_t rv = sendATcommand("AT", "OK", timeout);
	return rv;
}

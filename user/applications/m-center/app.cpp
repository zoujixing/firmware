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

	Serial.begin(115200);
	Serial3.begin(115200);

	// TEST RGB LED
	FAIL_RED();
	delay(100);
	PASS_GREEN();
	delay(100);
	FAIL_BLUE();
	delay(100);
	RGB_OFF();
	delay(500);

	int i = 10;
	uint8_t r  = 0;
	while (i--) {
        // SARA-U2/LISA-U2 50..80us
        digitalWrite(PWR_UC, 0); delay(50);
        digitalWrite(PWR_UC, 1); delay(10);

        // SARA-G35 >5ms, LISA-C2 > 150ms, LEON-G2 >5ms
        digitalWrite(PWR_UC, 0); delay(150);
        digitalWrite(PWR_UC, 1); delay(100);

        // purge any messages
        clearUbloxBuffer();

        // check interface
        r = testATOK(1000);
        if(r) {
        	PASS_GREEN();
            break;
        }
    }
    if (i < 0) {
        Serial.println("No Reply from Modem, try power cycling.\r\n");
        FAIL_RED();
        return;
    }

    r = sendATcommand("AT+IPR=115200\r\n", "OK", 1000);
    if(!r) {
        FAIL_RED();
        return;
    }
    // wait some time until baudrate is applied
    delay(100); // SARA-G > 40ms

    Serial.println("Hi, I'm the Electron!");
	Serial.println("Type in \"AT\" and press enter to see an \"OK\" response.");
}

void loop()
{
	// if ( Serial.available() ) {
	// 	char c = Serial.read();
	// 	com += c;
	// 	Serial.write(c); //echo input
	// 	if (c == '\r') {
	// 		Serial3.print(com);
	// 		com = "";
	// 	}
	// }

	if (Serial.available()) {
		char c = Serial.read();
		Serial3.write(c);
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

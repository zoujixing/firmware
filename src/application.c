#include "hw_config.h"
#include "spark_utilities.h"
#include "application.h"
#include "spark_wiring.h"
#include <string.h>
#include "socket.h"
#include "netapp.h"
#include <stdlib.h>

/*
#include "main.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "sst25vf_spi.h"
#include "spark_utilities.h"
*/

void setup();
void loop();
void allOff();
void handlePinMessage(int pin);
void checkButton();
void handleRGBMessage(int pin);
void setRGBLED();
void wifiEnable();
void wifiDisable();

uint8_t serialAvailable();
int32_t serialRead();
void serialPrintln(const char * str);
void serialPrint(const char * str);

char *coreIdToHex(unsigned char *data, int length, char* buffer);

int state = 0;
int lastButton = 0;
int lastButtonState = 0;
uint32_t btnTime = 0;
uint32_t debounceDelay = 50;
uint32_t msgDelay = 2500;
uint32_t btnDelay = 100;
int msgCount = 0;
int sendAlives = 0;



    //in theory, these pin mappings could change, so this seems safest
    //A0 = 10,A1 = 11,A2 = 12,A3 = 13,A4 = 14,A5 = 15,A6 = 16,A7 = 17
	//D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7
    //we aren't expecting D4-D7 to work on the programming rig because of the wiring

//0,1,2,3,4,5,6,7,8,9,:,;,<,=,>,?
int testPins[] = {
    10,11,12,13,14,15,16,17,	0,1,2 /*,3,4,5,6,7*/
};
int numPins = 11;
int pins_start = (int)'0';
int pins_end = (int)'?';
char buf[256];




int numTestColors = 7;
int rgb_start = (int)'A';
int rgb_end = (int)'I';

uint32_t RGBColor = 0;
uint32_t testColors[] = {
	0xFF0000, /*RGB_COLOR_RED*/	
	0x00FF00, /*RGB_COLOR_GREEN*/	
	0x0000FF, /*RGB_COLOR_BLUE*/	
	0xFFFF00, /*RGB_COLOR_YELLOW*/	
	0x00FFFF, /*RGB_COLOR_CYAN*/	
	0xFF00FF, /*RGB_COLOR_MAGENTA*/	
	0xFFFFFF /*RGB_COLOR_WHITE*/	
};


/* -------------WIFI ------------------ */
extern uint8_t WLAN_MANUAL_CONNECT;
extern uint8_t WLAN_DHCP;
extern uint8_t SPARK_SOCKET_CONNECTED;
extern uint8_t SPARK_DEVICE_ACKED;

void setup();
void loop();
void checkWifiSerial(char c);
void tester_connect(char *ssid, char *pass);
void tester_ping(char *ip, char *port, char *msg);
void tokenizeCommand(char *cmd, char** parts);
int* parseIP(char *ip, int *parts);
void printIP(int* parts);

int Connect_IP(char *ip, int port, char *msg);
int Disconnect_IP(void);
long testSocket = 0;
int notYetConnected = 1;
int notifiedAboutDHCP = 0;

int cmd_index = 0, cmd_length = 256;
char command[256];
const char cmd_CONNECT[] = "CONNECT:";
const char cmd_OPEN[] = "OPEN:";
const char cmd_PARSE[] = "PARSE:";
const char cmd_ENABLE[] = "ENABLE:";
const char cmd_DISABLE[] = "DISABLE:";


/* -------------WIFI ------------------ */




void setup()
{
    // light em up.
    int i;
    for(i=0;i<numPins;i++) {
        pinMode(testPins[i], OUTPUT);
	}
	
	//always take over the light.
	Set_RGBUserMode(1);
	USERLED_SetRGBColor(0xFFFFFF);
	
	//BUTTON_Init(BUTTON1, BUTTON_MODE_GPIO);
	Serial.begin(9600);
	Serial1.begin(9600);	
}


//DEUBG
void FLASH_Read_ServerPublicKey(uint8_t *keyBuffer)
{
	sFLASH_ReadBuffer(keyBuffer, 0x00001000, 294);
}


void loop()
{
	if ( sendAlives ) {
		if ((millis() % msgDelay) == 0) {	
			if (msgCount == 0) {
				serialPrintln("ALIVE\n");
			}
			msgCount++;
		}
		else {
			msgCount = 0;
		}
	}
	
	if ((millis() % btnDelay) == 0) {
		checkButton();
		btnDelay = 100;
	}


	if (!WLAN_MANUAL_CONNECT && WLAN_DHCP && !notifiedAboutDHCP) {
		notifiedAboutDHCP = 1;
		serialPrintln(" DHCP DHCP DHCP DHCP: We have DHCP! DHCP DHCP ");
		serialPrintln(" DHCP DHCP DHCP DHCP: We have DHCP! DHCP DHCP ");
		SPARK_SOCKET_CONNECTED = 1;
		SPARK_DEVICE_ACKED = 1;
		RGBColor = 0x00FF00;		//green
		//USERLED_SetRGBColor(0x00FF00);		//green
		//USERLED_On(LED_RGB);
	}

	setRGBLED();

	
	if (serialAvailable()) {
		int c = serialRead();
		
		checkWifiSerial((char)c);


		if ((c >= pins_start) && (c <= pins_end)) {
		//if ((c >= 0) && (c < numPins)) {
			//if we should receive a byte value in the range of '0'-'9'
			//lets assume they would like us to turn off all the pins, and turn on just that pin.
			
			handlePinMessage(c - pins_start);
						
			//retLen = itoa(c-pins_start, retStr);
			//retStr[retLen] = '\0';
			//serialPrintln(strcat(strcat("OK PIN ", retStr), "\n"));
			serialPrintln("OK PIN \n");
		}
		else if ((c >= rgb_start) && (c <= rgb_end)) {
			handleRGBMessage(c - rgb_start);
						
			//retLen = itoa(c-rgb_start, retStr);
			//retStr[retLen] = '\0';
			//serialPrintln(strcat(strcat("OK LED ", retStr), "\n"));
			serialPrintln("OK LED \n");
		}
		else if (c == 'T') {
			char buffer[32];
			unsigned char size = itoa(millis(), buffer);
			buffer[size] = '\0';

			serialPrint("The time is:");
			serialPrint(buffer);
			serialPrintln(":");
		}
		else if (c == 'X') {
			sendAlives = 1;
		}
		else if (c == 'Z') {
			sendAlives = 0;
		}
		else if (c == 'V') {
			serialPrintln("Serial+Pin+Wifi+RTC+RGB Tester!");
		}
		else if (c == 'M') {
			
			char buffer[25];
			unsigned char coreid[12];
			memcpy(coreid, (void *)ID1, 12);
			coreIdToHex(coreid, 12, buffer);

			serialPrint("ID:");
			serialPrint(buffer);
			serialPrintln(":END");
		}
		else if (c == ' ') {
			;
		}
		else if  (c == 'j') {
			unsigned char pubkey[294];
			char buffer[588];
			FLASH_Read_ServerPublicKey(pubkey);
			coreIdToHex(pubkey, 294, buffer);

			serialPrintln("public key is");
			serialPrintln(buffer);
		}
		else {
			//other commands...?

			unsigned char in[1];
			in[0] = c;

			char buffer[3];		
			coreIdToHex(in, 1, buffer);

			serialPrint("HUH: ");
			serialPrintln(buffer);
		}
	}
}

void checkWifiSerial(char c) {
	if (cmd_index < cmd_length) {
		command[cmd_index] = c;
		cmd_index++;
	}
	else {
		cmd_index = 0;
	}

	if (c == ' ') {
		//reset the command index.
		cmd_index = 0;
	}
	else if (c == ';') {
		serialPrintln("got semicolon.");
		serialPrint("checking command: ");
		serialPrintln(command);

		char *parts[5];			
		char *start;
			
		if (start = strstr(command, cmd_ENABLE)) {
			serialPrintln("enable wifi...");
			//initializeWifi();
			wifiEnable();
			serialPrintln("DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE");
		}
		else if (start = strstr(command, cmd_DISABLE)) {
			serialPrintln("disable wifi...");
			//initializeWifi();
			wifiDisable();
			serialPrintln("DONE DONE DONE DONE DONE DONE DONE DONE DONE DONE");
		}
		else if (start = strstr(command, cmd_CONNECT)) {
			cmd_index = 0;
				
			serialPrintln("tokenizing...");
				
			//expecting CONNECT:SSID:PASS;
			tokenizeCommand(start, parts);
			serialPrintln("parts...");
			serialPrintln(parts[0]);
			serialPrintln(parts[1]);
			serialPrintln(parts[2]);

			serialPrintln("connecting...");
			tester_connect(parts[1], parts[2]);	
		}
		else if (start = strstr(command, cmd_OPEN)) { 
			cmd_index = 0;
			
			//expecting OPEN:IP:PORT:MSG;
			serialPrintln("tokenizing...");
			tokenizeCommand(start, parts);
				
			serialPrintln(parts[0]);
			serialPrintln(parts[1]);
			serialPrintln(parts[2]);
			serialPrintln(parts[3]);
				
			serialPrintln("sending...");
			//char *ip, char *port, char *msg
			tester_ping(parts[1], parts[2], parts[3]);
				
		}
		else if (start = strstr(command, cmd_PARSE)) {
			cmd_index = 0;
				
			serialPrintln("tokenizing...");
			tokenizeCommand(start, parts);
				
			serialPrintln(parts[0]);
			serialPrintln(parts[1]);
			serialPrintln(parts[2]);
			serialPrintln(parts[3]);
				
			int ip[4];
			parseIP(parts[1], ip);
			printIP(ip);
				
			int test[4];
			test[0] = 11;
			test[1] = 22;
			test[2] = 33;
			test[3] = 44;
			printIP(test);
		}
	}

}



char *coreIdToHex(unsigned char *data, int length, char* buffer) {	
	const char * hex = "0123456789ABCDEF";	
	int i=0, a=0;
	
	for(i=0;i<length;i++) {
		unsigned char c = data[i];
		buffer[a] = hex[(c>>4) & 0xF];
		buffer[a+1] = hex[(c) & 0xF];
		a+=2;
	}
	buffer[24] = 0;	//null
	return buffer;
}

void handlePinMessage(int pin) {
    if ((pin < 0) || (pin > numPins)) {
        //shouldn't get here.
		serialPrintln("ERROR: Weird pin value\n");
		return;
    }

	allOff();
	digitalWrite(testPins[pin], HIGH);
}

void allOff() {
    //	digitalWrite(D1, LOW);
    //	...
    //	digitalWrite(D3, LOW);

    //iterate over the pins, setting them all LOW

    int i;
    for(i=0;i<numPins;i++) {
        digitalWrite(testPins[i], LOW);
    }
}

void handleRGBMessage(int idx) {
	if ((idx >= 0) && (idx < numTestColors)) {
		RGBColor = testColors[idx];
	}
	else {
		RGBColor = 0;
		USERLED_Off(LED_RGB);
	}
}

void setRGBLED() {
	if (RGBColor > 0) {
		USERLED_SetRGBColor(RGBColor);
		USERLED_On(LED_RGB);
	}
	else {
		//USERLED_Off(LED_RGB);
	}
}




void checkButton() {
	if(BUTTON_GetDebouncedTime(BUTTON1) >= 100)
	{
		BUTTON_ResetDebouncedState(BUTTON1);
		serialPrintln("BTN DOWN\n");
	}
}




/* -------------WIFI ------------------ */

void tester_connect(char *ssid, char *pass) {

	wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);
	wlan_connect(WLAN_SEC_WPA2, ssid, strlen(ssid), NULL, pass, strlen(pass));
	WLAN_MANUAL_CONNECT = 0;
	
	RGBColor = 0xFF00FF;		//purple
	//USERLED_SetRGBColor(0xFF00FF);		//purple
	//USERLED_On(LED_RGB);
	serialPrintln("  WIFI Connected?    ");
}

void tester_ping(char *ip, char *port, char *msg) {

	Connect_IP(ip, 8989, msg);
	serialPrintln("  Msg sent?    ");
	RGBColor = 0x00FFFF;		//cyan
	//USERLED_SetRGBColor(0x00FFFF);		//cyan?
	//USERLED_On(LED_RGB);
}


void tokenizeCommand(char *cmd, char* parts[]) {
	char * pch;
	int idx = 0;
	
	//printf ("Splitting string \"%s\" into tokens:\n", cmd);
	pch = strtok (cmd,":;");
	while (pch != NULL)
	{
		if (idx < 5) {
			parts[idx++] = pch;
		}
		pch = strtok (NULL, ":;");
	}
}


int* parseIP(char *ip, int* parts) {
	int idx = 0;
	char *pch = strtok (ip,".");
	while (pch != NULL)
	{
		if (idx < 4) {
			parts[idx++] = (char)atoi(pch);
		}
		pch = strtok (NULL, ".");
	}
	return parts;
}

void printIP(int* parts) {
	char ip[6];
	int i=0;
	for(i=0;i<4;i++) {
		itoa(parts[i], ip);
		serialPrint(ip);
		serialPrint(".");
	}
}

int Connect_IP(char *ip, int port, char *msg)
{
	int parts[4];
	parseIP(ip, parts);
	int retVal = 0;

    long testSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (testSocket < 0)
    {
        //wlan_stop();
        return -1;
    }

	// the family is always AF_INET
	sockaddr tSocketAddr;
    tSocketAddr.sa_family = AF_INET;

	// the destination port
    tSocketAddr.sa_data[0] = (SPARK_SERVER_PORT & 0xFF00) >> 8;
    tSocketAddr.sa_data[1] = (SPARK_SERVER_PORT & 0x00FF);
	
	// the destination IP address
	tSocketAddr.sa_data[2] = parts[0];	// First Octet of destination IP
	tSocketAddr.sa_data[3] = parts[1];	// Second Octet of destination IP
	tSocketAddr.sa_data[4] = parts[2]; 	// Third Octet of destination IP
	tSocketAddr.sa_data[5] = parts[3];	// Fourth Octet of destination IP

	retVal = connect(testSocket, &tSocketAddr, sizeof(tSocketAddr));

	if (retVal < 0)
	{
		// Unable to connect
		serialPrintln("Not connected?");
		return -1;
	}
	else
	{
		serialPrintln("Connected?! sending msg...");
		retVal = send(testSocket, msg, strlen(msg), 0);
	}

    return retVal;
}

int Disconnect_IP(void)
{
    int retVal = 0;

    retVal = closesocket(testSocket);

    if(retVal == 0)
    	testSocket = 0xFFFFFFFF;

    return retVal;
}




uint8_t serialAvailable() {
	return Serial.available() | Serial1.available();
}

int32_t serialRead() {
	if (Serial.available()) {
		return Serial.read();
	}
	else if (Serial1.available()) {
		return Serial1.read();
	}
	return 0;
}


void serialPrintln(const char * str) {
	Serial.println(str);
	Serial1.println(str);
}
void serialPrint(const char * str) {
	Serial.print(str);
	Serial1.print(str);
}


//
//void initializeWifi() {
//	/* Initialize CC3000's CS, EN and INT pins to their default states */
//	CC3000_WIFI_Init();
//
//	/* Configure & initialize CC3000 SPI_DMA Interface */
//	CC3000_SPI_DMA_Init();
//
//	/* WLAN On API Implementation */
//	wlan_init(WLAN_Async_Callback, WLAN_Firmware_Patch, WLAN_Driver_Patch, WLAN_BootLoader_Patch,
//				CC3000_Read_Interrupt_Pin, CC3000_Interrupt_Enable, CC3000_Interrupt_Disable, CC3000_Write_Enable_Pin);
//
//	Delay(100);
//
//	/* Trigger a WLAN device */
//	wlan_start(0);
//
//	/* Mask out all non-required events from CC3000 */
//	wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT | HCI_EVNT_WLAN_ASYNC_PING_REPORT);
//
//	//if(nvmem_read(NVMEM_SPARK_FILE_ID, NVMEM_SPARK_FILE_SIZE, 0, NVMEM_Spark_File_Data) != 0)
//	//{
//		/* Delete all previously stored wlan profiles */
//		wlan_ioctl_del_profile(255);
//
//		/* Create new entry for Spark File in CC3000 EEPROM */
//		//nvmem_create_entry(NVMEM_SPARK_FILE_ID, NVMEM_SPARK_FILE_SIZE);
//		//nvmem_write(NVMEM_SPARK_FILE_ID, NVMEM_SPARK_FILE_SIZE, 0, NVMEM_Spark_File_Data);
//	//}
//
//	//Spark_Error_Count = NVMEM_Spark_File_Data[ERROR_COUNT_FILE_OFFSET];
//
//	Delay(1000);
//}

void wifiEnable() {
	/* Disable CC3000 */
	CC3000_Write_Enable_Pin(WLAN_ENABLE);

	wlan_start(0);
}

void wifiDisable() {
	/* Disable CC3000 */
	CC3000_Write_Enable_Pin(WLAN_DISABLE);

	//WLAN_CONNECTED = 0;
	//WLAN_DHCP = 0;
	//SPARK_WLAN_RESET = 0;
	//SPARK_WLAN_STARTED = 0;
	//SPARK_SOCKET_CONNECTED = 0;
	//SPARK_SOCKET_ALIVE = 0;
	//SPARK_DEVICE_ACKED = 0;
	//SPARK_FLASH_UPDATE = 0;
	//SPARK_LED_FADE = 0;
	//Spark_Error_Count = 0;
	//TimingSparkProcessAPI = 0;
	//TimingSparkAliveTimeout = 0;

	//CC3000_Write_Enable_Pin(WLAN_DISABLE);

}

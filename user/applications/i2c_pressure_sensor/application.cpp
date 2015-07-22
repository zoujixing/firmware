#include "ssc.h"

//Sensor part no. tested :  SSCMRNN001PG2A3
//Example code: http://playground.arduino.cc/Main/HoneywellTruStabilitySSC-HSCPressureSensors

//create an SSC sensor with I2C address 0x28
SSC ssc(0x28);

void setup() 
{
    Serial.begin(115200);
    Wire.begin();

    //  set min / max reading and pressure, see datasheet for the values for your
    //  sensor
    ssc.setMinRaw(0);
    ssc.setMaxRaw(16383);
    ssc.setMinPressure(0.0);
    ssc.setMaxPressure(1.0);

    //  start the sensor
    Serial.print("start()\t\t");
    Serial.println(ssc.start());
} 

void loop() 
{
    //  update pressure / temperature
    Serial.print("update()\t");
    Serial.println(ssc.update());

    // print pressure
    Serial.print("pressure()\t");
    Serial.print(ssc.pressure_Raw());
    Serial.print(" : ");
    Serial.println(ssc.pressure());

    // print temperature
    Serial.print("temperature()\t");
    Serial.print(ssc.temperature_Raw());
    Serial.print(" : ");
    Serial.println(ssc.temperature());

    Serial.println("");

/*
    while(1) {
        // use Serial as a command stream
        while(Serial.available()) {
            ssc.commandRequest(Serial);
        }
        delay(100);
    }
*/

    delay(1000);
}

#include "application.h"

const int redLED = 3;
const int greenLED = 5;
const int blueLED = 6;
const int whiteLED = 9;

int i;

void setup()  {
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);
}

void loop()
{
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(whiteLED, LOW);

  delay(1000);

  // Red
  for(i=0; i<255; i++) {
    analogWrite(redLED, i);
    delay(4);
  }

  delay(1000);

  // Green
  for(i=0; i<255; i++) {
    analogWrite(redLED, 255-i);
    analogWrite(greenLED, i);
    delay(4);
  }

  delay(1000);

  // Blue
  for(i=0; i<255; i++) {
    analogWrite(greenLED, 255-i);
    analogWrite(blueLED, i);
    delay(4);
  }

  delay(1000);

  // Red - Green
  for(i=0; i<255; i++) {
    analogWrite(redLED, i);
    analogWrite(greenLED, i);
    analogWrite(blueLED, 255-i);
    delay(4);
  }

  delay(1000);

  // Green - Blue
  for(i=0; i<255; i++) {
    analogWrite(redLED, 255-i);
    analogWrite(blueLED, i);
    delay(4);
  }

  delay(1000);

  // Red - Blue
  for(i=0; i<255; i++) {
    analogWrite(redLED, i);
    analogWrite(greenLED, 255-i);
    delay(4);
  }

  delay(1000);

  // Red - Green - Blue
  for(i=0; i<255; i++) {
    analogWrite(greenLED, i);
    delay(4);
  }

    delay(1000);

    // White
  for(i=0; i<255; i++) {
    analogWrite(blueLED, 255-i);
    analogWrite(greenLED, 255-i);
    analogWrite(redLED, 255-i);
    analogWrite(whiteLED, i);
    delay(4);
  }

  delay(1000);

  for(i=0; i<255; i++) {
    analogWrite(greenLED, i);
    analogWrite(redLED, i);
    analogWrite(blueLED, i);
    delay(4);
  }

  delay(5000);
}

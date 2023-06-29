//You may ignore the serial prints throughout the program. These were used for testing.
// Made by Marelle Vink


#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
//#include "vars.h"

#define measPin A0 //probably analog pin, so assuming values from 0 to 1023
#define buzzerPin 14
#define powerLostPin 6 


double scaleFactor = 2.6; // To scale from 13V (maximum of charging point) to 5V, because of the 
                          // voltage divider between the CP wire and the Arduino IO pin to use its maximum range of 5V. 


// Thresholds for the states. All in volts scaled to 5V, assumed from the picture. (Blue lines)
const float MIN_STATE_A = 11.0 / scaleFactor;
const float MAX_STATE_A = 13.0 / scaleFactor; 
const float MIN_STATE_B = 8.0 / scaleFactor;
const float MAX_STATE_B = 10.0 / scaleFactor;
const float MIN_STATE_C = 5.0 / scaleFactor;
const float MAX_STATE_C = 7.0 / scaleFactor;

enum stateChargepoint{
  STATE_A,
  STATE_B,
  STATE_C,
  STATE_D //may be ignored
};

float convertToVoltage(int valuePin);
void handleValue(float voltage);
void changeState(stateChargepoint newState);
void activateBuzzer();


float voltage = 0;
// float maxDemoVoltage = 13.0; //Top of highest blue line in picture
 float maxIOvoltage = 5.0; // Because of the voltage divider between the CP wire and the Arduino IO pin 

unsigned long buzzerTime = 500; // 500ms
int valuePin = 0;


stateChargepoint currentState = STATE_D; // Do nothing initially, assigned later
stateChargepoint newState = STATE_D; // Do nothing initially, assigned later

void setup() {
  Serial.begin(9600); 
  //pinMode(LEDS, OUTPUT);
  //pinMode(measPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  //initNeoPixel();
  //ledsOff();
}

void loop() {
  valuePin = analogRead(measPin);
    Serial.println(valuePin);

  // voltage = convertToVoltage(valuePin); 
  // Serial.println(voltage);
  // handleValue(voltage);

  //  if (newState != currentState) {
  //   changeState(newState);
  //   currentState = newState;
  // }
  delay(3000);
}

float convertToVoltage(int valuePin){
  voltage = maxIOvoltage / 1023.0 * valuePin;
  Serial.print("Voltage calculated: ");
  return voltage;
}

void handleValue(float value){
  
  if (value >= MIN_STATE_A && value <= MAX_STATE_A){
   newState = STATE_A;
   Serial.println("Changing to state A");
  }
  else if (value >= MIN_STATE_B && value <= MAX_STATE_B){
   newState = STATE_B;
   Serial.println("Changing to state B");
  }
  else if (value >= MIN_STATE_C && value <= MAX_STATE_C){
    newState = STATE_C;
    Serial.println("Changing to state C");
  }
  else {
    //do nothing
    Serial.println("No change of state");
  }
}

void changeState(stateChargepoint newState) {
  switch (newState) {
    case STATE_A:
      // setLed(101);
      // setLed(200);
      // setLed(300);
      break;
    case STATE_B:
      // setLed(102);
      // setLed(200);
      // setLed(300);
      break;
    case STATE_C:
      // setLed(104);
      // setLed(200);
      // setLed(300);
      break;
    case STATE_D:
      // May be ignored
      break;
  }
  Serial.println("State changed, leds activated");
  //updateLeds(); // Set leds as described
  activateBuzzer(); // Mention a change state
}

void activateBuzzer() {
  digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  Serial.println("Buzzer activated");
  unsigned long startTime = millis(); //Keep track of buzzer on-time

  if (millis() - startTime >= buzzerTime){
  digitalWrite(buzzerPin, LOW); // Turn off buzzer after set time
  Serial.println("Buzzer deactivated");
  startTime = 0; // Reset time for next change
  }
}
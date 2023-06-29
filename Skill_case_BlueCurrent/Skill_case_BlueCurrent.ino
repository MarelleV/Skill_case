// You may ignore the serial prints throughout the program. These were used for testing.
// Made by Marelle Vink

#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "vars.h"

#define measPin A1  //probably analog pin, so assuming values from 0 to 1023
#define buzzerPin 3
#define powerLostPin 6


double scaleFactor = 2.6;  // To scale from 13V (maximum of charging point) to 5V, because of the
                           // voltage divider between the CP wire and the Arduino IO pin to use its maximum range of 5V.

// Thresholds for the states. Min and max taken from given picture. (Blue lines).
const float MIN_STATE_A = 11.0 / scaleFactor;
const float MAX_STATE_A = 13.0 / scaleFactor;
const float MIN_STATE_B = 8.0 / scaleFactor;
const float MAX_STATE_B = 10.0 / scaleFactor;
const float MIN_STATE_C = 5.0 / scaleFactor;
const float MAX_STATE_C = 7.0 / scaleFactor;

enum stateChargepoint {
  STATE_A,
  STATE_B,
  STATE_C,
  STATE_D  //may be ignored
};

// Function prototypes
float convertToVoltage(int valuePin);
void handleValue(float voltage);
void changeState(stateChargepoint newState);
void activateBuzzer();
void powerLostInterrupt();


// Variables (also to avoid magic numbers)
float maxIOvoltage = 5.0;  // Because of the voltage divider between the CP wire and the Arduino IO pin
float topValueAnalogPin = 1023.0; // Highest value that the analog pin can receive
float voltage = 0;
int valuePin = 0;

// To keep track of states
stateChargepoint currentState = STATE_D;  // Do nothing initially, assigned later
stateChargepoint newState = STATE_D;      // Do nothing initially, assigned later

// For the interrupt on the powerLostPin
volatile bool powerLost = false;

// Simple interrupt to start measure after a given time
unsigned long startTime = 0;
unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 2000;  // Measurement interval in milliseconds

// For deactivating buzzer after given time
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerTime = 500;  // 500ms

// For deactivating leds after given time
bool ledsOn = false;
unsigned long startTimeLed = 0;
unsigned long ledOnTime = 3000; // 3 seconds


void setup() {
  Serial.begin(9600);
  //pinMode(LEDS, OUTPUT);
  pinMode(measPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  initNeoPixel();
  ledsOff();

  pinMode(powerLostPin, INPUT_PULLUP);  // Set pin 6 as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(powerLostPin), powerLostInterrupt, CHANGE); // Interrupt for power lost
}


void loop() {
  unsigned long currentTime = millis();
  if (powerLost) { // Is the power lost?
    ledsOff();
    powerLost = false;
  }
  if (currentTime - lastMeasurementTime >= measurementInterval) { //is it time for a new measurement?
    valuePin = analogRead(measPin);
    voltage = convertToVoltage(valuePin);
    handleValue(voltage);

    if (newState != currentState) { // is state changed?
      changeState(newState);
      currentState = newState;
    }

    lastMeasurementTime = currentTime;
  }

  unsigned long currentBuzzTime = millis();
  unsigned long currentLedTime = millis();

    // Check if the buzzer should be deactivated
  if (buzzerActive && buzzerStartTime > 0 && currentBuzzTime - buzzerStartTime >= buzzerTime) {
    digitalWrite(buzzerPin, LOW);  // Turn off the buzzer
    buzzerActive = false;
    buzzerStartTime = 0;
    Serial.println("Buzzer deactivated");
  }

   // Check if the LEDs should be deactivated
  if (ledsOn && startTimeLed > 0 && currentLedTime - startTimeLed >= ledOnTime) {
    ledsOff();
    ledsOn = false;
    startTimeLed = 0;
    Serial.println("LEDs deactivated");
  }
}

void powerLostInterrupt() {
  powerLost = true;
}

float convertToVoltage(int valuePin) {
  voltage = maxIOvoltage / topValueAnalogPin * valuePin;
  Serial.print("Voltage calculated: ");
  Serial.println(voltage);
  return voltage;
}

void handleValue(float value) {

  if (value >= MIN_STATE_A && value <= MAX_STATE_A) {
    newState = STATE_A;
    Serial.println("Changing to state A");
  } else if (value >= MIN_STATE_B && value <= MAX_STATE_B) {
    newState = STATE_B;
    Serial.println("Changing to state B");
  } else if (value >= MIN_STATE_C && value <= MAX_STATE_C) {
    newState = STATE_C;
    Serial.println("Changing to state C");
  } else {
    //do nothing
    Serial.println("No change of state or invalid value out of states");
  }
}

void changeState(stateChargepoint newState) {
    switch (newState) {
      case STATE_A:
        setLed(101);
        setLed(200);
        setLed(300);
        break;
      case STATE_B:
        setLed(102);
        setLed(200);
        setLed(300);
        break;
      case STATE_C:
        setLed(104);
        setLed(200);
        setLed(300);
        break;
      case STATE_D:
        // May be ignored
        break;
    }
    Serial.println("State changed, leds activated");
    updateLeds(); // Set leds as described
    activateBuzzer();  // Mention a change state
    ledsOn = true;
    startTimeLed = millis(); // Start counting the LED on-time
  }


void activateBuzzer() {
  if (!buzzerActive) {
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    buzzerActive = true;
    buzzerStartTime = millis();     // Start counting the buzzer on-time
    Serial.println("Buzzer activated");
  }
}
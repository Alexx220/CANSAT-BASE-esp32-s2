#include <Arduino.h>

// Define pin 15 as the LED pin
const int ledPin = 15;

void setup() {
  // Initialize pin 15 as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn the LED on (HIGH is the voltage level)
  digitalWrite(ledPin, HIGH);

  // Wait for one second (1000 milliseconds)
  delay(100);

  // Turn the LED off by making the voltage LOW
  digitalWrite(ledPin, LOW);

  // Wait for one second (1000 milliseconds)
  delay(100);
}
// blink shit 
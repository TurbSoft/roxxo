
#include <Arduino.h>

const int line_pin[5] = {2, 3, 4, 5, 6};

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set all line pins as input
  for (int i = 0; i < 5; i++) {
    pinMode(line_pin[i], INPUT);
  }
}

void loop() {
  // Check each line pin and print its state
  for (int i = 0; i < 5; i++) {
    int pinState = digitalRead(line_pin[i]);
    Serial.print("Pin ");
    Serial.print(line_pin[i]);
    Serial.print(": ");
    Serial.println(pinState == HIGH ? "HIGH" : "LOW");
  }

  // Add a delay to avoid flooding the serial monitor
  delay(100);
}

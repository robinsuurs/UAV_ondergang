#include <Arduino.h>

void setup() {
    pinMode(25, INPUT_PULLUP);
    pinMode(2, OUTPUT);
}

void loop() {
    if (digitalRead(25)) {
        digitalWrite(2, HIGH);
    } else {
        digitalWrite(2, LOW);
    }
}

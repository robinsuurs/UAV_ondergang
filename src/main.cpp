#include <Arduino.h>

void setup() {
    pinMode(25, INPUT_PULLUP);
    pinMode(26, OUTPUT);
}

void loop() {
    if (digitalRead(25)) {
        digitalWrite(26, HIGH);
    } else {
        digitalWrite(26, LOW);
    }
}

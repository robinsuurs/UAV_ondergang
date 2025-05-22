#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensorVoor, sensorLinksVoor, sensorLinksAchter;
#define XSHUT_VOOR 2
#define XSHUT_LINKS_VOOR 3
#define XSHUT_LINKS_ACHTER 4

// Declareer de array buiten de functies
uint16_t afstanden[3];

const int windowSize = 10;  // Pas dit aan voor meer/minder "heftig" middelen
uint16_t bufferVoor[windowSize], bufferLinksVoor[windowSize], bufferLinksAchter[windowSize];
int bufferIndex = 0;

uint16_t movingAverage_ToF(uint16_t newValue, uint16_t *buffer) {
    buffer[bufferIndex] = newValue; // Voeg nieuwe waarde toe aan buffer
    uint32_t sum = 0;

    for (int i = 0; i < windowSize; i++) {
        sum += buffer[i];
    }

    bufferIndex = (bufferIndex + 1) % windowSize; // Update index voor volgende meting
    return sum / windowSize;
}

void setup_ToF() {
    Serial.begin(9600);
    Wire.begin();

    // Initialiseer XSHUT-pinnen
    pinMode(XSHUT_VOOR, OUTPUT);
    pinMode(XSHUT_LINKS_VOOR, OUTPUT);
    pinMode(XSHUT_LINKS_ACHTER, OUTPUT);

    // Schakel alle sensoren uit
    digitalWrite(XSHUT_VOOR, LOW);
    digitalWrite(XSHUT_LINKS_VOOR, LOW);
    digitalWrite(XSHUT_LINKS_ACHTER, LOW);
    delay(10);

    // Initialiseer sensor vooraan
    digitalWrite(XSHUT_VOOR, HIGH);
    delay(10);
    sensorVoor.init();
    sensorVoor.setAddress(0x30);
    sensorVoor.startContinuous();

    // Initialiseer sensor linksvoor
    digitalWrite(XSHUT_LINKS_VOOR, HIGH);
    delay(10);
    sensorLinksVoor.init();
    sensorLinksVoor.setAddress(0x31);
    sensorLinksVoor.startContinuous();

    // Initialiseer sensor linksachter
    digitalWrite(XSHUT_LINKS_ACHTER, HIGH);
    delay(10);
    sensorLinksAchter.init();
    sensorLinksAchter.setAddress(0x32);
    sensorLinksAchter.startContinuous();
}

void readout_ToF() {
    // Lees ruwe waarden
    uint16_t ruweVoor = sensorVoor.readRangeContinuousMillimeters();
    uint16_t ruweLinksVoor = sensorLinksVoor.readRangeContinuousMillimeters();
    uint16_t ruweLinksAchter = sensorLinksAchter.readRangeContinuousMillimeters();

    // Bereken gemiddelden en sla op in de array
    afstanden[0] = movingAverage_ToF(ruweVoor, bufferVoor);
    afstanden[1] = movingAverage_ToF(ruweLinksVoor, bufferLinksVoor);
    afstanden[2] = movingAverage_ToF(ruweLinksAchter, bufferLinksAchter);
}

void test() {
//test ding van mij
}

void setup() {
    setup_ToF();
}

void loop() {
    readout_ToF();
}


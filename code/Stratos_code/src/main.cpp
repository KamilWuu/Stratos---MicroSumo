#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.printf("PSRAM is %s\n", psramInit() ? "enabled" : "not available");
}

void loop() {}
#include <WiFi.h>
#include <ArduinoOTA.h>

// Dane Wi-Fi
const char* ssid = "internet";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nPołączono z WiFi");

  // Inicjalizacja OTA
  ArduinoOTA.begin();
  Serial.println("OTA gotowe!");
}

void loop() {
  // Obsługa OTA
  ArduinoOTA.handle();

  // Twój kod (np. migająca dioda)
  static unsigned long prevMillis = 0;
  if (millis() - prevMillis > 1000) {
    prevMillis = millis();
    Serial.println("Działa :)");
  }
}


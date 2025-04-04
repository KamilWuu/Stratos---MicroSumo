#include <WiFi.h>
#include <ArduinoOTA.h>

#define LED_1 5
#define LED_2 2
#define LED_3 42

// Dane Wi-Fi
const char* ssid = "KoNaR_T3";
const char* password = "konarpany_down";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    //WiFi.begin(ssid, password);

  }
  Serial.println("\nPołączono z WiFi");

  // Inicjalizacja OTA
  ArduinoOTA.begin();
  Serial.println("OTA gotowe!");

  pinMode(LED_2,OUTPUT);

}
bool led_state = true;
void loop() {
  // Obsługa OTA
  ArduinoOTA.handle();
  
  static unsigned long prevMillis = 0;
  if (millis() - prevMillis > 1000) {
    prevMillis = millis();
    led_state=!led_state;
    digitalWrite(LED_2, led_state);
    Serial.println("Działa :)");
  }
}


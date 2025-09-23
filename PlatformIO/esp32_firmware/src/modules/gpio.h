#if USE_GPIO_MODULE

#include <Arduino.h>
#include <Preferences.h>

#define LED_PIN 2  // Confirm your board’s LED pin

Preferences prefs;

void gpioInit() {
  prefs.begin("gpio", false);
  bool savedState = prefs.getBool("state", false);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, savedState);
  prefs.end();
  Serial.printf("[GPIO] Initialized with state: %d\n", savedState);
}

void gpioWebHandler(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_POST) {
    prefs.begin("gpio", false);
    bool currentState = prefs.getBool("state", false);
    bool newState = !currentState;
    
    Serial.printf("[GPIO] Toggling from %d to %d\n", currentState, newState);
    digitalWrite(LED_PIN, newState);
    prefs.putBool("state", newState);
    prefs.end();

    request->send(200, "text/plain", newState ? "ON" : "OFF");
  } else {
    prefs.begin("gpio", false);
    bool currentState = prefs.getBool("state", false);
    prefs.end();
    request->send(200, "text/plain", currentState ? "ON" : "OFF");
  }
}

#endif
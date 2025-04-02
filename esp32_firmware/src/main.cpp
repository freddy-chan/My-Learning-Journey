#include <Arduino.h>
#include "config.h"
#include "core/wifi.h"
#include "core/web.h"
#include "core/mqtt.h"
#include "modules/gpio.h"
#include "modules/dht22.h"

void setup()
{
  Serial.begin(115200);
#if USE_WIFI
  initWiFi();
#endif
#if USE_WEB
  initWebServer();
#endif
#if USE_MQTT
  initMQTT();
#endif
#if USE_GPIO_MODULE
  gpioInit();
  server.on("/gpio", HTTP_GET | HTTP_POST, gpioWebHandler);
#endif
#if USE_DHT_MODULE
  dhtInit();
  server.on("/dht", HTTP_GET, dhtWebHandler);
#endif
}



void loop()
{
#if USE_MQTT
  mqttloop();
#endif
#if USE_DHT_MODULE
static unsigned long lastDhtRead = 0;
if (millis() - lastDhtRead > 2000) {  // Read every 2 seconds
  dhtUpdate();
  lastDhtRead = millis();
}
#endif
  delay(10);
}
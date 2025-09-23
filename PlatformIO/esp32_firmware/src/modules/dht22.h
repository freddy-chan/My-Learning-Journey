#if USE_DHT_MODULE
#include <DHT.h>
#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
float lastTemp = 0;
float lastHum = 0;

void dhtInit(){
    dht.begin();
}
void dhtUpdate(){
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    Serial.print("Temperature:");
    Serial.println(temp);
    Serial.print("Humidity :");
    Serial.println(hum);

    if(!isnan(temp) && !isnan(hum)){
        lastTemp = temp;
        lastHum = hum;
        #if USE_MQTT
        mqttClient.publish("stat/dht/temperature", String(temp).c_str());
        mqttClient.publish("stat/dht/humidity", String(hum).c_str());
        #endif
    }
}
void dhtWebHandler(AsyncWebServerRequest *request) {
    String json = "{\"temperature\":" + String(lastTemp) + ",\"humidity\":" + String(lastHum) + "}";
    request->send(200, "application/json", json);
  }
#endif
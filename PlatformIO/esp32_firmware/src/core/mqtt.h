#include <PubSubClient.h>
#include <WiFi.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void mqttCallback(char* topic,byte* payload, unsigned int length){
    // Handle incoming MQTT messages
    Serial.printf("MQTT: %s\n",topic);
}
void initMQTT(){
    #if USE_MQTT
        mqttClient.setServer("temperature.intelimyanmar.com",1883);
        mqttClient.setCallback(mqttCallback);
    #endif
}
void mqttloop(){
    #if USE_MQTT
    if(!mqttClient.connected()){
        mqttClient.connect("ESP32Client");
        mqttClient.subscribe("cmnd/+/+");
    }
    mqttClient.loop();
    #endif
}
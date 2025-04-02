#include "SensorManager.h"

SensorManager::SensorManager():
    dht(DHT_PIN,DHT_TYPE),
    temperature(NAN),
    humidity(NAN){
        xMutex = xSemaphoreCreateMutex();
    }

SensorManager& SensorManager::getInstance(){
    if(!instance){
        instance = new SensorManager();
    }
    return *instance;
}
void SensorManager::begin(){
    dht.begin();
}
void SensorManager::readData(){
    xSemaphoreTake(xMutex,portMAX_DELAY);
    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();
    if(!isnan(newTemp)) temperature = newTemp;
    if(!isnan(newHum)) humidity = newHum;
    xSemaphoreGive(xMutex);
    
}
float SensorManager::getHumidity(){
    xSemaphoreTake(xMutex,portMAX_DELAY);
    float hum = humidity;
    xSemaphoreGive(xMutex);
    return hum;
}
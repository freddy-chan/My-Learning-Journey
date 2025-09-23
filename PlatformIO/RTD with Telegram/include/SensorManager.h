#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


class SensorManager{
    public: 
        static SensorManager& getInstance();
        void begin();
        void readData();
        float getTemperature();
        float getHumidity();
    private:
        SensorManager();
        static SensorManager* instance;
        static const uint8_t DHT_PIN= 4; //GPIO 4
        static const int DHT_TYPE= DHT22;

        DHT dht;
        float temperature;
        float humidity;
        SemaphoreHandle_t xMutex;
};


#endif
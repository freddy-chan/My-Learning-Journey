#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SensorManager.h"

#define WIFI_SSID "abc"
#define WIFI_PASS "1234567890"
#define BOT_TOKEN "7673206145:AAFlfgbXheH7F1ZJy1xstqbyYUNaGb7SYEI"
#define CHAT_ID "4788440420"


// Global instances
UniversalTelegramBot bot(BOT_TOKEN, WiFiClientSecure{});
SensorManager& sensorManager = SensorManager::getInstance();

void telegramTask(void *pvParams) {
    while(1) {
        int numMessages = bot.getUpdates(bot.last_message_received + 1);
        
        while(numMessages > 0) {
            String text = bot.messages[0].text;
            if(text == "/data") {
                String message = "Environment Data:\n";
                message += "Temperature: " + String(sensorManager.getTemperature()) + "°C\n";
                message += "Humidity: " + String(sensorManager.getHumidity()) + "%";
                bot.sendMessage(CHAT_ID, message, "");
            }
            numMessages = bot.getUpdates(bot.last_message_received + 1);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensorTask(void *pvParams) {
    sensorManager.begin();
    while(1) {
        sensorManager.readData();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // DHT22 max 0.5Hz read rate
    }
}

void setup() {
    Serial.begin(115200);
    
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    // Create tasks
    xTaskCreate(
        telegramTask,
        "Telegram",
        10000,
        nullptr,
        1,
        nullptr
    );

    xTaskCreate(
        sensorTask,
        "Sensor",
        4096,
        nullptr,
        2,
        nullptr
    );

    vTaskDelete(nullptr);  // Delete setup task
}

void loop() {
    // Empty as FreeRTOS handles tasks
}
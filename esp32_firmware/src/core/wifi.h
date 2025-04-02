#include <WiFi.h>
void initWiFi(){
    #if USE_WIFI
        const char* ssid = "abc";
        const char* password = "1234567890";
        WiFi.begin(ssid,password);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
            /* code */

        }
        Serial.println("\nWiFi connected . IP: " + WiFi.localIP().toString());
    #endif
        
}
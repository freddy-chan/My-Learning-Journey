#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

AsyncWebServer server(80);
void initWebServer(){
    #if USE_WEB
        if(!SPIFFS.begin(true)){
            Serial.println("SPIFFS failed!");
            return;
        }
        server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){
            request->send(SPIFFS,"/index.html","text/html");
        });
        server.begin();
        Serial.println("Web server started");
    #endif
}
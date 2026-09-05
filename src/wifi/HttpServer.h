#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>

// Configuration web server used in WiFi AP mode. Serves a small form to capture
// the station SSID + password and persists them to system preferences.
class HttpServer
{
public:
    HttpServer(Preferences *systemPrefs);
    void begin();
    void handleClient();

private:
    WebServer server;
    Preferences *_systemPrefs;

    void handleRoot();
    void handleSave();
};

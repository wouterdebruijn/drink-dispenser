#pragma once
#include <Arduino.h>
#include <WiFi.h>

class WifiClient
{
public:
    WifiClient();
    void begin(char *ssid, char *password);

private:
};
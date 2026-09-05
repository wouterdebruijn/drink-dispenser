#pragma once
#include <Arduino.h>
#include <WiFi.h>

class WifiClient
{
public:
    WifiClient();

    // Start a station (STA) connection to the given network. Non-blocking: this
    // kicks off the association and returns immediately. Poll connected() to see
    // when the link is up.
    void beginStation(const char *ssid, const char *password);

    // Start a SoftAP for configuration. Clients can join this network to reach
    // the config HTTP server (default gateway 192.168.4.1).
    void beginAccessPoint(const char *apSsid, const char *apPassword);

    // True once the station link is associated.
    bool connected();

private:
};

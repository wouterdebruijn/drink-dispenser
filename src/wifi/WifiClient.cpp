#include "WifiClient.h"

WifiClient::WifiClient()
{
    // Constructor implementation (if needed)
}

void WifiClient::beginStation(const char *ssid, const char *password)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

#ifdef DEBUG
    Serial.print("Connecting to WiFi SSID: ");
    Serial.println(ssid);
#endif
}

void WifiClient::beginAccessPoint(const char *apSsid, const char *apPassword)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, apPassword);

#ifdef DEBUG
    Serial.print("Started SoftAP: ");
    Serial.println(apSsid);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
#endif
}

bool WifiClient::connected()
{
    return WiFi.status() == WL_CONNECTED;
}

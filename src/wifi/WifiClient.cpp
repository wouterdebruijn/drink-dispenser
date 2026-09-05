#include "WifiClient.h"

WifiClient::WifiClient()
{
    // Constructor implementation (if needed)
}

void WifiClient::begin(char *ssid, char *password)
{
    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    WiFi.begin(ssid, password);

#ifdef DEBUG
    Serial.print("Connecting to WiFi");
#endif

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
#ifdef DEBUG
        Serial.print(".");
#endif
        delay(500);
    }
}
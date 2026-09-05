#include "WifiReporter.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include "../lora/secrets.h"

// Static reporting endpoint. Tag counts are POSTed here as the raw binary
// payload (same layout as the LoRaWAN uplink).
static const char *SLURP_ENDPOINT = "https://slurp.hedium.nl/api/device";

// Reporting cadence, matching the LoRaWAN post-join interval (5 minutes).
static const unsigned long REPORT_INTERVAL_MS = 300UL * 1000UL;

WifiReporter::WifiReporter(RfidStorage *storage)
    : _storage(storage)
{
}

void WifiReporter::begin()
{
    _lastReportMs = millis();
    _firstReportDone = false;
}

void WifiReporter::loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    unsigned long now = millis();

    // Report once shortly after the link comes up, then on the interval.
    if (_firstReportDone && (now - _lastReportMs) < REPORT_INTERVAL_MS)
    {
        return;
    }

    _lastReportMs = now;
    _firstReportDone = true;

    report();
}

void WifiReporter::report()
{
    // Persist tag counts before sending, mirroring the LoRaWAN do_send path.
    _storage->storeTagData();

    static uint8_t buffer[RFID_MAX_TAGS * 4] = {0};
    uint8_t length = _storage->dumpTagStorage(buffer);

    if (length == 0)
    {
        Serial.println(F("WifiReporter: no data to send"));
        return;
    }

    WiFiClientSecure secureClient;
    // No certificate pinning yet; parity with the current setup. Hardening TODO.
    secureClient.setInsecure();

    HTTPClient http;
    if (!http.begin(secureClient, SLURP_ENDPOINT))
    {
        Serial.println(F("WifiReporter: http.begin failed"));
        return;
    }

    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("Authorization", "Bearer " SLURP_API_SECRET);

    int status = http.POST(buffer, length);
    Serial.printf("WifiReporter: POST %d bytes -> HTTP %d\n", length, status);

    if (status >= 200 && status < 300)
    {
        // Acknowledged; advance the send bookkeeping just like EV_TXCOMPLETE.
        _storage->clearChangedTags();
    }

    http.end();
}

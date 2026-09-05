#pragma once
#include <Arduino.h>

#include "../rfid/RfidStorage.h"

// Reports tag counts over HTTPS while in WiFi (STA) mode. Mirrors the LoRaWAN
// uplink: it sends the same binary payload produced by RfidStorage on a timed
// interval and clears the changed-tag flags once the backend acknowledges.
class WifiReporter
{
public:
    WifiReporter(RfidStorage *storage);

    void begin();

    // Call frequently from the main loop. Sends on the reporting interval when
    // the WiFi link is up and there is changed tag data to report.
    void loop();

private:
    void report();

    RfidStorage *_storage;
    unsigned long _lastReportMs = 0;
    bool _firstReportDone = false;
};

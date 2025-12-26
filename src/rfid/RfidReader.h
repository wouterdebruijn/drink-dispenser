#pragma once
#include <Arduino.h>
#include <RF_Commands.h>
#include "RfidStorage.h"

#define NO_TAG_LOCKOUT_THRESHOLD 30
#define DEBUG_RFID

typedef void (*PumpOnEnable)();

class RfidReader
{
public:
    RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage *storage, PumpOnEnable pumpEnableCallback);
    void begin();
    void loop();
    void parseSerial();
    void disableLockout();
    uint16_t getLastTagId() const { return lastTagId; }
    uint16_t getLastTagCount() const { return lastTagCount; }

private:
    uint16_t lastTagId = 0;    // Store the last tag ID to avoid duplicates
    uint16_t lastTagCount = 0; // Store the last tag count to avoid duplicates
    HardwareSerial *serial;
    RFC_Class rfc;
    RfidStorage *storage;
    uint8_t enablePin;
    PumpOnEnable pumpEnable;
    uint8_t noTagCount = 0;
    uint8_t isReading = 0;
    void handleTag(const Inventory_t &label);
};
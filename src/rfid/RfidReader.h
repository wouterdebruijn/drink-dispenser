#pragma once
#include <Arduino.h>
#include <RF_Commands.h>
#include "RfidStorage.h"

class RfidReader
{
public:
    RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage *storage);
    void begin();
    void loop();
    void parseSerial();
    String displayLine();

private:
    uint16_t lastTagId = 0;    // Store the last tag ID to avoid duplicates
    uint16_t lastTagCount = 0; // Store the last tag count to avoid duplicates
    HardwareSerial *serial;
    RFC_Class rfc;
    RfidStorage *storage;
    uint8_t enablePin;
    void handleTag(const Inventory_t &label);
};
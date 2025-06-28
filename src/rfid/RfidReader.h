#pragma once
#include <Arduino.h>
#include <RF_Commands.h>
#include "RfidStorage.h"

class RfidReader
{
public:
    RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage storage);
    void begin();
    void loop();

private:
    HardwareSerial *serial;
    RFC_Class rfc;
    RfidStorage &storage;
    uint8_t enablePin;
    void handleTag(const Inventory_t &label);
};
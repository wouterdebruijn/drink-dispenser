#pragma once
#include <Arduino.h>
#include <RF_Commands.h>
#include "RfidStorage.h"

#define NO_TAG_LOCKOUT_THRESHOLD 30
// Amount of times a tag needs to be seen before the pump is triggered.
#define TAG_CONFIRM_COUNT 3
#define RFID_TX_POWER 0x05DC // 0x05DC = 1500 = 15.00 dBm

#define RFID_MIXER_GAIN 0x02         // mixer gain index (higher = more gain)
#define RFID_IF_GAIN 0x06            // IF amplifier gain index (0x07 = max)
#define RFID_SIGNAL_THRESHOLD 0x00A0 // detection threshold (lower = more sensitive)
// #define DEBUG_RFID

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
    bool isConfirmingTag() const { return pendingCount > 0; }

private:
    uint16_t lastTagId = 0;    // Store the last tag ID to avoid duplicates
    uint16_t lastTagCount = 0; // Store the last tag count to avoid duplicates
    uint16_t pendingTagId = 0; // Candidate tag currently accumulating confirmations
    uint8_t pendingCount = 0;  // Consecutive successful scans of pendingTagId so far
    HardwareSerial *serial;
    RFC_Class rfc;
    RfidStorage *storage;
    uint8_t enablePin;
    PumpOnEnable pumpEnable;
    uint8_t noTagCount = 0;
    uint8_t isReading = 0;
    void handleTag(const Inventory_t &label);
};
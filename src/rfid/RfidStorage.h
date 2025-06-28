#pragma once

#include <Arduino.h>
#include <WString.h>

#define RFID_ID_LENGTH 12
#define RFID_MAX_TAGS 32

class RfidStorage
{
public:
    RfidStorage() = default;
    uint16_t incrementTagCount(const String &tagId, uint16_t increment);
    String dumpTagStorage();
    void clearChangedTags();

private:
    String tagIdArray[RFID_MAX_TAGS] = {""};     // Array to store tag IDs
    uint16_t tagCountArray[RFID_MAX_TAGS] = {0}; // Array to store tag counts
    bool changedTagIdMask[RFID_MAX_TAGS] = {false};
};

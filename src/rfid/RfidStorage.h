#pragma once

#include <Arduino.h>
#include <WString.h>

#define RFID_ID_LENGTH 12
#define RFID_MAX_TAGS 32

class RfidStorage
{
public:
    RfidStorage() = default;
    uint16_t incrementTagCount(uint16_t tagId, uint16_t increment);
    uint8_t dumpTagStorage(uint8_t *buffer);
    void clearChangedTags();
    void debugPrint();

private:
    uint16_t tagIdArray[RFID_MAX_TAGS] = {0};        // Array to store tag IDs
    uint16_t tagCountArray[RFID_MAX_TAGS] = {0};     // Array to store tag counts
    uint16_t tagSendCountArray[RFID_MAX_TAGS] = {0}; // Array to store sent tag counts
    bool tagIsInPayload[RFID_MAX_TAGS] = {false};    // Array to track if tag is in current payload
};

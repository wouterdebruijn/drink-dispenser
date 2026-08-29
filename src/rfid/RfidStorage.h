#pragma once

#include <Arduino.h>
#include <WString.h>
#include <Preferences.h>

#define RFID_ID_LENGTH 12
#define RFID_MAX_TAGS 32

class RfidStorage
{
public:
    RfidStorage(Preferences &prefs) : preferences(prefs) {}
    uint16_t incrementTagCount(uint16_t tagId, uint16_t increment);
    uint8_t dumpTagStorage(uint8_t *buffer);
    void begin();
    void clearChangedTags();
    void debugPrint();
    void storeTagData();
    void clearTagData();

private:
    Preferences &preferences;
    uint16_t tagIdArray[RFID_MAX_TAGS] = {0};           // Array to store tag IDs
    uint16_t tagCountArray[RFID_MAX_TAGS] = {0};        // Array to store tag counts
    uint16_t tagIdStoredArray[RFID_MAX_TAGS] = {0};     // Mirror of tag IDs currently persisted in flash
    uint16_t tagCountStoredArray[RFID_MAX_TAGS] = {0};  // Mirror of tag counts currently persisted in flash
    uint8_t tagSendRemainingArray[RFID_MAX_TAGS] = {0}; // Array keeping track of remaining sends per tag, we send each tag 3 times
    bool tagIsInPayload[RFID_MAX_TAGS] = {false};       // Array to track if tag is in current payload
};

#include "RfidStorage.h"

#include <Arduino.h>
#include <WString.h>

void RfidStorage::begin()
{
    // Load tag IDs and counts from preferences
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        String tagIdKey = "tagId_" + String(i);
        String tagCountKey = "tagCount_" + String(i);

        this->tagIdArray[i] = preferences.getUInt(tagIdKey.c_str(), 0);
        this->tagCountArray[i] = preferences.getUInt(tagCountKey.c_str(), 0);
        this->tagIdStoredArray[i] = this->tagIdArray[i];       // Track what is persisted in flash
        this->tagCountStoredArray[i] = this->tagCountArray[i]; // Track what is persisted in flash
        this->tagSendRemainingArray[i] = 3; // Initialize remaining sends to 3
        this->tagIsInPayload[i] = false;    // Initialize payload tracking to false
    }
}

void RfidStorage::debugPrint()
{
    Serial.println("RFID Tag Ids:");
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        Serial.printf("Tag %d: ID: %04X, Count: %d\n", i, tagIdArray[i], tagCountArray[i]);
    }
}

uint16_t RfidStorage::incrementTagCount(uint16_t tagId, uint16_t increment)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIdArray[i] == tagId)
        {
            this->tagCountArray[i] += increment; // Increment the count for the existing tag
            this->tagSendRemainingArray[i] = 3;  // We got a new value, reset the remaining sends

            Serial.printf("Tag ID %04X incremented by %d, new count: %d\n", tagId, increment, this->tagCountArray[i]);

            return this->tagCountArray[i];
        }
    }

    // If the tag is not found, add it to the first empty slot
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIdArray[i] == 0)
        {
            tagIdArray[i] = tagId;
            this->tagCountArray[i] = increment; // Set the count for the new tag
            this->tagSendRemainingArray[i] = 3; // Initialize the remaining sends for the new tag

            Serial.printf("New Tag ID %04X added with count: %d\n", tagId, this->tagCountArray[i]);

            return this->tagCountArray[i];
        }
    }
    Serial.println("No space available in tag storage.");
    return 0; // Return 0 if no space available
}

/* Clear tag data resetting all values to their initial state, call storageTagData to save the changes */
void RfidStorage::clearTagData()
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        this->tagIdArray[i] = 0;
        this->tagCountArray[i] = 0;
        this->tagSendRemainingArray[i] = 3; // Reset remaining sends to 3
        this->tagIsInPayload[i] = false;    // Reset payload tracking to false
    }
}

/* Store tag data in preferences, only writing slots whose values changed to limit costly flash operations */
void RfidStorage::storeTagData()
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIdArray[i] != this->tagIdStoredArray[i])
        {
            String tagIdKey = "tagId_" + String(i);
            preferences.putUInt(tagIdKey.c_str(), this->tagIdArray[i]);
            this->tagIdStoredArray[i] = this->tagIdArray[i];
        }

        if (this->tagCountArray[i] != this->tagCountStoredArray[i])
        {
            String tagCountKey = "tagCount_" + String(i);
            preferences.putUInt(tagCountKey.c_str(), this->tagCountArray[i]);
            this->tagCountStoredArray[i] = this->tagCountArray[i];
        }
    }
}

uint8_t RfidStorage::dumpTagStorage(uint8_t *buffer)
{
    int bufferIndex = 0;

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagSendRemainingArray[i] == 0) // Only dump changed tags
        {
            continue; // Skip unchanged tags
        }

        uint16_t tagId = this->tagIdArray[i];
        uint16_t value = this->tagCountArray[i];

        this->tagIsInPayload[i] = true;

        // Send as two uint16_t values using big-endian format
        buffer[bufferIndex++] = (tagId >> 8) & 0xFF; // Store high byte
        buffer[bufferIndex++] = tagId & 0xFF;        // Store low byte
        buffer[bufferIndex++] = (value >> 8) & 0xFF; // Store high byte of count
        buffer[bufferIndex++] = value & 0xFF;        // Store low byte of count
    }

    // End of buffer
    return bufferIndex; // Return the number of bytes written to the buffer
}

void RfidStorage::clearChangedTags()
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIsInPayload[i])
        {
            this->tagIsInPayload[i] = false;

            if (this->tagSendRemainingArray[i] > 0)
            {
                this->tagSendRemainingArray[i]--;
            }
        }
    }
}
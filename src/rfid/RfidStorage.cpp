#include "RfidStorage.h"

#include <Arduino.h>
#include <WString.h>

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
            this->tagSendCountArray[i] = 0;     // Initialize the sent count for the new tag
            return this->tagCountArray[i];
        }
    }
    Serial.println("No space available in tag storage.");
    return 0; // Return 0 if no space available
}

uint8_t RfidStorage::dumpTagStorage(uint8_t *buffer)
{
    int bufferIndex = 0;

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagSendCountArray[i] == this->tagCountArray[i]) // Only dump changed tags
        {
            continue; // Skip unchanged tags
        }

        uint16_t tagId = this->tagIdArray[i];
        uint16_t difference = this->tagCountArray[i] - this->tagSendCountArray[i];

        this->tagIsInPayload[i] = true;

        // Send as two uint16_t values using big-endian format
        buffer[bufferIndex++] = (tagId >> 8) & 0xFF;      // Store high byte
        buffer[bufferIndex++] = tagId & 0xFF;             // Store low byte
        buffer[bufferIndex++] = (difference >> 8) & 0xFF; // Store high byte of count
        buffer[bufferIndex++] = difference & 0xFF;        // Store low byte of count
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
            this->tagSendCountArray[i] = this->tagCountArray[i];
        }
    }
}
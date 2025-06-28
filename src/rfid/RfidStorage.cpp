#include "RfidStorage.h"

#include <Arduino.h>
#include <WString.h>

uint16_t RfidStorage::incrementTagCount(const String &tagId, uint16_t increment)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIdArray[i] == tagId)
        {
            this->changedTagIdMask[i] += increment; // Increment the count for the existing tag
            return changedTagIdMask[i];
        }
    }

    // If the tag is not found, add it to the first empty slot
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (this->tagIdArray[i].isEmpty())
        {
            tagIdArray[i] = tagId;
            this->changedTagIdMask[i] = increment; // Set the count for the new tag
            return changedTagIdMask[i];
        }
    }
    return 0; // Return 0 if no space available
}

String RfidStorage::dumpTagStorage()
{
    String json = "[";
    bool first = true;

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if (!this->changedTagIdMask[i])
        {
            if (!first)
            {
                json += ",";
            }
            json += "{\"id\":\"" + this->tagIdArray[i] + "\",\"count\":" + String(this->tagCountArray[i]) + "}";
            first = false;
        }
    }

    json += "]";
    return json;
}

void RfidStorage::clearChangedTags()
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        this->changedTagIdMask[i] = false; // Reset the changed mask
    }
}
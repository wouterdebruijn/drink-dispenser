#include "RfidReader.h"

RfidReader::RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage *storage, PumpOnEnable pumpEnableCallback)
    : serial(serial), rfc(serial), enablePin(enablePin), storage(storage), pumpEnable(pumpEnableCallback)
{
}

void RfidReader::begin()
{
    // Reset the RFID reader by toggling the enable pin.
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, 0);

    delay(100);
    digitalWrite(enablePin, 1);

    // Wait for the RFID reader to power up.
    delay(500);

    // Initialize the serial communication and the RFC class.
    serial->begin(115200, SERIAL_8N1, 2, 13);
    rfc.begin();

    delay(2000);

    // Set the serial timeout and FIFO full settings.
    Serial.println(rfc.SetRegionFrame(REGION_CODE_EUR) ? "SUCCESS" : "Fail");
    Serial.println(rfc.SetPaPowerFrame(0x00) ? "SUCCESS" : "Fail");
}

void RfidReader::parseSerial()
{
    while (serial->available())
        rfc.encode(serial->read());
}

void RfidReader::loop()
{
    // Multiread for 16 tags.
    rfc.SetGetLabelStart(0x0004);

    if (rfc.inventory.isValid() && rfc.inventory.isUpdated())
    {
        Inventory_t label = rfc.inventory.GetLabel();
        this->handleTag(label);
    }
    else
    {
        uint8_t errorCode = rfc.error.ErrorCode();

        if (errorCode == 0x15)
        {
            // Generic error, no tag found.

            if (lockout)
            {
                noTagCount++;
                if (noTagCount >= NO_TAG_LOCKOUT_THRESHOLD)
                {
                    // After NO_TAG_LOCKOUT_THRESHOLD consecutive no-tag reads, disable lockout.
                    lockout = false;
                    noTagCount = 0;
                }
            }

            return;
        }
        Serial.printf("Error Code: %X\n", errorCode);
    }
}

void RfidReader::handleTag(const Inventory_t &label)
{
    // Last 2 bytes of the EPC are used as the tag ID.
    uint16_t value = 0;

    value = (label.epc[10] << 8) | label.epc[11];
    if (lockout)
    {
        return;
    }

    uint16_t count = storage->incrementTagCount(value, 20); // TODO change to variable from Lora

    pumpEnable();

    lockout = true;
    lastTagId = value;
    lastTagCount = count;
}

String RfidReader::displayLine()
{
    if (lastTagId == 0)
    {
        return "Waiting for tag...";
    }

    String line = "Tag: ";
    line += String(lastTagId, HEX);
    line += " Count: ";
    line += String(lastTagCount);
    return line;
}

void RfidReader::disableLockout()
{
    lockout = false;
}
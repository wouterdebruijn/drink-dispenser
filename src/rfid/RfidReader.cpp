#include "RfidReader.h"

RfidReader::RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage *storage)
    : serial(serial), rfc(serial), enablePin(enablePin), storage(storage)
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

    if (value == lastTagId)
    {
        // TODO add reset
        return;
    }

    uint16_t count = storage->incrementTagCount(value, 20); // TODO change to variable from Lora

    lastTagCount = count;
    lastTagId = value;
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
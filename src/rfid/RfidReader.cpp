#include "RfidReader.h"

RfidReader::RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage storage)
    : serial(serial), rfc(serial), enablePin(enablePin), storage(storage)
{
    // Constructor initializes the RFID reader with the provided serial port and enable pin.
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
    Serial.println(rfc.SetPaPowerFrame(0x14) ? "SUCCESS" : "Fail");
    Serial.println(rfc.SetGetLabelStart(0xFFFF) ? "SUCCESS" : "Fail");
}

void RfidReader::loop()
{
    while (serial->available())
        rfc.encode(serial->read());

    if (rfc.inventory.isValid() && rfc.inventory.isUpdated())
    {
        Inventory_t label = rfc.inventory.GetLabel();
        this->handleTag(label);
    }
    else
    {
        uint8_t errorCode = rfc.error.ErrorCode();

        if (errorCode != 0x15)
        {
            Serial.print("RFID Error: ");
            Serial.println(errorCode, HEX);
            return;
        }
    }
}

void RfidReader::handleTag(const Inventory_t &label)
{
    String tagId = String((char *)label.epc, 12);
    Serial.print("RFID Tag ID: ");
    Serial.println(tagId);

// Increment the tag count by the given ML amount.]

// TODO: Implement logic to determine the ML amount based on the tag ID.
#define ML_AMOUNT 20 // Example amount, replace with actual logic

    uint16_t count = storage.incrementTagCount(tagId, ML_AMOUNT);

    Serial.print("Tag Count for ");
    Serial.print(tagId);
    Serial.print(": ");
    Serial.println(count);
}
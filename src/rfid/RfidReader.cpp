#include "RfidReader.h"

RfidReader::RfidReader(HardwareSerial *serial, uint8_t enablePin, RfidStorage *storage, PumpOnEnable pumpEnableCallback)
    : serial(serial), rfc(serial), enablePin(enablePin), storage(storage), pumpEnable(pumpEnableCallback)
{
}

void RfidReader::begin()
{
    uint8_t REGION_SET = 0;
    uint8_t POWER_SET = 0;
    uint8_t MODEM_SET = 0;

    // Reset the RFID reader by toggling the enable pin.
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, 0);

    delay(100);
    digitalWrite(enablePin, 1);

    // Wait for the RFID reader to power up.
    delay(500);

    // Initialize the serial communication and the RFC class.
    serial->begin(115200, SERIAL_8N1, 13, 2);
    rfc.begin();

    delay(2000);

    while (REGION_SET != 255)
    {
        bool success = rfc.SetRegionFrame(REGION_CODE_EUR);
        REGION_SET = success ? 255 : REGION_SET + 1;

        Serial.print("Setting region to EUR: ");
        Serial.println(success ? "SUCCESS" : "Fail");

        delay(500);
    }

    while (POWER_SET != 255)
    {
        bool success = rfc.SetPaPowerFrame(0x00B8); // 0x00B8 = 0dBm
        POWER_SET = success ? 255 : POWER_SET + 1;

        Serial.print("Setting power to 0x07D0 dBm: ");
        Serial.println(success ? "SUCCESS" : "Fail");

        delay(500);
    }

    while (MODEM_SET != 255)
    {
        bool success = rfc.SetDemodulatorParameterFrame({0x02, 0x06, 0x00A0});
        MODEM_SET = success ? 255 : MODEM_SET + 1;

        Serial.print("Setting modem parameters: ");
        Serial.println(success ? "SUCCESS" : "Fail");
        delay(500);
    }
}

void RfidReader::parseSerial()
{
    while (serial->available())
        rfc.encode(serial->read());
}

void RfidReader::loop()
{
    if (isReading == 0)
    {
        rfc.SetGetLabelOnce();
        isReading = 1;

        return;
    }
    else
    {
        isReading++;
    }

    if (!rfc.error.isValid() && rfc.inventory.isValid())
    {
        isReading = 0;

        Inventory_t label = rfc.inventory.GetLabel();
        this->handleTag(label);

#ifdef DEBUG_RFID
        Serial.print("EPC: ");
        for (int i = 0; i < 2; i++)
        {
            Serial.printf("%02X", label.epc[i]);
        }
        Serial.printf(" | RSSI: %d | PC: %04X | CRC: %04X\n", label.RSSI, label.PC, label.CRC);
#endif
    }
    else if (rfc.error.isValid())
    {
        isReading = 0;

        uint8_t errorCode = rfc.error.ErrorCode();

        if (errorCode == 0x15)
        {
#ifdef DEBUG_RFID
            Serial.println("No tag found.");
#endif
            if (lastTagId != 0)
            {
                noTagCount++;
                if (noTagCount >= NO_TAG_LOCKOUT_THRESHOLD)
                {
                    // After NO_TAG_LOCKOUT_THRESHOLD consecutive no-tag reads, disable lockout.
                    lastTagId = 0;
                    noTagCount = 0;
                }
            }
            return;
        }
        Serial.printf("Error Code: %X\n", errorCode);
    }
    else if (isReading > 10)
    {
        isReading = 0;
        Serial.println("Timeout waiting for inventory data.");
    }
}

void RfidReader::handleTag(const Inventory_t &label)
{
    // Last 2 bytes of the EPC are used as the tag ID.
    uint16_t value = 0;

    value = (label.epc[0] << 8) | label.epc[1];
    if (lastTagId == value)
    {
#ifdef DEBUG_RFID
        Serial.println("Duplicate tag read; ignoring.");
#endif
        return;
    }

    if (value > 0 && value < 20)
    {
        uint16_t count = storage->incrementTagCount(value, 20); // TODO change to variable from Lora

        pumpEnable();

        lastTagId = value;
        lastTagCount = count;
    }
    else if (value == 0xFFFF)
    {
        // Special tag only pump
        pumpEnable();
    }
}

void RfidReader::disableLockout()
{
    lastTagId = 0;
}

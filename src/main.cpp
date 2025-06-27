#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include <RF_Commands.h>
#include "display/Display.h"

HardwareSerial SerialRF(2);
RFC_Class rfc(&SerialRF);

Display display;

#define RFIDEN 14

uint8_t rfidLockout = 0; // Lockout for RFID reading

void setup()
{
  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC();

  pinMode(RFIDEN, OUTPUT);
  digitalWrite(RFIDEN, 0);

  delay(100);
  digitalWrite(RFIDEN, 1);
  delay(500);

  SerialRF.begin(115200, SERIAL_8N1, 2, 13);
  rfc.begin();

  display.begin();
  display.startupText();

  delay(2000);

  Serial.println(rfc.SetRegionFrame(REGION_CODE_EUR) ? "SUCCESS" : "Fail");
  Serial.println(rfc.SetPaPowerFrame(0x14) ? "SUCCESS" : "Fail");
  Serial.println(rfc.SetGetLabelStart(0x2710) ? "SUCCESS" : "Fail");
}

void rfidLoop()
{
}

void loop()
{
  loopLMIC();

  rfidLockout++;

  if (rfidLockout > 250)
  {

    while (SerialRF.available())
      rfc.encode(SerialRF.read());

    if (rfc.inventory.isValid() && rfc.inventory.isUpdated())
    {
      Inventory_t label = rfc.inventory.GetLabel();
      Serial.printf("RSSI : %d dB\n", (int8_t)label.RSSI);
      Serial.printf("PC : %02X\n", label.PC);
      Serial.print("EPC : ");

      for (int i = 0; i < 12; i++)
      {
        Serial.print(label.epc[i], HEX);
      }

      Serial.println();
      Serial.printf("CRC : %02X\n", label.CRC);
      display.clearBuffer();
      display.setCursor(0, 10);
      display.print("EPC : ");

      for (int i = 0; i < 12; i++)
      {
        display.print(label.epc[i], HEX);
      }

      display.setCursor(0, 20);
      display.print("PC : ");
      display.print(label.PC, HEX);
      display.setCursor(0, 30);
      display.print("RSSI : ");
      display.print((int8_t)label.RSSI);
      display.setCursor(0, 40);
      display.print("CRC : ");
      display.print(label.CRC, HEX);
      display.sendBuffer();
    }
    else
    {
      uint8_t errorCode = rfc.error.ErrorCode();

      if (errorCode != 0x15)
      {
        display.clearBuffer();
        display.setCursor(0, 10);
        display.print("Error Code: ");
        display.print(errorCode, HEX);
        display.sendBuffer();
      }
    }

    rfidLockout = 0;
  }
}

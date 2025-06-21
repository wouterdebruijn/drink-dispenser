/*
 * @Author: your name
 * @Date: 2021-10-19 17:54:59
 * @LastEditTime: 2021-10-29 10:45:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MagicRF M100\src\main.cpp
 */

#include <Arduino.h>
#include "RF_Commands.h"
#include "HardwareSerial.h"

#include <Wire.h>
#include <U8g2lib.h>

HardwareSerial SerialRF(2);
RFC_Class rfc(&SerialRF);

#define RFIDEN 14

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup()
{
    pinMode(RFIDEN, OUTPUT);
    digitalWrite(RFIDEN, 0);

    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.print("MagicRF M100");
    u8g2.sendBuffer();

    delay(100);
    digitalWrite(RFIDEN, 1);
    delay(500);

    SerialRF.begin(115200, SERIAL_8N1, 2, 13);
    Serial.begin(115200);
    rfc.begin();

    delay(2000);

    Serial.println(rfc.SetRegionFrame(REGION_CODE_EUR) ? "SUCCESS" : "Fail");
    Serial.println(rfc.SetPaPowerFrame(0x07D0) ? "SUCCESS" : "Fail");
    Serial.println(rfc.SetGetLabelStart(0x2710) ? "SUCCESS" : "Fail");
}

void loop()
{
    delay(200);

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

        u8g2.clearBuffer();
        u8g2.setCursor(0, 10);
        u8g2.print("EPC : ");
        for (int i = 0; i < 12; i++)
        {
            u8g2.print(label.epc[i], HEX);
        }
        u8g2.setCursor(0, 20);
        u8g2.print("PC : ");
        u8g2.print(label.PC, HEX);
        u8g2.setCursor(0, 30);
        u8g2.print("RSSI : ");
        u8g2.print((int8_t)label.RSSI);
        u8g2.setCursor(0, 40);
        u8g2.print("CRC : ");
        u8g2.print(label.CRC, HEX);
        u8g2.sendBuffer();
    }
    else
    {
        Serial.printf("Error Code: %X\n", rfc.error.ErrorCode());
    }
}
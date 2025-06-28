#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "display/Display.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

Display display;

RfidStorage rfidStorage;

#define RFID_ENABLE_PIN 14
HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, rfidStorage);

void setup()
{
  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC();

  rfidReader.begin();
  display.begin();
  display.startupText();

  delay(1000);
}

void loop()
{
  loopLMIC();
  // rfidReader.loop();
}

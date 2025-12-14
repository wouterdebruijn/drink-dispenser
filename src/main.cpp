#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

#include <TaskScheduler.h>

Scheduler ts;

RfidStorage rfidStorage;

#define RFID_ENABLE_PIN 14
HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, &rfidStorage);

void rfidLoop()
{
  rfidReader.loop();
}

void displayLoop()
{
  Serial.println(rfidReader.displayLine());
}

Task displayTask(1000 * TASK_MILLISECOND, TASK_FOREVER, &displayLoop, &ts, true);
Task rfidTask(100 * TASK_MILLISECOND, TASK_FOREVER, &rfidLoop, &ts, true);

void setup()
{
  setupBoards(true);
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC(&rfidStorage);

  rfidReader.begin();

  delay(1000);
}

bool rfidScheduled = false;

void loop()
{
  loopLMIC();
  rfidReader.parseSerial();
  ts.execute();
}

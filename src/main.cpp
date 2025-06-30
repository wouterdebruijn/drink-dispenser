#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "display/Display.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

#include <TaskScheduler.h>

Scheduler ts;

Display display;

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
  display.clear();
  display.setCursor(0, 10);
  display.print("Battery: ");
  display.print(PMU->getBatteryPercent());
  display.print("%");
  display.setCursor(0, 20);
  display.print(rfidReader.displayLine());
  display.sendBuffer();
}

Task displayTask(1000 * TASK_MILLISECOND, TASK_FOREVER, &displayLoop, &ts, true);
Task rfidTask(100 * TASK_MILLISECOND, TASK_FOREVER, &rfidLoop, &ts, true);

void setup()
{
  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC(&rfidStorage);

  rfidReader.begin();
  display.begin();
  display.startupText();

  delay(1000);
}

bool rfidScheduled = false;

void loop()
{
  loopLMIC();
  rfidReader.parseSerial();
  ts.execute();
}

#define _TASK_WDT_IDS           // To enable task unique IDs
#define _TASK_SLEEP_ON_IDLE_RUN // Compile with support for entering IDLE SLEEP state for 1 ms if not tasks are scheduled to run
#define _TASK_LTS_POINTER       // Compile with support for Local Task Storage pointer
#define _TASK_SELF_DESTRUCT     // Enable tasks to "self-destruct" after disable

#define RFID_ENABLE_PIN 14
#define PUMP_PIN 25

#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

#include "display/Display.h"

#include "peripherals/Pump.h"

#include <TaskScheduler.h>

int freeMemory() { return ESP.getFreeHeap(); }

Scheduler ts;

RfidStorage rfidStorage;

void enablePumpForDuration();

HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, &rfidStorage, &enablePumpForDuration);
Pump pump(PUMP_PIN);
Display display;

void pumpTimerCallback();
void pumpDisableCallback();
void displayLoop();

#define PUMP_STEP_COUNT 6
Task pumpOffTask(100 * TASK_MILLISECOND, PUMP_STEP_COUNT * 4, &pumpTimerCallback, &ts, false, NULL, &pumpDisableCallback);
Task displayTask(10000 * TASK_MILLISECOND, TASK_FOREVER, &displayLoop, &ts, true);

uint8_t pump_dispense_counter = 0;

void pumpDisableCallback()
{
  Serial.println("Pump disabled");
  pump.disablePump();
  pump_dispense_counter = 0;
}

void pumpTimerCallback()
{
  pump_dispense_counter++;
  displayLoop();
}

void enablePumpForDuration()
{
  Serial.println("Pump enabled");
  pump.enablePump();
  pumpOffTask.restartDelayed();
}

void rfidLoop()
{
  rfidReader.loop();
}

void displayLoop()
{
  display.clearBuffer();
  display.setFontMode(1);
  display.setBitmapMode(1);

  if (rfidReader.getLastTagId() != 0)
  {
    uint16_t tagId = rfidReader.getLastTagId();
    uint16_t tagCount = rfidReader.getLastTagCount();

    // Display tag ID as decimal
    char tagIdStr[20];
    sprintf(tagIdStr, "Cheers #%d (%d)", tagId, tagCount / 20);

    display.setFont(u8g2_font_6x12_tr);
    display.drawStr(10, 58, tagIdStr);
  }
  else
  {
    display.setFont(u8g2_font_6x12_tr);
    display.drawStr(19, 58, "Shot Machine");

    display.setFont(u8g2_font_5x8_tr);
    display.drawStr(95, 58, "v4");
  }

  // joinStatus
  if (joinStatus != EV_JOINED)
  {
    display.setFont(u8g2_font_4x6_tr);
    display.drawStr(1, 10, "Joining...");
  }
  else
  {
    display.setFont(u8g2_font_4x6_tr);
    display.drawStr(1, 10, "Connected");
  }

  if (pump_dispense_counter > 0)
  {
    switch (pump_dispense_counter / 6)
    {
    case 0:
      display.drawXBM(46, 7, 36, 39, shot_glass_frame_1);
      break;
    case 1:
      display.drawXBM(46, 7, 36, 39, shot_glass_frame_2);
      break;
    case 2:
      display.drawXBM(46, 7, 36, 39, shot_glass_frame_3);
      break;
    case 3:
      display.drawXBM(46, 7, 36, 39, shot_glass_frame_4);
      break;
    default:
      display.drawXBM(46, 7, 36, 39, shot_glass_frame_0);
      break;
    }
  }
  else
  {
    display.drawXBM(46, 7, 36, 39, shot_glass_frame_0);
  }

  display.sendBuffer();
}

Task rfidTask(250 * TASK_MILLISECOND, TASK_FOREVER, &rfidLoop, &ts, true);

void setup()
{
  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC(&rfidStorage);

  rfidReader.begin();
  pump.begin();
  display.begin();
  display.startupText();

  delay(1000);

  displayLoop();
}

bool rfidScheduled = false;

void loop()
{
  loopLMIC();
  rfidReader.parseSerial();
  ts.execute();
}

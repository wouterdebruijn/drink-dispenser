#define _TASK_WDT_IDS           // To enable task unique IDs
#define _TASK_SLEEP_ON_IDLE_RUN // Compile with support for entering IDLE SLEEP state for 1 ms if not tasks are scheduled to run
#define _TASK_LTS_POINTER       // Compile with support for Local Task Storage pointer
#define _TASK_SELF_DESTRUCT     // Enable tasks to "self-destruct" after disable

#define RFID_ENABLE_PIN 14
#define PUMP_PIN 21

#include <Arduino.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

#include "peripherals/Pump.h"

#include <TaskScheduler.h>

int freeMemory() { return ESP.getFreeHeap(); }

Scheduler ts;

RfidStorage rfidStorage;

void enablePumpForDuration();

HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, &rfidStorage, &enablePumpForDuration);
Pump pump(PUMP_PIN);

void pumpTimerCallback();

Task pumpOffTask(2000 * TASK_MILLISECOND, TASK_ONCE, &pumpTimerCallback, &ts, false);

void pumpTimerCallback()
{
  Serial.println("Pump disabled");
  pump.disablePump();
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
  pump.begin();

  delay(1000);
}

bool rfidScheduled = false;

void loop()
{
  loopLMIC();
  rfidReader.parseSerial();
  ts.execute();
}

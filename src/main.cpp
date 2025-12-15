#define _TASK_WDT_IDS           // To enable task unique IDs
#define _TASK_SLEEP_ON_IDLE_RUN // Compile with support for entering IDLE SLEEP state for 1 ms if not tasks are scheduled to run
#define _TASK_LTS_POINTER       // Compile with support for Local Task Storage pointer
#define _TASK_SELF_DESTRUCT     // Enable tasks to "self-destruct" after disable

#define RFID_ENABLE_PIN 14
#define PUMP_PIN 15

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
  loopPMU(NULL);

  display.clear();

  // Shot machine display
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 10);
  display.print("Shot Machine");
  display.setCursor(0, 30);
  display.print("Last Tag:");
  display.setCursor(0, 50);
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
  pump.begin();
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

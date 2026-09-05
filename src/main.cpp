#define _TASK_WDT_IDS           // To enable task unique IDs
#define _TASK_SLEEP_ON_IDLE_RUN // Compile with support for entering IDLE SLEEP state for 1 ms if not tasks are scheduled to run
#define _TASK_LTS_POINTER       // Compile with support for Local Task Storage pointer
#define _TASK_SELF_DESTRUCT     // Enable tasks to "self-destruct" after disable

#define RFID_ENABLE_PIN 14
#define PUMP_PIN 25
#define MENU_BUTTON_PIN 15

#include <Arduino.h>
#include <TaskScheduler.h>
#include <Preferences.h>

#include "lora/loramac.h"
#include "lora/LoRaBoards.h"

#include "rfid/RfidReader.h"
#include "rfid/RfidStorage.h"

#include "display/Display.h"
#include "display/Button.h"
#include "display/Menu.h"
#include "peripherals/Pump.h"

#include "wifi/WifiClient.h"
#include "wifi/HttpServer.h"

int freeMemory() { return ESP.getFreeHeap(); }
void enablePumpForDuration();

Scheduler ts;
Preferences systemPrefs;
Preferences rfidStoragePrefs;
RfidStorage rfidStorage(rfidStoragePrefs);

HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, &rfidStorage, &enablePumpForDuration);
Pump pump(PUMP_PIN);
Display display;
Button menuButton(MENU_BUTTON_PIN);
Menu menu(&menuButton, &display, &rfidReader, &rfidStorage, &systemPrefs, &enablePumpForDuration);

void pumpTimerCallback();
void pumpDisableCallback();
void displayLoop();
void rfidLoop();

#define PUMP_STEP_COUNT 6
Task pumpOffTask(100 * TASK_MILLISECOND, PUMP_STEP_COUNT * 4, &pumpTimerCallback, &ts, false, NULL, &pumpDisableCallback);
Task displayTask(10000 * TASK_MILLISECOND, TASK_FOREVER, &displayLoop, &ts, false);
Task rfidTask(250 * TASK_MILLISECOND, TASK_FOREVER, &rfidLoop, &ts, true);

uint8_t pump_dispense_counter = 0;

void pumpDisableCallback()
{
  Serial.println("Pump disabled");
  pump.disablePump();
  pump_dispense_counter = 0;
  menu.setPumpProgress(0);
  displayLoop();
}

void pumpTimerCallback()
{
  pump_dispense_counter++;
  menu.setPumpProgress(pump_dispense_counter);
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
  // Rendering now lives in the Menu class; this stays as the entry point used
  // by the periodic display task and the pump animation callbacks.
  menu.render();
}

void setup()
{
  systemPrefs.begin("system", false);

  bool loraEnabled = systemPrefs.getBool("loraEnabled", true);

  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);

  // if (loraEnabled)
  // {
  setupLMIC(&rfidStorage);
  // }

  rfidStoragePrefs.begin("rfid", false);
  rfidStorage.begin();

  rfidReader.begin();
  pump.begin();
  display.begin();
  display.startupText();

  menu.begin();

  delay(1000);

  displayLoop();
  displayTask.enable();
}

bool rfidScheduled = false;

void loop()
{
  loopLMIC();
  rfidReader.parseSerial();
  menu.loop();
  ts.execute();
}

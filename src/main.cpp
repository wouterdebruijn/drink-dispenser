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
#include "wifi/WifiReporter.h"

#define WIFI_AP_SSID "ShotMachine-Setup"
#define WIFI_AP_PASSWORD "shotmachine"

int freeMemory() { return ESP.getFreeHeap(); }
void enablePumpForDuration();

Scheduler ts;
Preferences systemPrefs;
Preferences rfidStoragePrefs;
RfidStorage rfidStorage(rfidStoragePrefs);

// The single connectivity mode resolved at boot from the persisted booleans.
// Only one is ever active; everything else follows from this value.
enum class ConnectivityMode : uint8_t
{
  Offline,
  Lora,
  Wifi,
  WifiAp,
};

ConnectivityMode connectivityMode = ConnectivityMode::Lora;

HardwareSerial SerialRF(2);
RfidReader rfidReader(&SerialRF, RFID_ENABLE_PIN, &rfidStorage, &enablePumpForDuration);
Pump pump(PUMP_PIN);
Display display;
Button menuButton(MENU_BUTTON_PIN);
Menu menu(&menuButton, &display, &rfidReader, &rfidStorage, &systemPrefs, &enablePumpForDuration);

WifiClient wifi;
HttpServer httpServer(&systemPrefs);
WifiReporter wifiReporter(&rfidStorage);

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

// Resolve the persisted mode booleans to a single active mode. Priority
// AP > WiFi > LoRa > Offline guards against any inconsistent stored state.
ConnectivityMode resolveConnectivityMode()
{
  if (systemPrefs.getBool("wifiApEnabled", false))
  {
    return ConnectivityMode::WifiAp;
  }
  if (systemPrefs.getBool("wifiEnabled", false))
  {
    return ConnectivityMode::Wifi;
  }
  if (systemPrefs.getBool("loraEnabled", true))
  {
    return ConnectivityMode::Lora;
  }
  return ConnectivityMode::Offline;
}

void setup()
{
  systemPrefs.begin("system", false);

  connectivityMode = resolveConnectivityMode();

  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);

  switch (connectivityMode)
  {
  case ConnectivityMode::Lora:
    setupLMIC(&rfidStorage);
    break;

  case ConnectivityMode::Wifi:
  {
    // Cut power to the LoRa modem; WiFi mode reports over HTTPS instead.
    setRadioPower(false);
    String ssid = systemPrefs.getString("wifiSsid", "");
    String password = systemPrefs.getString("wifiPassword", "");
    wifi.beginStation(ssid.c_str(), password.c_str());
    wifiReporter.begin();
    break;
  }

  case ConnectivityMode::WifiAp:
    // Configuration only: bring up the SoftAP + config web server, no radio.
    setRadioPower(false);
    wifi.beginAccessPoint(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    httpServer.begin();
    break;

  case ConnectivityMode::Offline:
  default:
    // No connectivity; leave the modem unpowered.
    setRadioPower(false);
    break;
  }

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
  switch (connectivityMode)
  {
  case ConnectivityMode::Lora:
    loopLMIC();
    break;

  case ConnectivityMode::Wifi:
    wifiReporter.loop();
    break;

  case ConnectivityMode::WifiAp:
    // Configuration only: serve the config page and keep the menu responsive
    // (so the user can switch modes and reboot). No dispensing/reporting.
    httpServer.handleClient();
    menu.loop();
    return;

  case ConnectivityMode::Offline:
  default:
    break;
  }

  rfidReader.parseSerial();
  menu.loop();
  ts.execute();
}

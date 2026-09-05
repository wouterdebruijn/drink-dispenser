#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "Button.h"
#include "Display.h"
#include "../rfid/RfidReader.h"
#include "../rfid/RfidStorage.h"

// Called when the menu should trigger a manual pump cycle (single press on the
// status screen), as an alternative to an RFID scan.
typedef void (*PumpTriggerCallback)();

class Menu
{
public:
    Menu(Button *button,
         Display *display,
         RfidReader *rfidReader,
         RfidStorage *rfidStorage,
         Preferences *systemPrefs,
         PumpTriggerCallback pumpTrigger);

    void begin();

    // Poll the button and react to input. Renders on state changes.
    // Call frequently (every main loop iteration).
    void loop();

    // Redraw the current screen. Safe to call from timers/tasks.
    void render();

    // Feed the pump animation frame counter used by the status screen.
    void setPumpProgress(uint8_t counter);

private:
    enum class Screen : uint8_t
    {
        Status,
        Menu,
    };

    enum class MenuItem : uint8_t
    {
        ResetTagCache,
        ToggleWifi,
        ToggleLora,
        Reboot,
        Exit,
        Count,
    };

    void renderStatus();
    void renderMenu();
    void openMenu();
    void selectCurrentItem();
    void reboot();

    Button *_button;
    Display *_display;
    RfidReader *_rfidReader;
    RfidStorage *_rfidStorage;
    Preferences *_systemPrefs;
    PumpTriggerCallback _pumpTrigger;

    Screen _screen = Screen::Status;
    uint8_t _selectedIndex = 0;
    uint8_t _pumpProgress = 0;
};

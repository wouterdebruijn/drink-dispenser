#include "Menu.h"

#include <WiFi.h>

#include "../lora/loramac.h"

Menu::Menu(Button *button,
           Display *display,
           RfidReader *rfidReader,
           RfidStorage *rfidStorage,
           Preferences *systemPrefs,
           PumpTriggerCallback pumpTrigger)
    : _button(button),
      _display(display),
      _rfidReader(rfidReader),
      _rfidStorage(rfidStorage),
      _systemPrefs(systemPrefs),
      _pumpTrigger(pumpTrigger)
{
}

void Menu::begin()
{
    _button->begin();
    // First render is driven by the caller after the startup screen delay.
}

void Menu::loop()
{
    ButtonEvent event = _button->poll();
    if (event == ButtonEvent::None)
    {
        return;
    }

    if (_screen == Screen::Status)
    {
        if (event == ButtonEvent::Click)
        {
            // Manual pump activation, standing in for an RFID scan.
            if (_pumpTrigger)
            {
                _pumpTrigger();
            }
        }
        else if (event == ButtonEvent::Hold)
        {
            openMenu();
        }
    }
    else // Screen::Menu
    {
        if (event == ButtonEvent::Click)
        {
            // Cycle to the next option.
            _selectedIndex = (_selectedIndex + 1) % static_cast<uint8_t>(MenuItem::Count);
            render();
        }
        else if (event == ButtonEvent::Hold)
        {
            selectCurrentItem();
        }
    }
}

void Menu::setPumpProgress(uint8_t counter)
{
    _pumpProgress = counter;
}

void Menu::openMenu()
{
    _screen = Screen::Menu;
    _selectedIndex = 0;
    _tagsReset = false; // Clear the "OK" confirmation from any previous session.
    render();
}

void Menu::selectCurrentItem()
{
    switch (static_cast<MenuItem>(_selectedIndex))
    {
    case MenuItem::ResetTagCache:
        _rfidStorage->clearTagData();
        _rfidStorage->storeTagData(); // Persist the cleared state to flash
        _tagsReset = true;
        break;

    case MenuItem::CycleMode:
        // Persisted only; applied on the next boot (use the Reboot item).
        cycleMode();
        break;

    case MenuItem::Reboot:
        reboot();
        return;

    case MenuItem::Exit:
        _screen = Screen::Status;
        break;

    default:
        break;
    }

    render();
}

void Menu::cycleMode()
{
    // Resolve the current mode to an index using the same priority as boot
    // (AP > WiFi > LoRa > Offline), then advance one step in the cycle:
    // 0 Offline -> 1 LoRa -> 2 WiFi -> 3 WiFi AP -> 0 Offline.
    uint8_t current;
    if (_systemPrefs->getBool("wifiApEnabled", false))
    {
        current = 3;
    }
    else if (_systemPrefs->getBool("wifiEnabled", false))
    {
        current = 2;
    }
    else if (_systemPrefs->getBool("loraEnabled", true))
    {
        current = 1;
    }
    else
    {
        current = 0;
    }

    uint8_t next = (current + 1) % 4;

    _systemPrefs->putBool("loraEnabled", next == 1);
    _systemPrefs->putBool("wifiEnabled", next == 2);
    _systemPrefs->putBool("wifiApEnabled", next == 3);
}

const char *Menu::modeName()
{
    if (_systemPrefs->getBool("wifiApEnabled", false))
    {
        return "WiFi AP";
    }
    if (_systemPrefs->getBool("wifiEnabled", false))
    {
        return "WiFi";
    }
    if (_systemPrefs->getBool("loraEnabled", true))
    {
        return "LoRa";
    }
    return "Offline";
}

void Menu::reboot()
{
    _rfidStorage->storeTagData(); // Persist any changes to flash before rebooting
    _display->clearBuffer();
    _display->setFont(u8g2_font_6x12_tr);
    _display->drawStr(20, 36, "Rebooting...");
    _display->sendBuffer();
    delay(600);
    ESP.restart();
}

void Menu::render()
{
    _display->clearBuffer();
    _display->setFontMode(1);
    _display->setBitmapMode(1);

    if (_screen == Screen::Status)
    {
        renderStatus();
    }
    else
    {
        renderMenu();
    }

    _display->sendBuffer();
}

void Menu::renderStatus()
{
    if (_rfidReader->getLastTagId() != 0)
    {
        uint16_t tagId = _rfidReader->getLastTagId();
        uint16_t tagCount = _rfidReader->getLastTagCount();

        // Display tag ID as decimal
        char tagIdStr[20];
        sprintf(tagIdStr, "Cheers #%d (%d)", tagId, tagCount / 20);

        _display->setFont(u8g2_font_6x12_tr);
        _display->drawStr(10, 58, tagIdStr);
    }
    else
    {
        _display->setFont(u8g2_font_6x12_tr);
        _display->drawStr(19, 58, "Shot Machine");

        _display->setFont(u8g2_font_5x8_tr);
        _display->drawStr(95, 58, "v5");
    }

    // Connectivity mode indicator, drawn as an icon in the top-right corner.
    // Only one of AP / WiFi / LoRa is ever active; otherwise the device is
    // offline. While a link is still coming up, a row of dots is drawn under
    // the icon to signal the connecting phase.
    const uint8_t *icon;
    uint8_t iconW;
    uint8_t iconH;
    bool connecting = false;

    if (_systemPrefs->getBool("wifiApEnabled", false))
    {
        icon = ap_bitmap;
        iconW = 6;
        iconH = 6;
    }
    else if (_systemPrefs->getBool("wifiEnabled", false))
    {
        icon = wifi_bitmap;
        iconW = 6;
        iconH = 6;
        connecting = WiFi.status() != WL_CONNECTED;
    }
    else if (_systemPrefs->getBool("loraEnabled", true))
    {
        icon = lora_bitmap;
        iconW = 7;
        iconH = 6;
        connecting = joinStatus != EV_JOINED;
    }
    else
    {
        icon = offline_bitmap;
        iconW = 5;
        iconH = 5;
    }

    const uint8_t iconY = 3;
    const uint8_t iconX = 3; // Draw top left
    _display->drawXBM(iconX, iconY, iconW, iconH, icon);

    if (connecting)
    {
        // Three dots centered under the icon as a "connecting..." hint.
        const uint8_t dotY = iconY + iconH + 2;
        const uint8_t centerX = iconX + iconW / 2;
        _display->drawBox(centerX - 3, dotY, 1, 1);
        _display->drawBox(centerX, dotY, 1, 1);
        _display->drawBox(centerX + 3, dotY, 1, 1);
    }

    if (_pumpProgress > 0)
    {
        switch (_pumpProgress / 6)
        {
        case 0:
            _display->drawXBM(46, 7, 36, 39, shot_glass_frame_1);
            break;
        case 1:
            _display->drawXBM(46, 7, 36, 39, shot_glass_frame_2);
            break;
        case 2:
            _display->drawXBM(46, 7, 36, 39, shot_glass_frame_3);
            break;
        case 3:
            _display->drawXBM(46, 7, 36, 39, shot_glass_frame_4);
            break;
        default:
            _display->drawXBM(46, 7, 36, 39, shot_glass_frame_0);
            break;
        }
    }
    else
    {
        _display->drawXBM(46, 7, 36, 39, shot_glass_frame_0);

        // A scan is being confirmed: overlay an hourglass on the empty glass.
        // Cleared automatically once the pump animation starts or the scan aborts.
        if (_rfidReader->isConfirmingTag())
        {
            _display->drawXBM(58, 19, 13, 12, hourglass_bitmap);
        }
    }
}

void Menu::renderMenu()
{
    char labels[static_cast<uint8_t>(MenuItem::Count)][20];
    strcpy(labels[static_cast<uint8_t>(MenuItem::ResetTagCache)], _tagsReset ? "Reset Tag Cache OK" : "Reset Tag Cache");
    sprintf(labels[static_cast<uint8_t>(MenuItem::CycleMode)], "Mode: %s", modeName());
    strcpy(labels[static_cast<uint8_t>(MenuItem::Reboot)], "Reboot");
    strcpy(labels[static_cast<uint8_t>(MenuItem::Exit)], "Exit");

    _display->setFont(u8g2_font_6x12_tr);

    const uint8_t itemCount = static_cast<uint8_t>(MenuItem::Count);
    const int rowHeight = 12;
    const int startY = 11;

    for (uint8_t i = 0; i < itemCount; i++)
    {
        int baselineY = startY + i * rowHeight;

        if (i == _selectedIndex)
        {
            // Highlight: filled bar with inverted text.
            _display->drawBox(0, baselineY - rowHeight + 3, 128, rowHeight);
            _display->setDrawColor(0);
            _display->drawStr(4, baselineY, labels[i]);
            _display->setDrawColor(1);
        }
        else
        {
            _display->drawStr(4, baselineY, labels[i]);
        }
    }
}

#include "Menu.h"

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
    render();
}

void Menu::selectCurrentItem()
{
    switch (static_cast<MenuItem>(_selectedIndex))
    {
    case MenuItem::ResetTagCache:
        _rfidStorage->clearTagData();
        break;

    case MenuItem::ToggleWifi:
    {
        // Persisted only; applied on the next boot (use the Reboot item).
        bool enabled = _systemPrefs->getBool("wifiEnabled", false);
        _systemPrefs->putBool("wifiEnabled", !enabled);
        break;
    }

    case MenuItem::ToggleLora:
    {
        // Persisted only; applied on the next boot (use the Reboot item).
        bool enabled = _systemPrefs->getBool("loraEnabled", true);
        _systemPrefs->putBool("loraEnabled", !enabled);
        break;
    }

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

void Menu::reboot()
{
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
        _display->drawStr(95, 58, "v4");
    }

    // LoRa link status
    _display->setFont(u8g2_font_4x6_tr);
    if (!_systemPrefs->getBool("loraEnabled", true))
    {
        _display->drawStr(1, 10, "LoRa Off");
    }
    else if (joinStatus != EV_JOINED)
    {
        _display->drawStr(1, 10, "Joining...");
    }
    else
    {
        _display->drawStr(1, 10, "Connected");
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
    }
}

void Menu::renderMenu()
{
    bool wifiEnabled = _systemPrefs->getBool("wifiEnabled", false);
    bool loraEnabled = _systemPrefs->getBool("loraEnabled", true);

    char labels[static_cast<uint8_t>(MenuItem::Count)][20];
    strcpy(labels[static_cast<uint8_t>(MenuItem::ResetTagCache)], "Reset Tag Cache");
    sprintf(labels[static_cast<uint8_t>(MenuItem::ToggleWifi)], "WiFi: %s", wifiEnabled ? "ON" : "OFF");
    sprintf(labels[static_cast<uint8_t>(MenuItem::ToggleLora)], "LoRa: %s", loraEnabled ? "ON" : "OFF");
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

#pragma once

#include <Arduino.h>

class Pump
{
public:
    Pump() = default;
    Pump(uint8_t pin);
    void begin();
    void enablePump() { setPumpPower(true); }
    void disablePump() { setPumpPower(false); }

private:
    uint8_t pin;
    void setPumpPower(bool powerState);
};

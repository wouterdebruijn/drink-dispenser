#include "Pump.h"

Pump::Pump(uint8_t pin)
    : pin(pin)
{
}

void Pump::begin()
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
}

void Interation()
{
    Serial.println("Pump duration ended, disabling pump.");
}

void Pump::setPumpPower(bool powerState)
{
    digitalWrite(pin, powerState ? LOW : HIGH);
}
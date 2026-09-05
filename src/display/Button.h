#pragma once

#include <Arduino.h>

class Button
{
public:
    Button(uint8_t pin) : _pin(pin), _lastState(false), _callback(nullptr) {}
    ~Button();
    void begin();
    void loop();
    void subscribe(void (*callback)(void));

private:
    int8_t _pin;
    bool _lastState;
    void (*_callback)(void);
};
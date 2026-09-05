#include "Button.h"

Button::Button(uint8_t pin) : _pin(pin), _lastState(false), _callback(nullptr) {}

Button::~Button() {}

void Button::begin()
{
    pinMode(_pin, INPUT_PULLUP);
}

void Button::loop()
{
    bool currentState = digitalRead(_pin) == LOW;

    if (currentState != _lastState)
    {
        _lastState = currentState;

        if (currentState && _callback)
        {
            _callback();
        }
    }
}

void Button::subscribe(void (*callback)(void))
{
    _callback = callback;
}
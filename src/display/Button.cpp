#include "Button.h"

void Button::begin()
{
    pinMode(_pin, INPUT_PULLUP);
}

ButtonEvent Button::poll()
{
    // Active low: pressed when the pin reads LOW.
    bool rawState = digitalRead(_pin) == LOW;
    unsigned long now = millis();

    // Debounce: only accept a new stable state after the bounce window.
    if (rawState != _debouncedState)
    {
        if (now - _lastChange >= DEBOUNCE_MS)
        {
            _debouncedState = rawState;
            _lastChange = now;

            if (_debouncedState)
            {
                // Press started.
                _pressed = true;
                _holdFired = false;
                _pressStart = now;
            }
            else if (_pressed)
            {
                // Released. A short press that never reached the hold
                // threshold counts as a click.
                _pressed = false;
                if (!_holdFired)
                {
                    return ButtonEvent::Click;
                }
            }
        }
    }
    else
    {
        _lastChange = now;
    }

    // Fire the hold event once while the button is still held down.
    if (_pressed && !_holdFired && (now - _pressStart >= _holdDurationMs))
    {
        _holdFired = true;
        return ButtonEvent::Hold;
    }

    return ButtonEvent::None;
}

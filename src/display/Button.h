#pragma once

#include <Arduino.h>

// Events produced by polling the button. A Click is a short press that has been
// released, a Hold fires once as soon as the press passes the hold threshold
// (while the button is still down, so the user gets immediate feedback).
enum class ButtonEvent : uint8_t
{
    None,
    Click,
    Hold,
};

class Button
{
public:
    Button(uint8_t pin, uint16_t holdDurationMs = 600)
        : _pin(pin), _holdDurationMs(holdDurationMs) {}

    void begin();

    // Reads the pin and returns the event that just completed, if any.
    // Call this frequently (every main loop iteration) for responsive input.
    ButtonEvent poll();

private:
    uint8_t _pin;
    uint16_t _holdDurationMs;

    bool _pressed = false;
    bool _holdFired = false;
    bool _debouncedState = false;
    unsigned long _pressStart = 0;
    unsigned long _lastChange = 0;

    static constexpr uint16_t DEBOUNCE_MS = 30;
};

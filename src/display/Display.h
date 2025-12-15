#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

class Display
{
public:
    Display();
    ~Display();
    bool begin();
    void startupText();
    void clear();
    void print(const char *str) { u8g2.print(str); }
    size_t print(const String &s) { return u8g2.print(s.c_str()); }
    size_t print(const char c) { return u8g2.print(c); }
    size_t print(unsigned char c, int base = DEC) { return u8g2.print(c, base); }
    size_t print(int n, int base = DEC) { return u8g2.print(n, base); }
    size_t print(unsigned int n, int base = DEC) { return u8g2.print(n, base); }
    size_t print(long n, int base = DEC) { return u8g2.print(n, base); }
    size_t print(unsigned long n, int base = DEC) { return u8g2.print(n, base); }
    size_t print(float n, int decimalPlaces = 2) { return u8g2.print(n, decimalPlaces); }
    size_t print(double n, int decimalPlaces = 2) { return u8g2.print(n, decimalPlaces); }
    size_t print(uint64_t n, int base = DEC) { return u8g2.print(n, base); }
    void println(const char *str) { u8g2.println(str); }
    void setCursor(int x, int y) { u8g2.setCursor(x, y); }
    void setFont(const uint8_t *font) { u8g2.setFont(font); }
    void clearBuffer() { u8g2.clearBuffer(); }
    void sendBuffer() { u8g2.sendBuffer(); }

private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2 = U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
};
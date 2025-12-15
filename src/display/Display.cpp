#include "Display.h"

Display::Display()
{
    // Constructor
}

Display::~Display()
{
    // Destructor
}

bool Display::begin()
{
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set default font
    u8g2.clearBuffer();                 // Clear the internal memory buffer
    u8g2.sendBuffer();                  // Send the buffer to the display
    return true;
}

void Display::clear()
{
    u8g2.clearBuffer(); // Clear the internal memory buffer
}

void Display::startupText()
{
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_6x12_tr);

    u8g2.drawStr(19, 58, "Shot Machine");

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(95, 58, "v4");

    u8g2.drawXBM(46, 7, 36, 39, shot_glass_frame_4);

    u8g2.sendBuffer();
}
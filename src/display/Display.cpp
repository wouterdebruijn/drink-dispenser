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
    // Startup text logo "Drink Dispenser" with version, including some simple art
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 10);
    u8g2.print("Drink Dispenser");
    u8g2.setCursor(0, 20);
    u8g2.print("Version 1.0");

    // Line art drinking glass
    u8g2.setCursor(0, 30);
    u8g2.print("  .-\"\"\"\"\"-.");
    u8g2.setCursor(0, 40);
    u8g2.print(" /          \\");
    u8g2.setCursor(0, 50);
    u8g2.print("|  Drink    |");
    u8g2.setCursor(0, 60);
    u8g2.print(" \\        /");
    u8g2.setCursor(0, 70);
    u8g2.print("  '-......-'");
    u8g2.sendBuffer(); // Send the buffer to the display
}
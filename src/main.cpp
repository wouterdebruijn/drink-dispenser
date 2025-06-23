#include "loramac.h"
#include "LoRaBoards.h"

void setup()
{
  setupBoards();
  // When the power is turned on, a delay is required.
  delay(1500);
  setupLMIC();
}

void loop()
{
  loopLMIC();
}

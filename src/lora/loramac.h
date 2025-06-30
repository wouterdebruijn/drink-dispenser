#pragma once

#include <Arduino.h>
#include <lmic.h>
#include "rfid/RfidStorage.h"

void setupLMIC(RfidStorage *storage);
void loopLMIC(void);

static osjob_t sendjob;
void do_send(osjob_t *j);
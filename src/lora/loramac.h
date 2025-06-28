#pragma once

#include <Arduino.h>
#include <lmic.h>

void setupLMIC(void);
void loopLMIC(void);

static osjob_t sendjob;
void do_send(osjob_t *j);
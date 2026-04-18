#pragma once

#include <Arduino.h>
#include <lmic.h>
#include "rfid/RfidStorage.h"
#include "common/TransportMode.h"

void setupLMIC(RfidStorage *storage);
void loopLMIC(void);
void setLoraTransport(TransportMode mode);

extern int joinStatus;

static osjob_t sendjob;
void do_send(osjob_t *j);
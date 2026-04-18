#pragma once
#include <stdint.h>

enum TransportMode : uint8_t {
    TRANSPORT_LORA = 0,
    TRANSPORT_WIFI = 1,
    TRANSPORT_BOTH = 2
};

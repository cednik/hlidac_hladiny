#pragma once

#include <SmartLeds.h>

#define ILEDS_COUNT 8
#define ILEDS_CHANNEL 0

SmartLed iLeds(LED_WS2812B, ILEDS_COUNT, PIN_ILED, ILEDS_CHANNEL, SingleBuffer);

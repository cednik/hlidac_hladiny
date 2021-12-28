#define Binary_h // Content of this file interfere with fmt

#include "Arduino.h"
#include <fmt/core.h>

extern "C" void app_main()
{
    initArduino();
    Serial.begin(115200);
    //Serial.print("\nHlidac hladiny\n");
    fmt::print("\nHlidac hladiny\n");
    for (;;yield()) {
        Serial.print("pokus\n");
        delay(1000);
    }
}
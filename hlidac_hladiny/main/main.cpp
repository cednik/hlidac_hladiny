#include "Arduino.h"

extern "C" void app_main()
{
    initArduino();
    Serial.begin(115200);
    Serial.print("\nHlidac hladiny\n");
    for (;;yield()) {
        Serial.print("pokus dnes\n");
        delay(1000);
    }
}
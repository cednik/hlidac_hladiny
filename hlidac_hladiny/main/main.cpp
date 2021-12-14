#include "Arduino.h"

extern "C" void app_main()
{
    initArduino();
    Serial.begin(115200);
    Serial.print("\nHlidac hladiny\n");
    for (;;yield()) {
        Serial.print("test\n");
        delay(1000);
    }
}
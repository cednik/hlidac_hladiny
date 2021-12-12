#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>

#include <format.h>

#include "pinout.hpp"

using fmt::print;

extern "C" void app_main() {
    initArduino();

    Serial.begin(115200);
    print(Serial, "\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);
    for (;;taskYIELD()) {
        Serial.println("app_main");
        delay(1000);
    }
}

void setup() {}
void loop() {}
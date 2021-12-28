#define Binary_h // Content of this Arduino file interfere with fmt

#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"
#include "i2c.hpp"
#include "display.hpp"

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();

    for (;;yield()) {
        print("pokus\n");
        delay(1000);
    }
}
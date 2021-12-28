#define Binary_h // Content of this Arduino file interfere with fmt

#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"

#include <Wire.h>
static const uint32_t I2C_FREQUENCY = 100000UL; // Hz

void i2c_scan(TwoWire& wire) {
    print("i2c scan:\n");
    uint8_t found = 0;
    for(uint8_t i = 1; i != 128; ++i) {
        wire.beginTransmission(i);
        if (wire.endTransmission() == 0) {
            print("Device fount at 0x{:02X}\n", i);
            ++found;
        }
    }
    print("Found {} devices\n", found);
}

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    for (;;yield()) {
        print("pokus\n");
        delay(1000);
    }
}
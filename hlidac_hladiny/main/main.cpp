#define Binary_h // Content of this Arduino file interfere with fmt

#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"
#include "i2c.hpp"
#include "display.hpp"
#include "DHT.hpp"

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();
    thermometer.begin();

    for (;;yield()) {
        const float temp = thermometer.readTemperature();
        const float humid = thermometer.readHumidity();
        print("t: {:4.1f} °C; h: {:2.0f}\n", temp, humid);
        delay(1000);
    }
}
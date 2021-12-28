#define Binary_h // Content of this Arduino file interfere with fmt

#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"
#include "i2c.hpp"
#include "display.hpp"
#include "DHT.hpp"
#include "rtc.hpp"
#include "iLeds.hpp"

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();
    thermometer.begin();
    RTC::init();
    int led = 0;
    for (;;yield()) {
        const float temp = thermometer.readTemperature();
        const float humid = thermometer.readHumidity();
        print("l: {}; t: {:4.1f} °C; h: {:2.0f}\n", led, temp, humid);
        
        for(int i = 0; i != led; ++i)
            iLeds[i] = Rgb(1, 1, 1);
        for(int i = led; i != ILEDS_COUNT; ++i)
            iLeds[i] = Rgb(0, 0, 0);
        if (led++ == ILEDS_COUNT)
            led = 0;
        iLeds.show();

        delay(1000);
    }
}
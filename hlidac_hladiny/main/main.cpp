#define Binary_h // Content of this Arduino file interfere with fmt

#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"
#include "i2c.hpp"
#include "display.hpp"
#include "DHT.hpp"
#include "rtc.hpp"

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    //delay(100);
    //Serial.begin(115200);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();
    thermometer.begin();
    RTC::init();

    for (;;yield()) {
        const float temp = thermometer.readTemperature();
        const float humid = thermometer.readHumidity();
        print("t: {:4.1f} °C; h: {:2.0f}\n", temp, humid);
        /*if (Serial.available()) {
            char c = Serial.read();
            switch(c) {
            case 'f':
                RTC::init();
                if (RTC::isConnected) {
                    rtc.writeSqwPinMode(DS1307_ON);
                    print("RTC set\n");
                }
                break;
            default:
                print("'{}' received\n", c);
                break;
            }
        }*/
        delay(1000);
    }
}
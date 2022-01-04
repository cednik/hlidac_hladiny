#include <Arduino.h>
#include <fmt/core.h>

using fmt::print;

#include "pinout.hpp"
#include "i2c.hpp"
#include "display.hpp"
#include "DHT.hpp"
#include "rtc.hpp"
#include "iLeds.hpp"
#include <uart.hpp>

Uart serial1 { UART_NUM_1 };

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();
    thermometer.begin();
    RTC::init();

    serial1.begin(115200, SERIAL_8N1, 22, 23);

    for (;;taskYIELD()) {
        Uart::process(&serial1);
        while (serial1.available()) {
            char c = serial1.read();
            print("received {}\n", c);
            serial1.write(c);
        }
        //print("tick\n");
        vTaskDelay(1);
    }
}
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

    esp_log_level_set("*", ESP_LOG_VERBOSE);

    serial1
        .pins(23, 22)
        .config(115200, SERIAL_8N1)
        .open();

    for (;;taskYIELD()) {
        while (serial1.available()) {
            char c = serial1.read();
            print("received {}\n", c);
            serial1.write(c);
        }
        //ESP_LOGD("MAIN", "tick");
        //vTaskDelay(100 / portTICK_PERIOD_MS);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
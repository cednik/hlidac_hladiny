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

#include <vector>

Uart& serial1 = Uart::get_port(UART_NUM_1);

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
        .onData([](Uart& uart) {
            const size_t len = uart.available();
            std::vector<char> buf(len + 1);
            uart.read(reinterpret_cast<uint8_t*>(&buf[0]), len);
            buf[len] = '\0';
            print("recv \"{}\"\n", static_cast<char*>(&buf[0]));
        } )
        .open();

    for (;;taskYIELD()) {
        //ESP_LOGD("MAIN", "tick");
        //vTaskDelay(100 / portTICK_PERIOD_MS);
        serial1.println("ahoj");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
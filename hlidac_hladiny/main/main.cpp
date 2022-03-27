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
#include <log_buffer.hpp>

#include <vector>

static const char* LOG_TAG = "LOG_MAIN";

Uart& serial0 = Uart::get_port(UART_NUM_0);

extern "C" void app_main()
{
    initArduino();
    print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);

    LogBuffer::begin();

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    Display::init();
    thermometer.begin();
    RTC::init();

    esp_log_level_set("*", ESP_LOG_DEBUG);

    ESP_LOGI(LOG_TAG, "Opening serial0");
    LogBuffer::pause();
    delay(20);
    serial0
        .pins(PIN_TXD0, PIN_RXD0)
        .config(115200, SERIAL_8N1)
        .onData([](Uart& uart) {
            const size_t len = uart.available();
            std::vector<char> buf(len + 1);
            uart.read(reinterpret_cast<uint8_t*>(&buf[0]), len);
            buf[len] = '\0';
            print("recv \"{}\"\n", static_cast<char*>(&buf[0]));
        } )
        .open();
    serial0.make_cstream_unbuffered();
    LogBuffer::redirect(serial0);
    LogBuffer::resume();
    ESP_LOGI(LOG_TAG, "Starting main loop");
    print(stdout, "stdout test\n");

    int i = 0;
    for (;;taskYIELD()) {
        //ESP_LOGD("MAIN", "tick");
        //vTaskDelay(100 / portTICK_PERIOD_MS);
        //print(serial1.cstream(), "{:4} ahoj\n", i%10000);
        print("{:4} ahoj\n", i%10000);
        ESP_LOGI(LOG_TAG, "pokus %d", i);
        ++i;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
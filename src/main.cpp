#include <Arduino.h>

//#include <format.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "pinout.hpp"

//using fmt::print;

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_ADDR 0x3C

Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

#define MAX_PULSE_LENGTH 25000 // us
#define MAX_RANGE 4000 // mm

#define SPEED_OF_SOUND 343.0 // m/s @ 20 °C dry air

//typedef uint32_t time_t;

uint16_t us2mm(time_t t) {
  return float(t) * (SPEED_OF_SOUND / 1000 / 2);
}

inline static void checkReset() {
    esp_reset_reason_t resetReason = esp_reset_reason();
    switch (resetReason) {
    case ESP_RST_UNKNOWN:
        printf("\tUnknown reset - strange\n");
        break;
    case ESP_RST_POWERON:
        printf("\tPoweron reset\n");
        break;
    case ESP_RST_EXT:
        printf("\tExternal reset\n");
        break;
    case ESP_RST_SW:
        printf("\tSoftware reset\n");
        break;
    case ESP_RST_PANIC:
        printf("\tReset due to core panic - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_INT_WDT:
        printf("\tReset due to interrupt watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_TASK_WDT:
        printf("\tReset due to task watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_WDT:
        printf("\tReset due to some watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_DEEPSLEEP:
        printf("\tWaked from deep sleep\n");
        break;
    case ESP_RST_BROWNOUT:
        printf("\tBrownout reset - please check power\n");
        break;
    case ESP_RST_SDIO:
        printf("\tSDIO reset - strange\n");
        break;
    }
}

void i2c_scan(TwoWire& wire) {
    printf("i2c scan:\n");
    uint8_t found = 0;
    for(uint8_t i = 1; i != 128; ++i) {
        wire.beginTransmission(i);
        if (wire.endTransmission() == 0) {
            printf("Device fount at 0x%02X\n", i);
            //print("Device fount at 0x{:02X}\n", i);
            ++found;
        }
    }
    printf("Found %d devices\n", found);
}

void setup() {
    Serial.begin(115200);
    printf("\nHlidac hladiny\n\t%s %s\n", __DATE__, __TIME__);
    //print("\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);
    checkReset();

    Wire.begin(PIN_SDA, PIN_SCL);
    i2c_scan(Wire);

    bool displayConnected = true;
    if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR, true, false)) { // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        printf("Display initialization failed\n");
        displayConnected = false;
    }

    if (displayConnected) {
        display.dim(false);
        display.clearDisplay();
        
        display.setTextColor(WHITE, BLACK);

        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("Hlidac\n\rhladiny");
        
        display.setTextSize(1);
        
        // display.setCursor(16, 24);
        // display.print(__DATE__);
        // display.write(' ');
        // display.print(__TIME__);
        
        display.setCursor(80, 8);
        display.print("FW v0.1");

        display.display();
        delay(2000);

        display.clearDisplay();
    }
}

void loop() {
}
#include <Arduino.h>

#include <format.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#include <time.hpp>

#include "pinout.hpp"

using fmt::print;

#define I2C_FREQUENCY 100000 // Hz

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_ADDR 0x3C

Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

#define MAX_PULSE_LENGTH 25000 // us
#define MAX_RANGE 4000 // mm

#define SPEED_OF_SOUND 343.0 // m/s @ 20 °C dry air

//typedef uint32_t time_t;

uint16_t uts2mm(time_t t) {
  return float(t) * (SPEED_OF_SOUND / 1000 / 2);
}

time_t utsMeas(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, HIGH);
    wait(usec(20));
    digitalWrite(trig, LOW);
    //wait(usec(1));
    return pulseIn(echo, HIGH);//, (10 / SPEED_OF_SOUND) * 1000000);
}

inline static void checkReset() {
    esp_reset_reason_t resetReason = esp_reset_reason();
    switch (resetReason) {
    case ESP_RST_UNKNOWN:
        print("\tUnknown reset - strange\n");
        break;
    case ESP_RST_POWERON:
        print("\tPoweron reset\n");
        break;
    case ESP_RST_EXT:
        print("\tExternal reset\n");
        break;
    case ESP_RST_SW:
        print("\tSoftware reset\n");
        break;
    case ESP_RST_PANIC:
        print("\tReset due to core panic - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_INT_WDT:
        print("\tReset due to interrupt watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_TASK_WDT:
        print("\tReset due to task watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_WDT:
        print("\tReset due to some watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_DEEPSLEEP:
        print("\tWaked from deep sleep\n");
        break;
    case ESP_RST_BROWNOUT:
        print("\tBrownout reset - please check power\n");
        break;
    case ESP_RST_SDIO:
        print("\tSDIO reset - strange\n");
        break;
    }
}

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

void setup() {
    Serial.begin(115200);
    print(Serial, "\nHlidac hladiny\n\t{} {}\n", __DATE__, __TIME__);
    checkReset();

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQUENCY);
    i2c_scan(Wire);

    bool displayConnected = true;
    if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR, true, false)) { // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        print(Serial, "Display initialization failed\n");
        displayConnected = false;
    }

    if (displayConnected) {
        display.dim(false);
        display.clearDisplay();
        
        display.setTextColor(WHITE, BLACK);

        display.setTextSize(2);
        display.setCursor(0, 0);
        print(display, "Hlidac\n\rhladiny");
        
        display.setTextSize(1);
        
        // display.setCursor(16, 24);
        // display.print(__DATE__);
        // display.write(' ');
        // display.print(__TIME__);
        
        display.setCursor(80, 8);
        print(display, "FW v0.1");

        display.display();
        delay(500);

        display.clearDisplay();
        display.display();
        display.setTextSize(2);
        display.setCursor(0, 0);
    }

    pinMode(PIN_LED, OUTPUT);

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT_PULLUP);

    timeout blink(msec(500));
    timeout meas(msec(1000));
    
    for(;;) {
        if (blink) {
            blink.ack();
            digitalWrite(PIN_LED, digitalRead(PIN_LED) == LOW);
        }
        if (meas) {
            meas.ack();
            const uint16_t mm = uts2mm(utsMeas(PIN_TRIG, PIN_ECHO));
            display.setCursor(0, 0);
            print(display, "d: {:4} mm\n", mm);
            print(Serial , "d: {:4} mm\n", mm);
            display.display();
        }
    }
}

void loop() {
}
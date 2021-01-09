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

int16_t utsMeas(HardwareSerial& port) {
    port.flush();
    port.write(0x55); // start meas command
    uint8_t checksum = 0xff;
    int16_t res = 0;
    timeout t(msec(100));
    while (port.available() < 4) {
        if (t) {
            port.flush();
            return -1;
        }
    }
    if (port.read() != 0xff) {
        port.flush();
        return -2;
    }
    res = port.read();
    checksum += res;
    res <<= 8;
    res |= port.read();
    checksum += (res & 0xFF);
    checksum -= port.read();
    if (checksum != 0)
        return -res;
    return res;
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
    HardwareSerial& uts = Serial1;
    uts.begin(9600, SERIAL_8N1, PIN_ECHO, PIN_TRIG);

    timeout blink(msec(500));
    timeout meas(msec(1000));
    
    for(;;) {
        if (blink) {
            blink.ack();
            digitalWrite(PIN_LED, digitalRead(PIN_LED) == LOW);
        }
        if (meas) {
            meas.ack();
            const uint16_t mm = utsMeas(uts);
            display.setCursor(0, 0);
            print(display, "d: {:4} mm\n", mm);
            print(Serial , "d: {:4} mm\n", mm);
            display.display();
        }
    }
}

void loop() {
}
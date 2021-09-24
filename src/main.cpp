#include <Arduino.h>

#include <format.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DHT.h>
#include <RTClib.h>

#include <SmartLeds.h>

#include <time.hpp>

#include "pinout.hpp"

using fmt::print;

#define I2C_FREQUENCY 100000 // Hz

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_ADDR 0x3C

#define ILEDS_COUNT 8
#define ILEDS_CHANNEL 0

Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

SmartLed iLeds(LED_WS2812B, ILEDS_COUNT, PIN_ILED, ILEDS_CHANNEL, SingleBuffer);

DHT thermometer(PIN_DHT11, DHT11);

RTC_DS1307 rtc;

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

    pinMode(PIN_BUZZER, OUTPUT);
    print("Buzzer on\n");
    digitalWrite(PIN_BUZZER, HIGH);
    wait(msec(1000));
    digitalWrite(PIN_BUZZER, LOW);
    print("Buzzer off\n");
    wait(msec(1000));

    pinMode(PIN_RELAY, OUTPUT);
    print("Relay on\n");
    digitalWrite(PIN_RELAY, HIGH);
    wait(msec(1000));
    digitalWrite(PIN_RELAY, LOW);
    print("Relay off\n");

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT_PULLUP);
    HardwareSerial& uts = Serial1;
    uts.begin(9600, SERIAL_8N1, PIN_ECHO, PIN_TRIG);

    pinMode(PIN_ILED, OUTPUT);
    for (uint8_t i = 0; i != ILEDS_COUNT; ++i)
        iLeds[i] = Rgb(16, 16, 16);
    iLeds.show();
    iLeds.wait();

    thermometer.begin();
    
    bool rtcConnected = false;
    if (rtc.begin(&Wire)) {
        rtcConnected = true;
        if (rtc.isrunning()) {
            print(Serial, "RTC time: {}\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str());
        } else {
            print(Serial, "RTC is not running!\n");
        }
    } else {
        print(Serial, "RTC not connected!\n");
    }
    

    timeout blink(msec(500));
    timeout meas(msec(1000));
    
    for(;;) {
        if (blink) {
            blink.ack();
            Rgb c;
            if (digitalRead(PIN_LED) == LOW) {
                digitalWrite(PIN_LED, HIGH);
                c = Rgb(16, 16, 16);
            } else {
                digitalWrite(PIN_LED, LOW);
                c = Rgb(0, 0, 0);
            }
            for (uint8_t i = 0; i != ILEDS_COUNT; ++i)
                iLeds[i] = c;
            iLeds.show();
            iLeds.wait();
        }
        if (meas) {
            meas.ack();
            const int16_t mm = utsMeas(uts);
            const float temp = thermometer.readTemperature();
            const float humid = thermometer.readHumidity();
            
            display.setCursor(0, 0);
            print(display, "d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}\n", mm, temp, humid);
            print(Serial , "d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}\n", mm, temp, humid);
            display.display();
        }
        if (Serial.available()) {
            char c = Serial.read();
            switch (c) {
            case '\n':
            case '\r':
                Serial.write('\n');
                break;
            case 'T':
                if (!rtcConnected) {
                    print(Serial, "No RTC connected.\n");
                } else {
                    print(Serial, "Insert time in ISO 8601 format (2021-09-24T13:48:12) and press enter:\n\t");
                    constexpr size_t BUFLEN = 20;
                    char buf[BUFLEN];
                    int i = 0;
                    timeout timeOut(sec(5));
                    while (!timeOut) {
                        if (Serial.available()) {
                            char c = Serial.read();
                            if (c == '\b') {
                                if (i > 0) {
                                    --i;
                                    Serial.write(c);
                                } else {
                                    Serial.write('\a');
                                }
                            } else if (c == '\n' || c == '\r') {
                                Serial.write('\n');
                                if (i != (BUFLEN-1)) {
                                    print(Serial, "Too short input ({}/{}), aborting.\n", i, (BUFLEN-1));
                                } else {
                                    buf[i++] = '\0';
                                    DateTime input(buf);
                                    if (!input.isValid()) {
                                        print(Serial, "Invalid input \"{}\"\n", buf);
                                    } else {
                                        rtc.adjust(input);
                                        print(Serial, "RTC time set to {}\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str());
                                    }
                                }
                                timeOut.cancel();
                                break;
                            } else {
                                if (i == BUFLEN) {
                                    Serial.write('\a');
                                } else {
                                    buf[i++] = c;
                                    Serial.write(c);
                                }
                            }
                            timeOut.restart();
                        }
                    }
                    if (timeOut) {
                        print(Serial, "\nTimeout.\n");
                    }
                }
                break;
            }
        }
    }
}

void loop() {
}
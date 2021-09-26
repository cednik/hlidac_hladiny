#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"

#include <string>
#include <cctype>

#include <format.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DHT.h>
#include <RTClib.h>

#include <SmartLeds.h>

#include <BasicOTA.hpp>

#include <time.hpp>

#include "pinout.hpp"

using fmt::print;

#define NVS_KEY_WIFI_SSID "WiFi_SSID"
#define NVS_KEY_WIFI_PSWD "WiFi_PSWD"
#define NVS_KEY_NAME "NAME"

#define USER_SERVER_PORT 54321

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

esp_err_t nvs_get_string(nvs_handle handle, const char *key, std::string& out_value) {
    size_t len = 0;
    esp_err_t err = nvs_get_str(handle, key, nullptr, &len);
    if (err != ESP_OK)
        return err;
    out_value.resize(len);
    err = nvs_get_str(handle, key, &out_value[0], &len);
    out_value.pop_back(); // remove original C-string trailing zero
    return err;
}

std::string enctype2str(uint8_t auth) {
    switch (auth) {
        case WIFI_AUTH_OPEN: return "open";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE : return "WPA2_ENTERPRISE";
        default: return fmt::format("Unknown[{}]", static_cast<int>(auth));
    }
}

template <class Stream>
std::string read_string(Stream& stream, const std::string& msg = "", timeout::time_type tmout = 0, size_t max_len = 0) {
    print(stream, msg);
    std::string res;
    if (max_len != 0) {
        res.reserve(max_len);
    }
    timeout timeOut(tmout);
    if (tmout == 0)
        timeOut.cancel();
    while (!timeOut) {
        if (stream.available()) {
            char c = stream.read();
            if (c == '\b') {
                if (!res.empty()) {
                    res.pop_back();
                    stream.write(c);
                } else {
                    stream.write('\a');
                }
            } else if (c == '\n' || c == '\r') {
                stream.write('\n');
                timeOut.reset(msec(2));
                timeOut.restart();
                while (!timeOut) {
                    switch (stream.peek()) {
                    case -1:
                        continue;
                    case '\n':
                    case '\r':
                        stream.read();
                    default:
                        timeOut.force();
                        break;
                    }
                }
                timeOut.cancel();
                break;
            } else {
                if (max_len > 0 && res.length() == max_len) {
                    stream.write('\a');
                } else {
                    res.push_back(c);
                    stream.write(c);
                }
            }
            if (timeOut.running())
                timeOut.restart();
        }
    }
    return res;
}

template <class Stream>
void wifi_save_and_connect(nvs_handle nvsHandle, const std::string& wifi_ssid, const std::string& wifi_pswd, Stream& debug) {
    esp_err_t err = nvs_set_str(nvsHandle, NVS_KEY_WIFI_SSID, wifi_ssid.c_str());
    if (err != ESP_OK) {
        print(debug, "\nSaving SSID failed, because {}\n", esp_err_to_name(err));
        return;
    }
    err = nvs_set_str(nvsHandle, NVS_KEY_WIFI_PSWD, wifi_pswd.c_str());
    if (err != ESP_OK) {
        print(debug, "\nSaving PSWD failed, because {}\n", esp_err_to_name(err));
        return;
    }
    err = nvs_commit(nvsHandle);
    if (err != ESP_OK) {
        print(debug, "\nNVS commit failed\n");
        return;
    }
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_pswd.c_str());
    WiFi.setAutoReconnect(true);
}
//void wifi_save_and_connect(nvs_handle nvsHandle, const std::string& wifi_ssid) { wifi_save_and_connect(nvsHandle, wifi_ssid, wifi_pswd, Serial); }

bool isInt(const std::string& str, int* pRes = nullptr) {
    int res = 0;
    if (str.empty())
        return false;
    auto c = str.begin();
    while (isspace(*c)) {
        if (++c == str.end())
            return false;
    }
    bool minus = false;
    if (*c == '-') {
        minus = true;
        if (++c == str.end())
            return false;
    }
    for (; c != str.end(); ++c) {
        if (*c < '0' || *c > '9')
            return false;
        res = res * 10 + *c - '0';
    }
    if (minus)
        res = -res;
    if (pRes)
        *pRes = res;
    return true;
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
    //wait(msec(1000));

    pinMode(PIN_RELAY, OUTPUT);
    /*print("Relay on\n");
    digitalWrite(PIN_RELAY, HIGH);
    wait(msec(1000));
    digitalWrite(PIN_RELAY, LOW);
    print("Relay off\n");*/

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
    
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        print(Serial, "NVS partition was truncated and needs to be erased\n");
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    // Open
    print(Serial, "Opening Non-Volatile Storage (NVS) handle... ");
    // Handle will automatically close when going out of scope or when it's reset.
    nvs_handle nvsHandle;
    err = nvs_open("storage", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        print(Serial, "Error {} opening NVS handle!\n", esp_err_to_name(err));
        nvsHandle = 0;
    } else {
        print(Serial, "\n");
    }

    std::string my_name = "Studna";
    err = nvs_get_string(nvsHandle, NVS_KEY_NAME, my_name);
    switch (err) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No name set.\n");
        break;
    default:
        print(Serial, "Error {} {} reading name!\n", err, esp_err_to_name(err));
        break;
    }
    print(Serial, "Hi, I'm {}.\n", my_name);

    std::string wifi_ssid;
    std::string wifi_pswd;
    err = nvs_get_string(nvsHandle, NVS_KEY_WIFI_SSID, wifi_ssid);
    switch (err) {
    case ESP_OK:
        nvs_get_string(nvsHandle, NVS_KEY_WIFI_PSWD, wifi_pswd);
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_ssid.c_str(), wifi_pswd.c_str());
        WiFi.setAutoReconnect(true);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No WiFi set\n");
        break;
    default:
        print(Serial, "Error {} {} reading Wi-Fi SSID!\n", err, esp_err_to_name(err));
        break;
    }

    BasicOTA OTA;

    timeout blink(msec(500));
    timeout meas(msec(1000));

    wl_status_t wifi_last_status = WiFi.status();

    WiFiServer user_server(USER_SERVER_PORT, 1);
    WiFiClient user_client;
    bool user_client_connected = false;
    Stream* user_stream = &Serial;
    
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
            print(*user_stream, "d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}\n", mm, temp, humid);
            display.display();
        }
        wl_status_t wifi_status = WiFi.status();
        if (wifi_status != wifi_last_status) {
            switch (wifi_status) {
            case WL_IDLE_STATUS:
                print(Serial, "Wi-Fi idle\n");
                break;
            case WL_CONNECTED:
                print(Serial, "Wi-Fi connected: IP {}\n", WiFi.localIP().toString().c_str());
                if (!MDNS.begin(my_name.c_str())) {
                    print(Serial, "Error setting up MDNS responder!");
                }
                OTA.begin();
                user_server.begin();
                break;
            case WL_CONNECT_FAILED:
                print(Serial, "Wi-Fi connecting failed\n");
                break;
            case WL_CONNECTION_LOST:
                print(Serial, "Wi-Fi connection lost\n");
                break;
            case WL_DISCONNECTED:
                print(Serial, "Wi-Fi disconnected\n");
                user_server.end();
                break;
            default:
                print(Serial, "Wi-Fi status {}\n", int(wifi_status));
                break;
            }
            wifi_last_status = wifi_status;
        }
        if (!user_client) {
            if (user_client_connected) {
                print(Serial, "Client disconnected\n");
                user_client_connected = false;
                user_stream = &Serial;
            }
            if (user_client = user_server.accept()) {
                print(Serial, "Accepted client at {}\n", user_client.remoteIP().toString().c_str());
                user_client_connected = true;
                print(user_client, "Hi, I'm {}.\n", my_name);
                print(user_client, "RTC time: {}\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str());
                user_stream = &user_client;
            }
        }
        if (user_stream->available()) {
            char c = user_stream->read();
            switch (c) {
            case '\n':
            case '\r':
                user_stream->write('\n');
                break;
            case 'T':
                if (!rtcConnected) {
                    print(*user_stream, "No RTC connected.\n");
                } else {
                    constexpr size_t EXPECTED_LEN = 19;
                    std::string input = read_string(*user_stream, "Insert time in ISO 8601 format (2021-09-24T13:48:12) and press enter:\n\t", 5, EXPECTED_LEN);
                    if (input.length() != EXPECTED_LEN) {
                        print(*user_stream, "Too short input ({}/{}).\n", input.length(), EXPECTED_LEN);
                    } else {
                        DateTime new_time(input.c_str());
                        if (!new_time.isValid()) {
                            print(*user_stream, "Invalid input \"{}\"\n", input);
                        } else {
                            rtc.adjust(new_time);
                            print(*user_stream, "RTC time set to {}\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str());
                        }
                    }
                }
                break;
            case 'W': {
                    print(*user_stream, "Scanning networks:\n");
                    int16_t networks = WiFi.scanNetworks(false, true);
                    String ssid;
                    uint8_t encryption;
                    int32_t rssi;
                    uint8_t* bssid;
                    int32_t channel;
                    int i;
                    print(*user_stream, "\tindex   SSID                                encryption           RSSI        channel MAC\n");
                    for(i = 0; i != networks; ++i) {
                        bool res = WiFi.getNetworkInfo(i, ssid, encryption, rssi, bssid, channel);
                        print(*user_stream, "\t{}{:2}\t{:32}\t{:15}\t{:4} dBm\t  {:2}  \t{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}\n", res ? ' ' : '!',
                            i, ssid.c_str(), enctype2str(encryption), rssi, channel, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
                    }
                    std::string cmd = read_string(*user_stream, "Type C for disable Wi-Fi, N for enter another SSID or selected Wi-Fi index and press enter: ");
                    if (cmd.empty()) {
                        print(*user_stream, "No input\n");
                    } else if (cmd == "C") {
                        err = nvs_erase_key(nvsHandle, NVS_KEY_WIFI_SSID);
                        if (err == ESP_OK)
                            print(*user_stream, "\nWi-Fi disabled\n");
                        else
                            print(*user_stream, "\nDisabling Wi-Fi failed, because {}\n", esp_err_to_name(err));
                    } else if (cmd == "N") {
                        wifi_ssid = read_string(*user_stream, "Enter SSID and press enter: ");
                        wifi_pswd = read_string(*user_stream, "Enter PSWD and press enter: ");
                        wifi_save_and_connect(nvsHandle, wifi_ssid, wifi_pswd, *user_stream);
                    } else if (isInt(cmd, &i)) {
                        bool res = WiFi.getNetworkInfo(i, ssid, encryption, rssi, bssid, channel);
                        if (!res) {
                            print(*user_stream, "Network {} {} is somehow invalid\n", i, ssid.c_str());
                        } else { 
                            wifi_ssid = ssid.c_str();
                            print(*user_stream, "Selected network {}: {}\n", i, wifi_ssid);
                            wifi_pswd = read_string(*user_stream, "Enter PSWD and press enter: ");
                            wifi_save_and_connect(nvsHandle, wifi_ssid, wifi_pswd, *user_stream);
                        }
                    } else {
                        print(*user_stream, "Unknown command {}\n", cmd);
                    }
                    WiFi.scanDelete();
                }
                break;
            case 'N':
                my_name = read_string(*user_stream, "Enter new device name: ", 0, 64);
                if (wifi_status == WL_CONNECTED) {
                    if (!MDNS.begin(my_name.c_str())) {
                        print(*user_stream, "Error setting up MDNS responder!");
                    }
                }
                err = nvs_set_str(nvsHandle, NVS_KEY_NAME, my_name.c_str());
                if (err != ESP_OK) {
                    print(*user_stream, "\nSaving name failed, because {}\n", esp_err_to_name(err));
                    break;
                }
                err = nvs_commit(nvsHandle);
                if (err != ESP_OK) {
                    print(*user_stream, "\nNVS commit failed\n");
                }
                break;
            }
        }
        OTA.handle();
    }
}

void loop() {
}
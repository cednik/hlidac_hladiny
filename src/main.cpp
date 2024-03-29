#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

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

#define NVS_KEY_NAME            "NAME"
#define NVS_KEY_WIFI_SSID       "WiFi_SSID"
#define NVS_KEY_WIFI_PSWD       "WiFi_PSWD"
#define NVS_KEY_COM_SERVER_ADDR "COM_ADDR"
#define NVS_KEY_COM_SERVER_PORT "COM_PORT"
#define NVS_KEY_COM_SERVER_TMOT "COM_TMOT"
#define NVS_KEY_LEVEL_ZERO      "LEVEL_ZERO"
#define NVS_KEY_LEVEL_ON        "LEVEL_ON"
#define NVS_KEY_MEAS_PERIOD     "MEAS_T"

#define USER_SERVER_PORT 54321

#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   16          //Multisampling

#define LEVEL_ERROR -32768

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
    //print("0x{:02X}", uint8_t(resetReason));
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
        print("\tReset due to RTC watchdog - triing again\n");
        //vTaskSuspend(nullptr);
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
bool isInt16(const std::string& str, int16_t* pRes = nullptr) {
    int n;
    bool res = isInt(str, &n);
    if (res && pRes)
        *pRes = int16_t(n);
    return res;
}
bool isUint16(const std::string& str, uint16_t* pRes = nullptr) {
    int n;
    bool res = isInt(str, &n);
    if (res && pRes)
        *pRes = uint16_t(n);
    return res;
}
bool isUint32(const std::string& str, uint32_t* pRes = nullptr) {
    int n;
    bool res = isInt(str, &n);
    if (res && pRes)
        *pRes = uint16_t(n);
    return res;
}

template <class Stream>
void listDir(Stream& stream, fs::FS &fs, const char * dirname, uint8_t levels = 0){
    print(stream, "Listing directory: {}\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        print(stream, "Failed to open directory\n");
        return;
    }
    if (!root.isDirectory()) {
        print(stream, "Not a directory\n");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if(file.isDirectory()) {
            print(stream, "  DIR : {}\n", file.name());
            if(levels) {
                std::string path = dirname;
                path += '/';
                path += file.name();
                listDir(stream, fs, path.c_str(), levels -1);
            }
        } else {
            print(stream, "  FILE: {}  SIZE: {}\n", file.name(), file.size());
        }
        file = root.openNextFile();
    }
}

static void adc_check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
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

    HardwareSerial& uts = Serial1;
    uts.begin(9600, SERIAL_8N1, PIN_ECHO, PIN_TRIG);

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT_PULLUP);
    pinMode(PIN_ILED, OUTPUT);
    pinMode(PIN_RTC_WAKEUP, INPUT_PULLUP);

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

    std::string com_addr;
    uint16_t com_port = 1234;
    int32_t com_timeout = 100;
    err = nvs_get_string(nvsHandle, NVS_KEY_COM_SERVER_ADDR, com_addr);
    switch (err) {
    case ESP_OK:
        err = nvs_get_u16(nvsHandle, NVS_KEY_COM_SERVER_PORT, &com_port);
        switch (err) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            print(Serial, "No COM server port set, fallback to {}.\n", com_port);
            break;
        default:
            print(Serial, "Error {} {} reading COM server port!\n", err, esp_err_to_name(err));
            print(Serial, "Fallback to {}.\n", com_port);
            break;
        }
        err = nvs_get_i32(nvsHandle, NVS_KEY_COM_SERVER_PORT, &com_timeout);
        switch (err) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            print(Serial, "No COM server timeout set, fallback to {}.\n", com_timeout);
            break;
        default:
            print(Serial, "Error {} {} reading COM server timeout!\n", err, esp_err_to_name(err));
            print(Serial, "Fallback to {}.\n", com_timeout);
            break;
        }
        print(Serial, "COM server: {}:{} (timeout {}).\n", com_addr, com_port, com_timeout);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No COM server set\n");
        break;
    default:
        print(Serial, "Error {} {} reading COM server address!\n", err, esp_err_to_name(err));
        break;
    }

    int16_t level_zero = 4300;
    err = nvs_get_i16(nvsHandle, NVS_KEY_LEVEL_ZERO, &level_zero);
    switch (err) {
    case ESP_OK:
        print(Serial, "Zero level: {} mm.\n", level_zero);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No Zero level set, fallback to {} mm.\n", level_zero);
        break;
    default:
        print(Serial, "Error {} {} reading zero level!\n", err, esp_err_to_name(err));
        print(Serial, "Fallback to {} mm.\n", level_zero);
        break;
    }

    int16_t level_on = 1000;
    err = nvs_get_i16(nvsHandle, NVS_KEY_LEVEL_ZERO, &level_on);
    switch (err) {
    case ESP_OK:
        print(Serial, "On level: {} mm.\n", level_on);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No On level set, fallback to {} mm.\n", level_on);
        break;
    default:
        print(Serial, "Error {} {} reading on level!\n", err, esp_err_to_name(err));
        print(Serial, "Fallback to {} mm.\n", level_on);
        break;
    }
    uint32_t meas_timeout = 1000;
    err = nvs_get_u32(nvsHandle, NVS_KEY_MEAS_PERIOD, &meas_timeout);
    switch (err) {
    case ESP_OK:
        print(Serial, "On meas period: {} ms.\n", meas_timeout);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        print(Serial, "No meas period set, fallback to {} ms.\n", meas_timeout);
        break;
    default:
        print(Serial, "Error {} {} reading meas period!\n", err, esp_err_to_name(err));
        print(Serial, "Fallback to {} ms.\n", meas_timeout);
        break;
    }

    BasicOTA OTA;

    bool sd_card_present = false;
    if (digitalRead(PIN_SD_PRESENT) == LOW) {
        if (digitalRead(PIN_SD_LOCK) == HIGH) {
            print(Serial, "SD card LOCKED!\n");
        } else {
            pinMode(PIN_SD_D0, INPUT_PULLUP);
            pinMode(PIN_SD_D1, INPUT_PULLUP);
            pinMode(PIN_SD_D2, INPUT_PULLUP);
            pinMode(PIN_SD_D3, INPUT_PULLUP);
            pinMode(PIN_SD_CMD, INPUT_PULLUP);
            pinMode(PIN_SD_CLK, INPUT_PULLUP);
            wait(msec(10));
            if(!SD_MMC.begin()) {
                print(Serial, "Card Mount Failed\n");
            }
            uint8_t cardType = SD_MMC.cardType();
            sd_card_present = true;
            switch (cardType) {
            case CARD_NONE:
                print(Serial, "No SD_MMC card attached\n");
                sd_card_present = false;
                break;
            case CARD_MMC:
                print(Serial, "MMC card\n");
                break;
            case CARD_SD:
                print(Serial, "SD card\n");
                break;
            case CARD_SDHC:
                print(Serial, "SDHC card\n");
                break;
            default:
                print(Serial, "Unknown card {}\n", cardType);
                break;
            }
            if (sd_card_present) {
                uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
                print(Serial, "Card Size: {} MB\n", cardSize);
                listDir(Serial, SD_MMC, "/");
            }
        }
    } else {
        print(Serial, "No SD card inserted\n");
    }

    timeout blink(msec(500));
    timeout meas(msec(meas_timeout));
    timeout com_reconect(msec(2000));
    com_reconect.cancel();

    wl_status_t wifi_last_status = WiFi.status();

    WiFiServer user_server(USER_SERVER_PORT, 1);
    WiFiClient user_client;
    WiFiClient com_client;
    bool user_client_connected = false;
    bool com_client_connected = false;
    Stream* user_stream = &Serial;
    Stream* mock_user_stream = nullptr;

    int16_t raw_level = 0;
    int16_t level = 0;
    raw_level = utsMeas(uts);
    level = (raw_level < 0) ? LEVEL_ERROR : (level_zero - raw_level);
    print(Serial, "Level {} mm: ", level);
    if (level > level_on) {
        digitalWrite(PIN_RELAY, 0);
        print(Serial, "switching on\n");
    } else {
        print(Serial, "keep off\n");
    }
    bool manual = false;
    bool force_on = false;
    bool print_raw = false;

    const adc_unit_t adc_unit = ADC_UNIT_1;
    const adc_atten_t adc_atten = ADC_ATTEN_DB_11;
    const adc_bits_width_t adc_width = ADC_WIDTH_BIT_12;
    const adc1_channel_t vin_adc_channel = adc1_channel_t(digitalPinToAnalogChannel(PIN_PWR_CHECK));
    adc_check_efuse();
    adc1_config_width(adc_width);
    adc1_config_channel_atten(vin_adc_channel, adc_atten);
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(adc_unit, adc_atten, adc_width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    uint32_t adc_vin = 0;
    
    meas.force();
    
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
        for (int i = 0; i != NO_OF_SAMPLES; i++)
            adc_vin += adc1_get_raw(vin_adc_channel);
        adc_vin = esp_adc_cal_raw_to_voltage(adc_vin / NO_OF_SAMPLES, adc_chars);
        // if (com_client) {
        //     print(com_client, "p {:4}\n", adc_vin);
        // }
        if (meas) {
            meas.ack();
            raw_level = utsMeas(uts);
            if (print_raw)
                level = raw_level;
            else
                level = (raw_level < 0) ? LEVEL_ERROR : (level_zero - raw_level);
            //level = raw_level;

            const float temp = thermometer.readTemperature();
            const float humid = thermometer.readHumidity();

            display.setCursor(0, 0);
            print(display, "d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}\n", level, temp, humid);
            print(*user_stream, "d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}; p: {:4} mV\n", level, temp, humid, adc_vin);
            display.display();
            if (com_client) {
                print(com_client, "r: {}; d: {:4} mm; t: {:4.1f} °C; h: {:2.0f}; p: {:4} mV\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str(), level, temp, humid, adc_vin);
            }
            if (!print_raw) {
                if (level < 0 && digitalRead(PIN_RELAY) == HIGH && !force_on) {
                    digitalWrite(PIN_RELAY, 0);
                    print(*user_stream, "LOW LEVEL, swithing off!\n");
                    if (com_client) {
                        print(com_client, "OFF\n");
                    }
                } else if (level > level_on && digitalRead(PIN_RELAY) == LOW && !manual) {
                    digitalWrite(PIN_RELAY, 1);
                    print(*user_stream, "Enough level, swithing on.\n");
                    if (com_client) {
                        print(com_client, "ON\n");
                    }
                }
            }
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
                if (!com_client.connected()) {
                    if (!com_addr.empty()) {
                        print(Serial, "Connecting to COM server {}:{}...", com_addr, com_port);
                        if (com_client.connect(com_addr.c_str(), com_port, com_timeout)) {
                            com_client_connected = true;
                            com_reconect.cancel();
                            print(Serial, " connected\n");
                        } else {
                            com_client_connected = false;
                            com_reconect.restart();
                            print(Serial, " failed\n");
                        }
                    }
                }
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
        if (wifi_status == WL_CONNECTED && !com_addr.empty() && !com_client) {
            if (com_client_connected) {
                com_client_connected = false;
                com_reconect.restart();
                com_reconect.force();
            }
            if (com_reconect) {
                print(Serial, "Connecting to COM server {}:{}...", com_addr, com_port);
                if (com_client.connect(com_addr.c_str(), com_port, com_timeout)) {
                    com_client_connected = true;
                    com_reconect.cancel();
                    print(Serial, " connected\n");
                } else {
                    com_client_connected = false;
                    com_reconect.restart();
                    print(Serial, " failed\n");
                }
            }
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
        if (com_client.available()) {
            mock_user_stream = user_stream;
            user_stream = &com_client;
        }
        if (user_stream->available()) {
            char c = user_stream->read();
            switch (c) {
            case '\n':
            case '\r':
                user_stream->write('\n');
                break;
            case '1':
                digitalWrite(PIN_RELAY, 1);
                print(*user_stream, "Manual ON\n");
                manual = true;
                break;
            case '0':
                digitalWrite(PIN_RELAY, 0);
                print(*user_stream, "Manual OFF\n");
                manual = true;
                break;
            case 'a':
                manual = false;
                force_on = false;
                print_raw = false;
                print(*user_stream, "Regular mode\n");
                break;
            case 'O':
                force_on = true;
                print(*user_stream, "Force mode, be aware!\n");
                break;
            case 'r':
                print_raw = true;
                manual = true;
                digitalWrite(PIN_RELAY, 0);
                print(*user_stream, "Raw mode, be aware!\n");
                break;
            case 'R':
                print(*user_stream, "Reseting...\n");
                delay(msec(100));
                if (com_client)
                    com_client.stop();
                if (user_client)
                    user_client.stop();
                user_stream = &Serial;
                user_server.end();
                WiFi.disconnect();
                esp_restart();
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
            case 'z':
                print(*user_stream, "Zero level: {}; On level: {}; raw level: {}; meas_period: {}\n", level_zero, level_on, raw_level, meas.get_timeout()/1000);
                break;
            case 'Z':
                if (isInt16(read_string(*user_stream, "Enter new zero level: "), &level_zero)) {
                    err = nvs_set_i16(nvsHandle, NVS_KEY_LEVEL_ZERO, level_zero);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nSaving level zero failed, because {}\n", esp_err_to_name(err));
                        break;
                    }
                    err = nvs_commit(nvsHandle);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nNVS commit failed\n");
                    }
                } else {
                    print(*user_stream, "Invalid input\n");
                }
                break;
            case 'o':
                if (isInt16(read_string(*user_stream, "Enter new on level: "), &level_on)) {
                    err = nvs_set_i16(nvsHandle, NVS_KEY_LEVEL_ON, level_on);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nSaving on level failed, because {}\n", esp_err_to_name(err));
                        break;
                    }
                    err = nvs_commit(nvsHandle);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nNVS commit failed\n");
                    }
                } else {
                    print(*user_stream, "Invalid input\n");
                }
                break;
            case 'P': {
                    uint32_t new_timeout = meas.get_timeout() / 1000;
                    if (isUint32(read_string(*user_stream, "Enter new meas period [ms]: "), &new_timeout)) {
                        err = nvs_set_u32(nvsHandle, NVS_KEY_MEAS_PERIOD, new_timeout);
                        if (err != ESP_OK) {
                            print(*user_stream, "\nSaving meas period failed, because {}\n", esp_err_to_name(err));
                            break;
                        }
                        err = nvs_commit(nvsHandle);
                        if (err != ESP_OK) {
                            print(*user_stream, "\nNVS commit failed\n");
                        }
                        meas.set_timeout(msec(new_timeout));
                        meas.restart();
                    } else {
                        print(*user_stream, "Invalid input\n");
                        break;
                    }
                }
                break;
            case 'f':
                rtc.writeSqwPinMode(DS1307_ON);
                print(*user_stream, "RTC set\n");
                break;
            case 'C':
                com_addr = read_string(*user_stream, "Enter new COM server address: ");
                if (isUint16(read_string(*user_stream, "Enter new COM server port: "), &com_port)) {
                    err = nvs_set_u16(nvsHandle, NVS_KEY_COM_SERVER_PORT, com_port);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nSaving COM port failed, because {}\n", esp_err_to_name(err));
                        break;
                    }
                    err = nvs_set_str(nvsHandle, NVS_KEY_COM_SERVER_ADDR, com_addr.c_str());
                    if (err != ESP_OK) {
                        print(*user_stream, "\nSaving COM address failed, because {}\n", esp_err_to_name(err));
                        break;
                    }
                    err = nvs_commit(nvsHandle);
                    if (err != ESP_OK) {
                        print(*user_stream, "\nNVS commit failed\n");
                    }
                } else {
                    print(*user_stream, "Invalid input\n");
                }
                if (com_client) {
                    print(*user_stream, "Disconnecting old COM client.\n");
                    com_client.stop();
                    com_client_connected = false;
                }
                com_reconect.restart();
                com_reconect.force();
                break;
            }
        }
        if (mock_user_stream) {
            user_stream = mock_user_stream;
            mock_user_stream = nullptr;
        }
        OTA.handle();
    }
}

void loop() {
}
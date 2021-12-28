idf_component_register(
    SRCS "RTClib/src/RTC_PCF8523.cpp"
         "RTClib/src/RTC_PCF8563.cpp"
         "RTClib/src/RTC_DS1307.cpp"
         "RTClib/src/RTC_DS3231.cpp"
         "RTClib/src/RTC_Millis.cpp"
         "RTClib/src/RTC_Micros.cpp"
         "RTClib/src/RTClib.cpp"
    INCLUDE_DIRS "RTClib/src"
    REQUIRES Adafruit_BusIO
)
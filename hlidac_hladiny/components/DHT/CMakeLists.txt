idf_component_register(
    SRCS "Sensor/Adafruit_Sensor.cpp"
         "DHT/DHT.cpp"
         "DHT/DHT_U.cpp"
    INCLUDE_DIRS "Sensor"
                 "DHT"
    REQUIRES Adafruit_BusIO
)
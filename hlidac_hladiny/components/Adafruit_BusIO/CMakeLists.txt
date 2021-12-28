idf_component_register(
    SRCS "Adafruit_BusIO/Adafruit_BusIO_Register.cpp"
         "Adafruit_BusIO/Adafruit_I2CDevice.cpp"
         "Adafruit_BusIO/Adafruit_SPIDevice.cpp"
    INCLUDE_DIRS "Adafruit_BusIO"
    REQUIRES arduino
)
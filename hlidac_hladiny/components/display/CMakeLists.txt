idf_component_register(
    SRCS "Adafruit-GFX-Library/Adafruit_GFX.cpp"
         "Adafruit-GFX-Library/Adafruit_GrayOLED.cpp"
         "Adafruit-GFX-Library/Adafruit_SPITFT.cpp"
         "Adafruit-GFX-Library/glcdfont.c"
         "Adafruit_SSD1306/Adafruit_SSD1306.cpp"
    INCLUDE_DIRS "Adafruit-GFX-Library"
                 "Adafruit_SSD1306"
    REQUIRES Adafruit_BusIO
)
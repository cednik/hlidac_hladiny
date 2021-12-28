#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_ADDR 0x3C

Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

namespace Display {

bool isConnected = false;

void init() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR, true, false)) { // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        print("Display initialization failed\n");
    } else {
        isConnected = true;

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
        // delay(500);

        // display.clearDisplay();
        // display.display();
        // display.setTextSize(2);
        // display.setCursor(0, 0);
    }
}

} // namespace Display

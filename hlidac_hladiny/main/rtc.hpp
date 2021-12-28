#pragma once

#include <RTClib.h>

RTC_DS1307 rtc;

namespace RTC {

bool isConnected = false;

void init() {
    if (rtc.begin(&Wire)) {
        isConnected = true;
        if (rtc.isrunning()) {
            print("RTC time: {}\n", rtc.now().timestamp(DateTime::TIMESTAMP_FULL).c_str());
        } else {
            print("RTC is not running!\n");
        }
    } else {
        isConnected = false;
        print("RTC not connected!\n");
    }
}

} // namespace RTC

#pragma once

/* pinout:
 0 RTC_WAKEUP | BOOT/U
 1 TXD0
 2 SD_DATA0 | BOOT/D
 3 RXD0
 4 SD_DATA1
 5 ILED     | SDIO/U
 6 FLASH
 7 FLASH
 8 FLASH
 9 FLASH
10 FLASH
11 FLASH
12 SD_DATA2 | VDD_SDIO/D
13 SD_DATA3
14 SD_CLK
15 SD_CMD   | LOG|SDIO/U 
16 SDA
17 SCL
18 TRIG
19 DHT11
21 ECHO
22 RELAY
23 BUZZER
25 ENC_A
26 ENC_SW
27 LED
32 PWR_CHECK
33 ENC_B
34 CARD_LOCK
35 CARD_DET
36 NC
39 NC
*/

#define PIN_TRIG       18
#define PIN_ECHO       21

#define PIN_DHT11      19

#define PIN_SDA        16
#define PIN_SCL        17

#define PIN_ILED        5
#define PIN_LED        27
#define PIN_BUZZER     23

#define PIN_ENC_A      25
#define PIN_ENC_B      33
#define PIN_ENC_SW     26

#define PIN_RELAY      22

#define PIN_RTC_WAKEUP  0

#define PIN_PWR_CHECK  32

#define PIN_SD_D0       2
#define PIN_SD_D1       4
#define PIN_SD_D2      12
#define PIN_SD_D3      13
#define PIN_SD_CMD     15
#define PIN_SD_CLK     14

#define PIN_SD_PRESENT 35
#define PIN_SD_LOCK    34

#define PIN_UNUSED0    36
#define PIN_UNUSED1    39

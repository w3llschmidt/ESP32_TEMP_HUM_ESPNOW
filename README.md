# ESP32_TEMP_HUM_ESPNOW

    Sender für **SparkFun Qwiic Pocket Dev Board – ESP32-C6** + **Adafruit SHT45** (I²C/Qwiic).  
    Überträgt Messwerte per **ESP-NOW** im **25-Byte-Unified-Frame** als Broadcast.

## Hardware

    Board: ESP32-C6 (Qwiic: **SDA=IO6**, **SCL=IO7**)
    https://www.sparkfun.com/sparkfun-qwiic-pocket-development-board-esp32-c6.html
    
    Sensor: SHT45 @ **0x44** (Qwiic/STEMMA QT) 
    https://www.adafruit.com/product/5665

## Frame (25B)

    dev_id (u32 FNV-1a der STA-MAC)
    svc (4B) = "TMP "
    dbm (i8) = 0 (senderseitig unbekannt)
    f1..f4 (float)

    - **F1 = Temperatur [°C]**
    - **F2 = rel. Feuchte [%]**
    - **F3 = 0.0**
    - **F4 = 0.0**

## Build & Flash (ESP-IDF)

    # Einmalig (falls nötig)
    idf.py set-target esp32c6

    # Build & Flash
    idf.py -p /dev/ttyUSB0 flash monitor
    # Monitor verlassen: Ctrl+]

    WLAN / Kanal

        Fixer Kanal 1, Country DE (1..13), 20 MHz, kein PS.
        Empfänger muss denselben Kanal nutzen.

    Verkabelung

        Qwiic-Verbinder nutzen (onboard Pull-ups), keine externen Pull-ups nötig.

## Lizenz

Copyright (c) 2025 w3llschmidt@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
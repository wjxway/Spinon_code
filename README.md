# Spinon_code
Code for Spinon. Please check https://github.com/wjxway/Spinon for more details.

## Warning
This code base is compatible with ESP32 arduino release v2.0.6 and ESP-IDF v4.4.3
Due to direct write of memory in RMT related code, this code is not compatible with the main interface of ESP-IDF v5.0.0 (though still compatible because v5.0.0 kept the deprecated interfaces)

## Note
To get as many GPIO as possible, we fixed the interal LDO's voltage at 3.3V and freed up MTDI pin (GPIO 12). (The operating voltage of ESP32-PICO-D4’s integrated external SPI flash is 3.3 V. Therefore, the strapping pin MTDI should hold bit ”0” during the module power-on reset. So it's completely fine to do this.)

To get the board to work correctly, you have to use espefuse.py to burn the fuse and solidify the voltage.

```
    espefuse.exe --port COMxxx set_flash_voltage 3.3V
```

And then the board will function as expected.
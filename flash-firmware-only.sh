#!/bin/bash

#esptool.py -p /dev/ttyUSB0 -b $BAUDRATE --before default_reset --after hard_reset --chip esp32s3 write_flash --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 build/esp32_mouse_keyboard.bin
BAUDRATE=2000000
esptool.py -b $BAUDRATE --before default_reset --after hard_reset --chip esp32s3 write_flash --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 build/esp32_mouse_keyboard.bin


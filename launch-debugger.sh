#!/bin/bash

openocd -c 'set ESP_RTOS none' -f board/esp32s3-builtin.cfg


#ifndef _CONFIG_H_
#define _CONFIG_H_

#define MODULE_ID "ESP32miniBT_v0.4.0"

#if CONFIG_USE_AS_FLIPMOUSE_FABI
  //this will be overwritten by FABI/FLipMouse/FLipPad firmware to correct
  //name, this is the BT device name for the first start
  //09.2026: we use this project for FM3 mostly (FABI is Pico2W, FLipPad should overwrite), set this name to FM per default
  #define GATTS_TAG "FM3"
//non FM/FP/FABI boards
#else
	#define EX_UART_NUM     	 CONFIG_MODULE_UART_NR
	#define GATTS_TAG "esp32_mouse_keyboard"
#endif

#define MAX_BT_DEVICENAME_LENGTH 40

// serial port of monitor and for debugging (not in KConfig, won't be changed normally)
#define CONSOLE_UART_NUM 	 UART_NUM_0

typedef struct config_data {
    char bt_device_name[MAX_BT_DEVICENAME_LENGTH];
    uint8_t locale;
    uint8_t joystick_active;
} config_data_t;


#endif

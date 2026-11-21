/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Copyright 2023:
 * Benjamin Aigner <beni@asterics-foundation.org>,<aignerb@technikum-wien.at>
 *
 * Copyright 2020-2021:
 * Benjamin Aigner <beni@asterics-foundation.org>,<aignerb@technikum-wien.at>,
 * Junaid Khan <junaid.khan.wien@gmail.com>
 *
 * This file is mostly based on the Espressif ESP32 BLE HID example.
 * Adaption were made for:
 * * UART interface
 * * console input for testing purposes
 * * Joystick support (replacing vendor report)
 * * command input via UART for controlling the BLE interface (get & delete pairings,...)
 * * Firmware update possible via UART (factory partition required)
 *
 */

/* Original license text:
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "cli.h"
#include "config.h"
#include "mousewheel.h"
#include "pinout.h"
#include "pwr.h"
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_event.h>
#include <esp_flash.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>
#include <esp_hidd_prf_api.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <hid_dev.h>
#include <nvs_flash.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 2. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"


#define EXT_UART_TAG "EXT_UART"

/** @warning Currently (07.2020) whitelisting devices is still not possible for all devices,
 * because many BT devices use resolvable random addresses, this seems to unsupported:
 * https://github.com/espressif/esp-idf/issues/1368
 * https://github.com/espressif/esp-idf/issues/2262
 * Therefore, if we enable pairing only on request, it is not possible to connect
 * to the ESP32 anymore. Either the ESP32 is visible by all devices or none.
 *
 * To circumvent any problems with this repository, if the config for
 * disabled pairing by default is active, we throw an error here.
 * @todo Check regularily for updates on the above mentioned issues.
 **/
#if CONFIG_MODULE_BT_PAIRING
#error "Sorry, currently the BT controller of the ESP32 does NOT support whitelisting. Please deactivate the pairing on demand option in make menuconfig!"
#endif

/** @brief NVS handle to resolve BT addr to name
 *
 * In NVS, we store the BT addr as key and the name as value. */
nvs_handle nvs_bt_name_h;

/** @brief NVS handle to store key/value pairs via UART
 *
 * In NVS, we store arbitrary values, which are sent via UART.
 * This can(will) be used for storing different values from the
 * GUI on the ESP32. */
nvs_handle nvs_storage_h;

/** Timestamp of last sent HID packet, used for idle sending timer callback
 * @see periodicHIDCallback */
uint64_t timestampLastSent;

/** Last mouse button state.
 * Due to the absolute values for mouse buttons, we need to keep track.
 * Because when the mouse button is pressed and hold (without steady movement),
 * the idle callback will send an empty report (X/Y/wheel are 0), but
 * the mousebuttons must be the same
 * @see periodicHIDCallback */
uint8_t mouseButtons = 0;

/** Flag for hardware which this firmware is running on.
 * If we are running this software on the Arudino RP2040 connect,
 * we need to use different pins, also another UART (including pins).
 * This flag is set immediately after startup in app_main and is set
 * based on the flash size. All modules we used were with >=4MB flash,
 * the uBlox NINA module on the Arduino RP2040 connect uses 2MB. */
bool onArduinoRP2040 = false;

/** pin number for indicator LED
 * Was also a KConfig define for all available builds, now it is fixed
 * for FABI/FM/FP builds depending on Arduino or addon board.
 * For other modules or custom builds, it can be defined via idf.py menuconfig. */
int indicator_led = -1;

/** "Keepalive" rate when in idle (no HID commands)
 * @note Microseconds!
 * @see timestampLastSent
 * @see periodicHIDCallback */
#define HID_IDLE_UPDATE_RATE 200000


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

config_data_t config;

//a list of active HID connections.
//conn_id array stores the connection ID, if unused it is -1
//active_connections stores the BT mac address
int16_t active_hid_conn_ids[CONFIG_BT_ACL_CONNECTIONS];
esp_bd_addr_t active_connections[CONFIG_BT_ACL_CONNECTIONS] = {0};
//this HID connection is used, if only ONE device should receive the HID data.
//currently, the $SW command to select the HID device to be controlled it is
//set to -1 (send to all devices). If it is != -1, we will send it to only this device.
///@note This works only for the UART interface (uart_parse_command).
int16_t hid_conn_id = -1;

static uint8_t manufacturer[19]= {'A', 's', 'T', 'e', 'R', 'I', 'C', 'S', ' ', 'F', 'o', 'u', 'n', 'd', 'a', 't', 'i', 'o', 'n'};


static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

/** @brief Event bit, set if pairing is enabled
 * @note If MODULE_BT_PAIRING ist set in menuconfig, this bit is disable by default
 * and can be enabled via $PM1 , disabled via $PM0.
 * If MODULE_BT_PAIRING is not set, this bit will be set on boot.*/
#define SYSTEM_PAIRING_ENABLED (1<<0)

/** @brief Event bit, set if the ESP32 is currently advertising.
 *
 * Used for determining if we need to set advertising params again,
 * when the pairing mode is changed. */
#define SYSTEM_CURRENTLY_ADVERTISING (1<<1)

/** @brief Event group for system status */
EventGroupHandle_t eventgroup_system;

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    //it's always that crappy iOS stuff:
    //09.2026->include name & include TX power set to false, otherwise it is not found at all (but works on every other platform)
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x000A, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    /** @note: HID generic appearance does not work on some iOS/Amazon FireTV Sticks, see https://github.com/asterics/esp32_mouse_keyboard/issues/52 */
    .appearance = 0x03c2,       //HID Mouse (keyboard: 0x03c1)
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_scan_params_t scan_params = {
	.scan_type = BLE_SCAN_TYPE_ACTIVE,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
	.scan_interval = 0x00A0, /* 100ms (n*0.625ms)*/
	.scan_window = 0x0090, /* 90ms */
	.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

// config scan response data
///@todo Scan response is currently not used. If used, add state handling (adv start) according to ble/gatt_security_server example of Espressif
static esp_ble_adv_data_t hidd_adv_resp = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(manufacturer),
    .p_manufacturer_data = manufacturer,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void on_thumbwheel(int32_t offset);

uint8_t uppercase(uint8_t c)
{
    if ((c>='a') && (c<='z')) return (c-'a'+'A');
    return(c);
}

/** Periodic sending of empty HID reports if no updates are sent via API */
static void periodicHIDCallback(void* arg)
{
	if((esp_timer_get_time()-timestampLastSent) > HID_IDLE_UPDATE_RATE)
	{
		//send empty report (but with last known button state)
		for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
		{
			if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],mouseButtons,0,0,0);
		}
		//save timestamp for next call
		timestampLastSent = esp_timer_get_time();
		ESP_LOGD(HID_DEMO_TAG,"Idle...");
	}
}

/**
 * Determine if there is at least one device connected.
 */
int isConnected()
{
	//determine if there is at least one device connected.
	for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
	{
		if(active_hid_conn_ids[i] != -1) return 1;
	}
	return 0;
}

void printConnectedDevicesTable()
{
	for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
	{
		ESP_LOGI(HID_DEMO_TAG, "%d: HID connection ID: %d",i,active_hid_conn_ids[i]);
		ESP_LOGI(HID_DEMO_TAG, "%d: remote BD_ADDR: %08x%04x",i,
		 (active_connections[i][0] << 24) + (active_connections[i][1] << 16) + (active_connections[i][2] << 8) + active_connections[i][3],
		 (active_connections[i][4] << 8) + active_connections[i][5]);
	 }
}


void update_config(void)
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(err != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    err = nvs_set_str(my_handle, "btname", config.bt_device_name);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - bt name");
    err = nvs_set_u8(my_handle, "locale", config.locale);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - locale");
    err = nvs_set_u8(my_handle, "joyactive", config.joystick_active);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - joystick state");
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    nvs_close(my_handle);
}


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(config.bt_device_name);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG: {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
        esp_err_t err = ESP_OK;
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");

        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS;i++)
        {
			if(active_hid_conn_ids[i] == -1) //search for the first unused slot
			{
				memcpy(active_connections[i], param->connect.remote_bda, sizeof(esp_bd_addr_t));
				active_hid_conn_ids[i] = param->connect.conn_id;
				ESP_LOGI(HID_DEMO_TAG, "Added connection: %d @ %d",active_hid_conn_ids[i],i);
				break;
			}
		}

		//because some devices do connect with a quite high connection
		//interval, we might have a congested channel...
		//to overcome this issue, we update the connection parameters here
		//to use a very low connection interval.
		esp_ble_conn_update_params_t new;
		memcpy(new.bda,param->connect.remote_bda,sizeof(esp_bd_addr_t));
		new.min_int = 80;
		new.max_int = 80;
		new.latency = 0;
		new.timeout = 800;
		err = esp_ble_gap_update_conn_params(&new);
        if (err != ESP_OK)
        {
            ESP_LOGE(HID_DEMO_TAG, "Failed to set BLE GAP params, err=%d", err);
        }

        // We don't need connections anymore. Stop advertising
        esp_ble_gap_stop_advertising();

        //xEventGroupClearBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
		for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
		{
			//check if this addr is in the array
			if(memcmp(active_connections[i],param->disconnect.remote_bda,sizeof(esp_bd_addr_t)) == 0)
			{
				//clear element
				ESP_LOGI(HID_DEMO_TAG, "Removed connection: %d @ %d",active_hid_conn_ids[i],i);
				memset(active_connections[i],0,sizeof(esp_bd_addr_t));
				active_hid_conn_ids[i] = -1;
				break;
			}
		}

        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        xEventGroupSetBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
        break;
    }
    /**
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
        ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_HIDD_EVENT_BLE_LED_OUT_WRITE_EVT: {
        ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_LED_OUT_WRITE_EVT, keyboard LED value: %d", __func__, param->vendor_write.data[0]);
        break;
    }
    */

    case ESP_HIDD_EVENT_BLE_CONGEST: {
		if(param->congest.congested)
		{
			ESP_LOGI(HID_DEMO_TAG, "Congest: %d, conn: %d",param->congest.congested,param->congest.conn_id);
			esp_gap_conn_params_t current;
			esp_ble_get_current_conn_params(active_connections[param->congest.conn_id],&current);
			ESP_LOGI(HID_DEMO_TAG, "Interval: %d, latency: %d, timeout: %d",current.interval, current.latency, current.timeout);
		}
		break;
	}
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        xEventGroupSetBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		if(esp_ble_gap_start_scanning(3600) != ESP_OK) ESP_LOGW(HID_DEMO_TAG,"Cannot start scan");
		else ESP_LOGI(HID_DEMO_TAG,"Start scan");
		break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
		{
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGW(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            xEventGroupClearBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
        }
#if CONFIG_MODULE_BT_PAIRING
        //add connected device to whitelist (necessary if whitelist connections only).
        if(esp_ble_gap_update_whitelist(true,bd_addr,BLE_WL_ADDR_TYPE_PUBLIC) != ESP_OK)
        {
            ESP_LOGW(HID_DEMO_TAG,"cannot add device to whitelist, with public address");
        } else {
            ESP_LOGI(HID_DEMO_TAG,"added device to whitelist");
        }
        if(esp_ble_gap_update_whitelist(true,bd_addr,BLE_WL_ADDR_TYPE_RANDOM) != ESP_OK)
        {
            ESP_LOGW(HID_DEMO_TAG,"cannot add device to whitelist, with random address");
        }
#endif
		}
        break;
    //handle scan responses here...
	case ESP_GAP_BLE_SCAN_RESULT_EVT: {
		uint8_t *adv_name = NULL;
	    uint8_t adv_name_len = 0;
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
			case ESP_GAP_SEARCH_INQ_RES_EVT:
				//ESP_LOG_BUFFER_HEX_LEVEL(HID_DEMO_TAG, scan_result->scan_rst.bda, 6, ESP_LOG_INFO);
				//ESP_LOGI(HID_DEMO_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
				adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
				if(adv_name_len == 0) adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
				//ESP_LOGI(HID_DEMO_TAG, "Searched Device Name Len %d", adv_name_len);
				//esp_log_buffer_char(HID_DEMO_TAG, adv_name, adv_name_len);
				//ESP_LOGI(HID_DEMO_TAG, "\n");
				if (adv_name != NULL) {
					//store name to BT addr...
					ESP_LOG_BUFFER_HEX_LEVEL(HID_DEMO_TAG, scan_result->scan_rst.bda, 6, ESP_LOG_DEBUG);
					ESP_LOG_BUFFER_HEX_LEVEL(HID_DEMO_TAG, adv_name, adv_name_len, ESP_LOG_DEBUG);
					adv_name[adv_name_len] = '\0';
					char key[13];
					sprintf(key,"%02X%02X%02X%02X%02X%02X",scan_result->scan_rst.bda[0],scan_result->scan_rst.bda[1], \
						scan_result->scan_rst.bda[2],scan_result->scan_rst.bda[3],scan_result->scan_rst.bda[4],scan_result->scan_rst.bda[5]);
					if(nvs_set_str(nvs_bt_name_h,key,(char*)adv_name) == ESP_OK)
					{
						ESP_LOGI(HID_DEMO_TAG,"Saved %s to %s",adv_name, key);
					} else ESP_LOGW(HID_DEMO_TAG,"Error saving %s for %s",adv_name,key);
				}
				break;
			case ESP_GAP_SEARCH_INQ_CMPL_EVT:
				break;
			default:
				break;
        }

		}
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(HID_DEMO_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(HID_DEMO_TAG, "Scan start success");
        break;

    default:
        break;
    }
}


//#if CONFIG_MODULE_NANO

int uart_vprintf_valid = 0;
vprintf_like_t uart0_vprintf;
/** a vprintf discard function. used to disable logging */
int vprintf_discard(const char *str, va_list l)
{
	(void)str;
	(void)l;
	return 0;
}
//#endif

void on_thumbwheel(int32_t offset)
{
    const uint8_t val = (int8_t)(offset > 127 ? 127 :
        offset < -127 ? -127 :
        offset);
    if(hid_conn_id == -1)
    {
        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
        {
            if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i], 0, 0, 0, val);
        }
    } else {
        esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, val);
    }
    //update timestamp
    timestampLastSent = esp_timer_get_time();
}

static void main_loop(void)
{
    const int32_t val = mousewheel_loop();
    if (val)
    {
        on_thumbwheel(val);
    }
}

void app_main(void)
{
    esp_err_t ret;

    pwr_init();

    // Initialize FreeRTOS elements
    eventgroup_system = xEventGroupCreate();
    if(eventgroup_system == NULL) ESP_LOGE(HID_DEMO_TAG, "Cannot initialize event group");
    //if set in KConfig, pairing is disable by default.
    //User has to enable pairing with $PM1
#if CONFIG_MODULE_BT_PAIRING
    ESP_LOGI(HID_DEMO_TAG,"pairing disabled by default");
    xEventGroupClearBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
    hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
#else
    ESP_LOGI(HID_DEMO_TAG,"pairing enabled by default");
    xEventGroupSetBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
    hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
#endif

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    /* Set BLE transmit power for various cases (`_N0` means 0 dBm)*/
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N0);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    //open NVS handle for storing BT device names
    ESP_LOGI("MAIN","opening NVS handle for BT names");
    ret = nvs_open("btnames", NVS_READWRITE, &nvs_bt_name_h);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS for bt names");

    //open NVS handle for key/value storage via UART
    ESP_LOGI("MAIN","opening NVS handle for key/value storage");
    ret = nvs_open("kvstorage", NVS_READWRITE, &nvs_storage_h);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS for key/value storage");

    //read the appearance value for advertising
    uint8_t advapp;
    ret = nvs_get_u8(nvs_storage_h,"BLEAPPEAR",&advapp);
    if(ret == ESP_OK)
    {
      ESP_LOGI("MAIN","Setting appearance to 0x03C%d",advapp);
      hidd_adv_data.appearance = 0x03C0 + advapp;
    }

    // Read config
    nvs_handle my_handle;
    ESP_LOGI("MAIN","loading configuration from NVS");
    ret = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    size_t available_size = MAX_BT_DEVICENAME_LENGTH;
    strcpy(config.bt_device_name, GATTS_TAG);
    nvs_get_str (my_handle, "btname", config.bt_device_name, &available_size);
    if(ret != ESP_OK)
    {
        ESP_LOGI("MAIN","error reading NVS - bt name, setting to default");
        strcpy(config.bt_device_name, GATTS_TAG);
    } else ESP_LOGI("MAIN","bt device name is: %s",config.bt_device_name);

    //get from NVS if the joystick should be registered
    #if CONFIG_MODULE_USEJOYSTICK
    config.joystick_active = 0;
    nvs_get_u8(my_handle, "joyactive", &config.joystick_active);
    ESP_LOGI("MAIN","Joystick: %d",config.joystick_active);
    #endif

    //get locale
    ret = nvs_get_u8(my_handle, "locale", &config.locale);
    //if(ret != ESP_OK || config.locale >= LAYOUT_MAX)
    ///@todo implement keyboard layouts.
    if(ret != ESP_OK)
    {
        ESP_LOGI("MAIN","error reading NVS - locale, setting to US_INTERNATIONAL");
        //config.locale = LAYOUT_US_INTERNATIONAL;
    } else ESP_LOGI("MAIN","locale code is : %d",config.locale);
    nvs_close(my_handle);
    ///@todo How to handle the locale here? We have the memory for full lookups on the ESP32, but how to communicate this with the Teensy?

    ///clear the HID connection IDs&MACs
    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS;i++)
    {
      active_hid_conn_ids[i] = -1;
      memset(active_connections[i],0,sizeof(esp_bd_addr_t));
    }
    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback,config.joystick_active);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    //start periodic timer to send HID reports
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodicHIDCallback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "HIDidle"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    //call every 100ms
    esp_timer_start_periodic(periodic_timer, 100000);

    mousewheel_init();
    cli_init();

    while (1)
    {
        main_loop();
        pwr_loop();
        vTaskDelay(5);
    }

    //avoid unused variable warnings here:
    (void)hidd_adv_resp;
    (void)scan_params;
}

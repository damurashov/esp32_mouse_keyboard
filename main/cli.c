/**
 * cli.c
 *
 * Created on: November 21, 2026
 *     Author: Dmitry Murashov
 */


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXT_UART_TAG "EXT_UART"
#define CONSOLE_UART_TAG "CONSOLE_UART"
#define HID_DEMO_TAG "HID_DEMO"

#define CMDSTATE_IDLE 0
#define CMDSTATE_GET_RAW 1
#define CMDSTATE_GET_ASCII 2

#define MAX_CMDLEN  100

/****************************************************************************
 * Included files
 ****************************************************************************/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

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

#include "config.h"
#include <sdkconfig.h>
#include <stdint.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cmdBuf {
	//current state of the parser, CMD_STATE*
    int state;
    //if a fixed length command is issued, we store expected length here
    int expectedBytes;
    int bufferLength;
    //if != 0, the result of a command will be sent to the debug console AND the external UART (-> FLipMouse/FABI GUI on PC)
    int sendToUART;
    uint8_t buf[MAX_CMDLEN];
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/** 2nd UART unit number for RX/TX of commands
 * Previously we used a define via KConfig to set the UART number.
 * Because we need to detect on the fly if we are on an Arduino RP2040 Connect
 * or on a Bluetooth addon, this value is set in a variable now. */
int ext_uart_num = 0;

/** 2nd UART unit RX pin */
int ext_uart_rx = -1;

/** 2nd UART unit TX pin */
int ext_uart_tx = -1;

extern nvs_handle nvs_storage_h;
extern uint8_t mouseButtons;
extern int16_t active_hid_conn_ids[CONFIG_BT_ACL_CONNECTIONS];
extern uint64_t timestampLastSent;
extern int16_t hid_conn_id;
extern config_data_t config;
extern esp_bd_addr_t active_connections[CONFIG_BT_ACL_CONNECTIONS];
extern nvs_handle nvs_bt_name_h;

extern void on_thumbwheel(int32_t offset);
extern void update_config(void);
extern int isConnected();

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int get_int(const char * input, int index, int * value)
{
    int sign=1, result=0, valid=0;

    while (input[index]==' ') index++;   // skip leading spaces
    if (input[index]=='-') {
        sign=-1;
        index++;
    }
    while ((input[index]>='0') && (input[index]<='9'))
    {
        result= result*10+input[index]-'0';
        valid=1;
        index++;
    }
    while (input[index]==' ') index++;  // skip trailing spaces
    if (input[index]==',') index++;     // or a comma

    if (valid) {
        *value = result*sign;
        return (index);
    }
    return(0);
}


void processCommand(struct cmdBuf *cmdBuffer)
{
  //commands:
  // $ID
  // $PMx (0 or 1)
  // $GP
  // $DPx (number of paired device, starting with 0)
  // $SW aabbccddeeff (select a BT addr to send the HID commands to)
  // $GC get connected devices
  // $NAME set name of bluetooth device
  // $JPx (0,1) en- / disable the joystick interface (for iOS compatibility) [available if compiled with Joystick support]
  // $APx (0-4) Set the appearance value for advertising (0x03C0 - 0x03C4; default is mouse). See https://specificationrefs.bluetooth.com/assigned-values/Appearance%20Values.pdf page 8
  // $GV <key>  get the value of the given key from NVS. Note: no spaces in <key>! max. key length: 15
  // $SV <key> <value> set the value of the given key & store to NVS. Note: no spaces in <key>!
  // $CV clear all key/value pairs set with $SV
  // $UG start flash update by searching for factory partition and rebooting there. Warning: not possible to boot back without flashing!
  // $LGx (0,1,2): enable / disable logging system of ESP32.0 is level error, 1 is level info, 2 is level debug

  if(cmdBuffer->bufferLength < 2) return;
  //easier this way than typecast in each str* function
  const char *input = (const char *) cmdBuffer->buf;
  int len = cmdBuffer->bufferLength;
  const char *nl = "\r\n";
  esp_ble_bond_dev_t * btdevlist;
  int counter;
  esp_err_t ret;

  #if CONFIG_MODULE_USEJOYSTICK
  /**++++ (de-)activate joystick ++++*/
  if(strncmp(input,"JP",2) == 0)
  {
    uint8_t joystate = input[2] - '0';
    if(joystate) config.joystick_active = 1;
    else config.joystick_active = 0;
    update_config();
    ESP_LOGI(EXT_UART_TAG,"new joystick state: %d, will show on next reboot", config.joystick_active);
    if(cmdBuffer->sendToUART != 0)
    {
      uart_write_bytes(ext_uart_num, "JS:",strlen("JS:"));
      if(joystate) uart_write_bytes(ext_uart_num, "1",1);
      else uart_write_bytes(ext_uart_num, "0",1);
      uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
    }
    return;
  }
  #endif

  /**++++ set BLE appearance ++++*/
  if(strncmp(input,"AP", 2) == 0)
  {
    uint8_t appv = input[2] - '0';
    if(appv <= 4)
    {
      ESP_LOGI(EXT_UART_TAG,"setting appearance to NVS, will show on next reboot");
      nvs_set_u8(nvs_storage_h,"BLEAPPEAR",appv);
			nvs_commit(nvs_storage_h);
      if(cmdBuffer->sendToUART != 0)
      {
        uart_write_bytes(ext_uart_num, "AP:",strlen("AP:"));
        uart_write_bytes(ext_uart_num, &input[2],1);
        uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
      }
    } else {
      ESP_LOGE(EXT_UART_TAG,"Cannot set appearance, value not correct. Use AP0 - AP4");
      if(cmdBuffer->sendToUART != 0)
      {
        uart_write_bytes(ext_uart_num, "AP:invalid number, AP0-AP4",strlen("AP:invalid number, AP0-AP4"));
        uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
      }
    }
    return;
  }

	/**++++en-/disable logging++++*/
	if(strcmp(input,"LG0") == 0)
    {
		esp_log_level_set("*",ESP_LOG_ERROR);
		if(cmdBuffer->sendToUART != 0)
		{
			uart_write_bytes(ext_uart_num, "LOG:0",strlen("LOG:0"));
			uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
		}
		return;
	}
	if(strcmp(input,"LG1") == 0)
    {
		esp_log_level_set("*",ESP_LOG_INFO);
		if(cmdBuffer->sendToUART != 0)
		{
			uart_write_bytes(ext_uart_num, "LOG:1",strlen("LOG:1"));
			uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
		}
		return;
	}
	if(strcmp(input,"LG2") == 0)
    {
		esp_log_level_set("*",ESP_LOG_DEBUG);
		if(cmdBuffer->sendToUART != 0)
		{
			uart_write_bytes(ext_uart_num, "LOG:2",strlen("LOG:2"));
			uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
		}
		return;
	}

	/**++++ key/value storing ++++*/
	if(strncmp(input,"CV ", 2) == 0)
	{
		//no error checks here, because all errors
		//are related to the NVS part, which cannot be fixed via
		//the UART console
		nvs_erase_all(nvs_storage_h);
		//commit NVS storage
		ret = nvs_commit(nvs_storage_h);
		ESP_LOGI(EXT_UART_TAG,"cleared all NVS key/value pairs");
		if(cmdBuffer->sendToUART != 0)
		{
			uart_write_bytes(ext_uart_num, "NVS:OK",strlen("NVS:OK"));
			uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
		}
		return;
	}
	if(strncmp(input,"GV ", 3) == 0)
	{
		char* work = (char*)cmdBuffer->buf;
		//remove GV command name
		strsep(&work, " ");
		//get key
		char *key = strsep(&work, " ");

		//get data size from NVS, check if key is set
		size_t sizeData;
		char* nvspayload = NULL;
		ret = nvs_get_str(nvs_storage_h, key,NULL,&sizeData);

		//if we have a data length, load string
		if(ret == ESP_OK)
		{
			//load str data
			nvspayload = malloc(sizeData);
			ret = nvs_get_str(nvs_storage_h, key, nvspayload, &sizeData);
		}

		//OK or error?
		if(ret != ESP_OK)
		{
			//send back error message
			ESP_LOGI(EXT_UART_TAG,"error reading value: %s",esp_err_to_name(ret));
			if(cmdBuffer->sendToUART != 0)
			{
				uart_write_bytes(ext_uart_num, "NVS:",strlen("NVS:"));
				uart_write_bytes(ext_uart_num, esp_err_to_name(ret), strlen(esp_err_to_name(ret)));
				uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
			}
		} else {
			ESP_LOGI(EXT_UART_TAG,"loaded - %s:%s",key,nvspayload);
			uart_write_bytes(ext_uart_num, "NVS:",strlen("NVS:"));
			uart_write_bytes(ext_uart_num, nvspayload, strlen(nvspayload));
			uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
		}

		//done with the payload
		if(nvspayload) free(nvspayload);
		return;
	}

	if(strncmp(input,"SV ", 3) == 0)
	{
		char* work = (char*)cmdBuffer->buf;
		//remove SV command name
		strsep(&work, " ");
		//get key
		char* key = strsep(&work, " ");
		//get payload
		char* nvspayload = work;

		if(work == NULL)
		{
			ESP_LOGI(EXT_UART_TAG,"error setting string: no value provided");
			if(cmdBuffer->sendToUART != 0)
			{
				uart_write_bytes(ext_uart_num, "NVS:ESP_ERR_NVS_NO_VALUE",strlen("NVS:ESP_ERR_NVS_NO_VALUE"));
				uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
			}
			return;
		}

		//try to set string data to nvs
		ret = nvs_set_str(nvs_storage_h, key,nvspayload);

		if(ret == ESP_OK)
		{
			//commit NVS storage
			ret = nvs_commit(nvs_storage_h);
		}

		if(ret != ESP_OK)
		{
			//send back error message
			ESP_LOGI(EXT_UART_TAG,"error setting string: %s",esp_err_to_name(ret));
			if(cmdBuffer->sendToUART != 0)
			{
				uart_write_bytes(ext_uart_num, "NVS:",strlen("NVS:"));
				uart_write_bytes(ext_uart_num, esp_err_to_name(ret), strlen(esp_err_to_name(ret)));
				uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
			}
		} else {
			//send back OK & used/free entries
			nvs_stats_t nvs_stats;
			nvs_get_stats(NULL, &nvs_stats);
			ESP_LOGI(EXT_UART_TAG,"set - %s:%s - used:%d,free:%d",key,nvspayload,nvs_stats.used_entries, nvs_stats.free_entries);
			if(cmdBuffer->sendToUART != 0)
			{
				uart_write_bytes(ext_uart_num, "NVS:OK ", strlen("NVS:OK "));
				char stats[64];
				sprintf(stats,"%d/%d - used/free",nvs_stats.used_entries, nvs_stats.free_entries);
				uart_write_bytes(ext_uart_num,stats,strnlen(stats,64));
				uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
			}
		}
		return;
	}

    /**++++ commands without parameters ++++*/
    //get connected devices
    if(strcmp(input,"GC") == 0)
    {
		char hexnum[5];
		ESP_LOGI(EXT_UART_TAG,"connected devices (starting with index 0):");
		ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
		for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
		{
			esp_bd_addr_t empty = {0,0,0,0,0,0};

			//only select active connections
			if(memcmp(active_connections[i],empty,sizeof(esp_bd_addr_t)) != 0)
			{
				//print on monitor & external uart
				if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, "CONNECTED:",strlen("CONNECTED:"));
				ESP_LOG_BUFFER_HEX_LEVEL(EXT_UART_TAG, active_connections[i], sizeof(esp_bd_addr_t), ESP_LOG_INFO);
				for (int t=0; t<sizeof(esp_bd_addr_t); t++) {
					sprintf(hexnum,"%02X ",active_connections[i][t]);
					if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, hexnum, 3);
				}
				if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
			}
		}
		ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
		return;
	}
    //switch between BT devices which are connected...
    if(input[0] == 'S' && input[1] == 'W')
    {
		if(len >= 15)
		{
			esp_bd_addr_t newaddr;
			for(uint8_t i = 0; i<6; i++) {
				if(input[i*2+3] >= '0' && input[i*2+3] <= '9') newaddr[i] = (input[i*2+3] - '0')<<4;
				if(input[i*2+3] >= 'a' && input[i*2+3] <= 'f') newaddr[i] = (input[i*2+3] + 10 - 'a')<<4;
				if(input[i*2+3] >= 'A' && input[i*2+3] <= 'F') newaddr[i] = (input[i*2+3] + 10 - 'A')<<4;

				if(input[i*2+1+3] >= '0' && input[i*2+1+3] <= '9') newaddr[i] |= input[i*2+1+3] - '0';
				if(input[i*2+1+3] >= 'a' && input[i*2+1+3] <= 'f') newaddr[i] |= input[i*2+1+3] + 10 - 'a';
				if(input[i*2+1+3] >= 'A' && input[i*2+1+3] <= 'F') newaddr[i] |= input[i*2+1+3] + 10 - 'A';
			}
			ESP_LOG_BUFFER_HEX_LEVEL(HID_DEMO_TAG, newaddr, 6, ESP_LOG_INFO);

			for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
			{
				//check if this addr is in the array
				if(memcmp(active_connections[i],newaddr,sizeof(esp_bd_addr_t)) == 0)
				{
					ESP_LOGI(EXT_UART_TAG, "New hid_conn_id: %d",i);
					hid_conn_id = i;
					return;
				}
			}
			ESP_LOGW(EXT_UART_TAG,"Cannot find BT MAC in connections");
		} else {
			ESP_LOGW(EXT_UART_TAG,"Command to short (need full BT MAC addr): %d",len);
		}
		return;
	}


    //get module ID
    if(strcmp(input,"ID") == 0)
    {
		if(cmdBuffer->sendToUART != 0)
		{
			uart_write_bytes(ext_uart_num, MODULE_ID, sizeof(MODULE_ID));
			uart_write_bytes(ext_uart_num, nl, sizeof(nl));
		}
		ESP_LOGI(EXT_UART_TAG,"ID: %s",MODULE_ID);
        return;
    }
    //disable pairing
    if(strcmp(input,"PM0") == 0)
    {
#if CONFIG_MODULE_BT_PAIRING
        ESP_LOGI(EXT_UART_TAG,"$PM0 - disabling pairing");
        xEventGroupClearBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
        if(hidd_adv_params.adv_filter_policy == ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY)
        {
            //restart advertising with whitelisted connection only (no connection is possible if not
            // on whitelist)
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
            esp_ble_gap_start_advertising(&hidd_adv_params);
        }
#else
        ESP_LOGW(EXT_UART_TAG,"Not available, cannot change pairing state (bug in esp-idf)");
#endif
        return;
    }
    //enable pairing
    if(strcmp(input,"PM1") == 0)
    {
#if CONFIG_MODULE_BT_PAIRING
        ESP_LOGI(EXT_UART_TAG,"$PM1 - enabling pairing");
        xEventGroupSetBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
        if(hidd_adv_params.adv_filter_policy == ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST)
        {
            //restart advertising with open connection
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
            esp_ble_gap_start_advertising(&hidd_adv_params);
        }
#else
        ESP_LOGW(EXT_UART_TAG,"Not available, cannot change pairing state (bug in esp-idf)");
#endif
        return;
    }

    //get all BT pairings
    if(strcmp(input,"GP") == 0)
    {
        counter = esp_ble_get_bond_device_num();
        char hexnum[5];

        if(counter > 0)
        {
            btdevlist = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t)*counter);
            if(btdevlist != NULL)
            {
                if(esp_ble_get_bond_device_list(&counter,btdevlist) == ESP_OK)
                {
                    ESP_LOGI(EXT_UART_TAG,"bonded devices (starting with index 0):");
                    ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
                    for(uint8_t i = 0; i<counter; i++)
                    {
                        //print on monitor & external uart
                        if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, "PAIRING:",strlen("PAIRING:"));
                        ESP_LOG_BUFFER_HEX_LEVEL(EXT_UART_TAG, btdevlist[i].bd_addr, sizeof(esp_bd_addr_t), ESP_LOG_INFO);
                        for (int t=0; t<sizeof(esp_bd_addr_t); t++) {
                            sprintf(hexnum,"%02X ",btdevlist[i].bd_addr[t]);
                            if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, hexnum, 3);
                        }
                        //print out name
                        char btname[64];
                        size_t name_len = 0;
                        char key[13];
                        sprintf(key,"%02X%02X%02X%02X%02X%02X",btdevlist[i].bd_addr[0],btdevlist[i].bd_addr[1], \
							btdevlist[i].bd_addr[2],btdevlist[i].bd_addr[3],btdevlist[i].bd_addr[4],btdevlist[i].bd_addr[5]);

                        if(nvs_get_str(nvs_bt_name_h,key,btname,&name_len) == ESP_OK)
                        {
							sprintf(hexnum," - ");
							if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, hexnum, 3);
							if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, btname, name_len);
							ESP_LOGI(EXT_UART_TAG,"%s",btname);
						} else ESP_LOGW(EXT_UART_TAG,"cannot find name for addr.");

                        if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num,nl,sizeof(nl)); //newline
                    }
                    ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
                } else ESP_LOGW(EXT_UART_TAG,"error getting device list");
            } else ESP_LOGE(EXT_UART_TAG,"error allocating memory for device list");
        } else {
			ESP_LOGI(EXT_UART_TAG,"error getting bonded devices count or no devices bonded");
			if(cmdBuffer->sendToUART != 0) uart_write_bytes(ext_uart_num, "END\r\n", 5);
		}
        return;
    }


    //DP: delete one pairing
    if(input[0] == 'D' && input[1] == 'P')
    {
        int index_to_remove;
        if (!(get_int(input,2,&index_to_remove))) {
            ESP_LOGI(EXT_UART_TAG,"DP: no integer, deleting all bonded devices");
            index_to_remove = -1;
        }

        counter = esp_ble_get_bond_device_num();
        if(counter == 0)
        {
            ESP_LOGI(EXT_UART_TAG,"error deleting device, no paired devices");
            return;
        }

        if(index_to_remove >= counter)
        {
            ESP_LOGW(EXT_UART_TAG,"error deleting device, number out of range");
            return;
        }
        if(counter >= 0)
        {
            btdevlist = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t)*counter);
            if(btdevlist != NULL)
            {
                if(esp_ble_get_bond_device_list(&counter,btdevlist) == ESP_OK)
                {
					//deleting only one pairing (-> -1 == delete all)
					if(index_to_remove >= 0)
					{
	                    esp_ble_remove_bond_device(btdevlist[index_to_remove].bd_addr);
	                    esp_ble_gap_update_whitelist(false,btdevlist[index_to_remove].bd_addr,BLE_WL_ADDR_TYPE_PUBLIC);
	                    esp_ble_gap_update_whitelist(false,btdevlist[index_to_remove].bd_addr,BLE_WL_ADDR_TYPE_RANDOM);
					} else {
						for(int i = 0; i<counter; i++)
						{
							esp_ble_remove_bond_device(btdevlist[i].bd_addr);
		                    esp_ble_gap_update_whitelist(false,btdevlist[i].bd_addr,BLE_WL_ADDR_TYPE_PUBLIC);
		                    esp_ble_gap_update_whitelist(false,btdevlist[i].bd_addr,BLE_WL_ADDR_TYPE_RANDOM);
						}
					}
                } else ESP_LOGI(EXT_UART_TAG,"error getting device list");
                free (btdevlist);
                //wait 20 ticks for everything to settle (write commits to NVS)
                vTaskDelay(20);
                //then restart to avoid re-bonding of the device(s).
                esp_restart();
            } else ESP_LOGW(EXT_UART_TAG,"error allocating memory for device list");
        } else ESP_LOGW(EXT_UART_TAG,"error getting bonded devices count");
        return;
    }

    //set BT GATT advertising name
    if(strncmp(input,"NAME ", 5) == 0)
    {
        if ((strlen(input)>6) && (strlen(input)-4<MAX_BT_DEVICENAME_LENGTH))
        {
            strcpy (config.bt_device_name, input+5);
            update_config();
            ESP_LOGI(EXT_UART_TAG,"NAME: new bt device name was stored");
        }
        else ESP_LOGI(EXT_UART_TAG,"NAME: given bt name is too long or too short");
        return;
    }

    //UG: triggering update mode of ESP by restarting into "factory partition"
    if (strcmp(input, "UG") == 0)
    {
        esp_partition_iterator_t pi;

        pi = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

        if (pi != NULL) {
            const esp_partition_t* factory = esp_partition_get(pi);
            esp_partition_iterator_release(pi);
            if (esp_ota_set_boot_partition(factory) == ESP_OK) {
                uart_write_bytes(ext_uart_num, "OTA:start", strlen("OTA:start"));
                uart_write_bytes(ext_uart_num, nl, sizeof(nl));
                ESP_LOGI(EXT_UART_TAG, "Addon board in upgrade mode");
                //LED off
                #if CONFIG_MODULE_NANO
                gpio_set_level(indicator_led, 1);
                #endif
                #if CONFIG_MODULE_MINIBT
                gpio_set_level(indicator_led, 0);
                #endif
                esp_restart();
            }else {
                ESP_LOGI(EXT_UART_TAG, "Booting factory partition not possible");
                uart_write_bytes(ext_uart_num, "OTA:not possible", strlen("OTA:not possible"));
                uart_write_bytes(ext_uart_num, nl, sizeof(nl));
            }
        } else {
			ESP_LOGI(EXT_UART_TAG, "Factory partition not found");
			uart_write_bytes(ext_uart_num, "OTA:not possible", strlen("OTA:not possible"));
			uart_write_bytes(ext_uart_num, nl, sizeof(nl));
		}
        return;
    }

    if(strcmp(input,"WHEELUP") == 0)
    {
        on_thumbwheel(1);
        ESP_LOGW(EXT_UART_TAG,"Moved the wheel up");
        return;
    }

    if(strcmp(input,"WHEELDOWN") == 0)
    {
        on_thumbwheel(-1);
        ESP_LOGW(EXT_UART_TAG,"Moved the wheel down");
        return;
    }

    ESP_LOGW(EXT_UART_TAG,"No command executed with: %s ; len= %d\n",input,len);
}


void uart_parse_command (uint8_t character, struct cmdBuf * cmdBuffer)
{
    switch (cmdBuffer->state) {

        case CMDSTATE_IDLE:
        if (character==0xfd) {
            cmdBuffer->bufferLength=0;
            cmdBuffer->expectedBytes=8;   // 8 bytes for raw report size
            cmdBuffer->state=CMDSTATE_GET_RAW;
        }
        else if (character == '$') {
            cmdBuffer->bufferLength=0;   // we will read an ASCII-command until CR or LF
            cmdBuffer->state=CMDSTATE_GET_ASCII;
        }
        break;

        case CMDSTATE_GET_RAW:
        cmdBuffer->buf[cmdBuffer->bufferLength]=character;
        if ((cmdBuffer->bufferLength == 1) && (character==0x01)) { // we have a joystick report: increase by 5 bytes
            cmdBuffer->expectedBytes += 5;
            //ESP_LOGI(EXT_UART_TAG,"expecting 5 more bytes for joystick");
        }

        cmdBuffer->bufferLength++;
        cmdBuffer->expectedBytes--;
        if (!cmdBuffer->expectedBytes) {
            if(!isConnected()) {
                ESP_LOGI(EXT_UART_TAG,"not connected, cannot send report");
            } else {
                if (cmdBuffer->buf[1] == 0x00) {   // keyboard report
                    //if hid_conn_id is set (!= -1) we send to one device only. Send to all otherwise
                    if(hid_conn_id == -1)
                    {
                        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                        {
                            if(active_hid_conn_ids[i] != -1) esp_hidd_send_keyboard_value(active_hid_conn_ids[i],cmdBuffer->buf[0],&cmdBuffer->buf[2],6);
                        }
                    } else {
                        esp_hidd_send_keyboard_value(hid_conn_id,cmdBuffer->buf[0],&cmdBuffer->buf[2],6);
                    }

                    //update timestamp
                    timestampLastSent = esp_timer_get_time();
                } else if (cmdBuffer->buf[1] == 0x01) {  // joystick report
                    ESP_LOGI(EXT_UART_TAG,"joystick: axis: 0x%X:0x%X:0x%X:0x%X, hat: %d",cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5],cmdBuffer->buf[8]);
                    ESP_LOGI(EXT_UART_TAG,"joystick: buttons: 0x%X:0x%X:0x%X:0x%X",cmdBuffer->buf[9],cmdBuffer->buf[10],cmdBuffer->buf[11],cmdBuffer->buf[12]);
                    //@todo should be HID_JOYSTICK_IN_RPT_LEN, but not available here.
                    uint8_t joy[11];
                    memcpy(joy,&cmdBuffer->buf[2],11);
                    //send joystick report
                    #if CONFIG_MODULE_USEJOYSTICK
                    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                    {
                        if(active_hid_conn_ids[i] != -1) esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
                    #else
                    ESP_LOGE(EXT_UART_TAG,"built without joystick support, cannot fix that!");
                    #endif
                } else if (cmdBuffer->buf[1] == 0x03) {  // mouse report
                    if(hid_conn_id == -1)
                    {
                        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                        {
                            if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5]);
                        }
                    } else {
                        esp_hidd_send_mouse_value(hid_conn_id,cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5]);
                    }
                    //update timestamp
                    timestampLastSent = esp_timer_get_time();
                    //and save mouse button state
                    mouseButtons = cmdBuffer->buf[2];
                    //ESP_LOGI(EXT_UART_TAG,"m: %d/%d",cmdBuffer->buf[3],cmdBuffer->buf[4]);
                }
                else ESP_LOGW(EXT_UART_TAG,"Unknown RAW HID packet");
            }
            cmdBuffer->state=CMDSTATE_IDLE;
        }
        break;

        case CMDSTATE_GET_ASCII:
        // collect a command string until CR or LF are received
        if ((character==0x0d) || (character==0x0a))  {
            cmdBuffer->buf[cmdBuffer->bufferLength]=0;
            ESP_LOGI(EXT_UART_TAG,"sending command to parser: %s",cmdBuffer->buf);

            processCommand(cmdBuffer);
            cmdBuffer->state=CMDSTATE_IDLE;
        } else {
            if (cmdBuffer->bufferLength < MAX_CMDLEN-1)
            cmdBuffer->buf[cmdBuffer->bufferLength++]=character;
        }
        break;
        default:
        cmdBuffer->state=CMDSTATE_IDLE;
    }
}

void uart_external_task(void *pvParameters)
{
    char character;
    struct cmdBuf cmdBuffer;
    int changePinning = 0;
    #if CONFIG_MODULE_MINIBT
      /*  determine if we should switch RX/TX pins.  */
      /*   we enable the RX pin as GPIO with pull-down.
       * 	 if "1" is read, this is the "real" RX pin. If not, it
       *   should be the TX pin. */

      gpio_config_t io_conf;
      //disable interrupt
      io_conf.intr_type = GPIO_INTR_DISABLE;
      //set as output mode
      io_conf.mode = GPIO_MODE_INPUT;
      //bit mask, we want to set the RX pin as input
      io_conf.pin_bit_mask = (1ULL<<ext_uart_rx);
      //disable pull-down mode
      io_conf.pull_down_en = 1;
      //disable pull-up mode
      io_conf.pull_up_en = 0;
      //configure GPIO with the given settings
      gpio_config(&io_conf);

      vTaskDelay(1);

      if(!gpio_get_level(ext_uart_rx))
      {
        ESP_LOGW(EXT_UART_TAG,"Switching pins!");
        changePinning = 1;
      }
    #endif


    //Install UART driver, and get the queue.
    esp_err_t ret = ESP_OK;
    uart_config_t uart_config = {
    /** on esp32miniBT module we use 9k6 for backwards compatibility
     * on Arduino Nano Connect with the uBlox Nina module we use
     * UART0 at 115k2 */
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    #if CONFIG_MODULE_NANO
      uart_config.baud_rate = 115200;
    #endif

    if (uart_driver_install(ext_uart_num, 1024, 1024, 0, NULL, 0) != ESP_OK)
    {
        ESP_LOGE(EXT_UART_TAG, "Failed to install driver");
    }

    //update UART config
    ret = uart_param_config(ext_uart_num, &uart_config);
    if(ret != ESP_OK)
    {
        ESP_LOGE(EXT_UART_TAG,"external UART param config failed");
    }

    //set IO pins
    #if CONFIG_MODULE_MINIBT
      if(changePinning) ret = uart_set_pin(ext_uart_num, ext_uart_rx, ext_uart_tx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
      else ret = uart_set_pin(ext_uart_num, ext_uart_tx, ext_uart_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
      if(ret != ESP_OK)
      {
        ESP_LOGE(EXT_UART_TAG,"external UART set pin failed");
      }
    #endif

    if (uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        ESP_LOGE(EXT_UART_TAG,"Failed to set UART pins");
    }

    ESP_LOGI(EXT_UART_TAG,"external UART processing task started");
    cmdBuffer.state=CMDSTATE_IDLE;
    cmdBuffer.sendToUART = 1;
    uart_write_bytes(ext_uart_num, "Console...\r\n", 11);

    while(1)
    {
        // read & process a single byte
        if (uart_read_bytes(ext_uart_num, (uint8_t*) &character, 1, 1) > 0)
        {
            uart_parse_command(character, &cmdBuffer);
        }
    }
}

void uart_console_task(void *pvParameters)
{
    char character;
    uint8_t kbdcmd[] = {28};
        //use input as HID test OR as input to processCommand (test commands)
        uint8_t hid_or_command = 0;
    struct cmdBuf commands;
    commands.sendToUART = 0;
    commands.state=CMDSTATE_IDLE;
    #if CONFIG_MODULE_USEJOYSTICK
        uint8_t joy[11] = {0};
    #endif

    //Install UART driver, and get the queue.
    uart_driver_install(CONSOLE_UART_NUM, UART_HW_FIFO_LEN(CONSOLE_UART_NUM) * 2, UART_HW_FIFO_LEN(CONSOLE_UART_NUM) * 2, 0, NULL, 0);

    ESP_LOGI("UART","console UART processing task started");

    while(1)
    {
        // read single byte
        uart_read_bytes(CONSOLE_UART_NUM, (uint8_t*) &character, 1, portMAX_DELAY);
        #if CONFIG_MODULE_NANO
          //if communcating with the RP2040, we need "dual-use" on UART0:
          // * debugging via esp-idf logger
          // * receiving HID stuff & commands
          //so we simply do the same stuff as in uart_external task
          uart_parse_command(character,&commands);
        #endif
        #if CONFIG_MODULE_MINIBT
          //enable character forwarding to command parser (used to test the commands)
          if(character == '$' && hid_or_command == 0) hid_or_command = 1;
          //if command mode is enabled, forward to uart_parse_command and continue with next character receiving.
          if(hid_or_command == 1)
          {
            uart_parse_command(character,&commands);
            if(commands.state == CMDSTATE_IDLE) hid_or_command = 0;
            continue;
          }
        #endif

        //if not in command mode, issue HID test commands.
        if(!isConnected()) {
            ESP_LOGI(CONSOLE_UART_TAG,"Not connected, ignoring '%c'", character);
        } else {
            switch (character) {
            case 'm':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_MUTE,true);
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_MUTE,false);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"consumer: mute");
                break;
    #if CONFIG_MODULE_USEJOYSTICK
            case '1':
                joy[8] = 0x01;
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"joystick button 1: press");
                break;
            case '2':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"joystick release");
                break;
            case '3':
                joy[0] = 127;
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"joystick axis1: 127");
                break;
            case '4':
                joy[0] = 0xFF;
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"joystick axis1: -127");
                break;
    #endif
            case 'p':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_VOLUME_UP,true);
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_VOLUME_UP,false);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"consumer: volume plus");
                break;
            case 'o':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_VOLUME_DOWN,true);
                        esp_hidd_send_consumer_value(active_hid_conn_ids[i],HID_CONSUMER_VOLUME_DOWN,false);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"consumer: volume minus");
                break;
            case 'a':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,-MOUSE_SPEED,0,0);
                }
                ESP_LOGI(CONSOLE_UART_TAG,"mouse: a");
                break;
            case 's':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,0,MOUSE_SPEED,0);
                }
                ESP_LOGI(CONSOLE_UART_TAG,"mouse: s");
                break;
            case 'd':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,MOUSE_SPEED,0,0);
                }
                ESP_LOGI(CONSOLE_UART_TAG,"mouse: d");
                break;
            case 'w':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,0,-MOUSE_SPEED,0);
                }
                ESP_LOGI(CONSOLE_UART_TAG,"mouse: w");
                break;
            case 'l':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_mouse_value(active_hid_conn_ids[i],(1<<0),0,0,0);
                        esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,0,0,0);
                    }
                }
                break;
            case 'r':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        esp_hidd_send_mouse_value(active_hid_conn_ids[i],(1<<1),0,0,0);
                        esp_hidd_send_mouse_value(active_hid_conn_ids[i],0,0,0,0);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"mouse: r");
                break;
            case 'q':
                for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                {
                    if(active_hid_conn_ids[i] != -1)
                    {
                        kbdcmd[0] = 28;
                        esp_hidd_send_keyboard_value(active_hid_conn_ids[i],0,kbdcmd,1);
                        kbdcmd[0] = 0;
                        esp_hidd_send_keyboard_value(active_hid_conn_ids[i],0,kbdcmd,1);
                    }
                }
                ESP_LOGI(CONSOLE_UART_TAG,"received q: sending key y (z for QWERTZ) for test purposes");
                break;
            default:
                ESP_LOGI(CONSOLE_UART_TAG,"received: %d, no HID action",character);
                break;
            }
        }
    }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cli_init(void)
{

    ext_uart_num = CONFIG_MODULE_UART_NR;
    ext_uart_rx = CONFIG_MODULE_RX_PIN;
    ext_uart_tx = CONFIG_MODULE_TX_PIN;
#if THUMBWHEEL_USE_CONSOLE

    #if CONFIG_MODULE_MINIBT
      xTaskCreate(&uart_console_task,  "console", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    #endif
    xTaskCreate(&uart_external_task, "external", 4096, NULL, configMAX_PRIORITIES-1, NULL);
#endif /* THUMBWHEEL_USE_CONSOLE */

}
/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "stdout/stdout.h"
#include "user.h"

MQTT_Client mqttClient;
 
void debounce_timerfunc(void *arg)
{	
	MQTT_Client* client = (MQTT_Client*)&mqttClient;
	
	uint32 gpio_00 = GPIO_INPUT_GET(0);
		
	if (gpio_00) {
			MQTT_Publish(client, "/ESP-01/GPIO00", "0", 1, 0, 0); //send hello
		}
	else
		{
			MQTT_Publish(client, "/ESP-01/GPIO00", "1", 1, 0, 0); //send hello
		}
		
	INFO("swtich status change\r\n");
	
    //enable interrupt
    gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_ANYEGDE); //mis-spelled in SDK
}

//Call back for WIFI connection
void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}

//Call back for connect
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, "/ESP-01/GPIO02", 0); //subcribe relavant topic
	MQTT_Publish(client, "/ESP-01/status", "hello", 5, 0, 0); //send hello
}

//Call back for disconnect
void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

//Call back for message published
void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

//Call back for message recieved
void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;
	
  	char *sp = topicBuf; // string pointer accessing internals of topicBuf
	
	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	
	//Process Recieved Data
 	if (strcmp(sp, "/ESP-01/GPIO02") == 0) {
		if (atoi(dataBuf)) {
			GPIO_OUTPUT_SET(2, 1); //Set GPIO2 High
			//gpio_output_set(BIT2, 0, BIT2, 0);
		}
		else 
		{
			GPIO_OUTPUT_SET(2, 0); //Set GPIO2 low
			//gpio_output_set(0, BIT2, BIT2, 0);
		}
	}
	 
	os_free(topicBuf);
	os_free(dataBuf);
}

LOCAL void switch_intr_handler(int8_t key)
{
    //disable interrupt
    gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_DISABLE);

	//Arm the debounce timer
    os_timer_disarm(&debounce_timer);
    os_timer_setfn(&debounce_timer, (os_timer_func_t *)debounce_timerfunc, NULL);
    os_timer_arm(&debounce_timer, 50, 0);
 
    //clear interrupt status
	uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(0));
}

void ICACHE_FLASH_ATTR switch_init()
{
    uint8 i;
    ETS_GPIO_INTR_ATTACH(switch_intr_handler,0);
    ETS_GPIO_INTR_DISABLE();
    
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	GPIO_DIS_OUTPUT(0); //Set GPIO0 to input
	GPIO_OUTPUT_SET(0, 1); //Set GPIO0 hight ot endable pull up.
	
    gpio_register_set(GPIO_PIN_ADDR(0), GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
          | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
          | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));
 
    //clear gpio status
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(0));
 
    //enable interrupt
    gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_ANYEGDE); //mis-spelled in SDK
    ETS_GPIO_INTR_ENABLE();
}

void ICACHE_FLASH_ATTR led_init()
{
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); //Set GPIO2 to output mode
	GPIO_OUTPUT_SET(2, 0); //Set GPIO2 low
}


void user_init(void)
{
    // Initialize the GPIO subsystem.
    gpio_init();
	
	led_init();
	
	switch_init(); //init iput key on GPIO0
	
    stdout_init(); //init TXD 9600baud. free up RXD for GPIO

	os_delay_us(1000000);

	CFG_Load(); //Load CFG SSID not loaded correctly on ESP8266-01
	
	//SetUp MQTT client
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, "/ESP-01/status", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
	
	//Connect to WIFI
	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
}

/**
 * @file main.c
 * @author Ho Xuan Phu (@Phu20214041)
 * @brief Main file of IOT Fishtank project
 * @version 0.1
 * @date 2023-09-09
 * 
 * @copyright Copyright (C) 2023
*/

/*--------------------------------------INCLUDE LIBRARY----------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_event_base.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_base.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "i2cdev.h"
#include "bmp280.h"
#include "bme280.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "DeviceManager.h"
#include "datamanager.h"



/*------------------------------------------DEFINE--------------------------------------------*/

__attribute__((unused)) static const char *TAG_main = "Main";
__attribute__((unsued)) static const char *TAG_wifi = "Wifi";

#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(5000 / portTICK_RATE_MS)

#define NO_WAIT (TickType_t)(0)
#define WAIT_10_TICK (TickType_t)(10 / portTICK_RATE_MS)
#define WAIT_100_TICK (TickType_t)(100 / portTICK_RATE_MS) 

#define QUEUE_SIZE 10U

#define WIFI_CONNECTED_SUCCESSFULLY_BIT BIT0
#define WIFI_CONNECTED_FAILED_BIT BIT1



EventGroupHandle_t taskCompletionEventGroup;

esp_mqtt_client_handle_t mqttClient_handle = NULL;

TaskHandle_t getDataFromSensorTask_handle = NULL ;
TaskHandle_t displayData_SSD1306 = NULL ;

SemaphoreHandle_t getDataSensor_semaphore = NULL;
SemaphoreHandle_t sentDataToMQTT_semaphore = NULL;
SemaphoreHandle_t allocateDataToMQTTandSDQueue_semaphore = NULL;

QueueHandle_t dataSensorSentToSSD1306_queue;
QueueHandle_t dataSensorSentToMQTT_queue;

static EventGroupHandle_t s_wifi_event_group;

static struct statusDevice_st statusDevice = {0};


static int s_retry_num = 0 ;

/*------------------------------------------------------------------------------------------------------------------------*/

i2c_dev_t ds3231_device ;
SSD1306_t SSD1306_device ;
bmp280_t bme280_device ;
bmp280_params_t bme280_params ;



/*--------------------------------------------WIFI---------------------------------------------------*/

static esp_err_t event_wifi_handler(void* argument, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_MAXIMUM_RETRY_CONNECT_WIFI) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_wifi, "Retry to connect to the AP %d time",s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_FAILED_BIT);
        }
        ESP_LOGI(TAG_wifi,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_wifi, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_SUCCESSFULLY_BIT);
    }
    return ESP_OK;
}

void Wifi_initMode_STA(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_wifi_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_wifi_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_wifi, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_SUCCESSFULLY_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_CONNECTED_FAILED_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_SUCCESSFULLY_BIT | WIFI_CONNECTED_FAILED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_SUCCESSFULLY_BIT) {
        ESP_LOGI(TAG_wifi, "Connected to ap SSID:%s password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else if (bits & WIFI_CONNECTED_FAILED_BIT) {
        ESP_LOGI(TAG_wifi, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else {
        ESP_LOGE(TAG_wifi, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}


// void displayData_SSD1306(void *argument){
    
// }

void getDataFromSensor_task(void *argument){
    struct dataSensor_st dataSensorTemp ;   

#if CONFIG_USING_BME280
    esp_err_t bme280error = bme280_readSensorData(  &bme280_device,&(dataSensorTemp.environment_temperature),
                                                    &(dataSensorTemp.humidity),&(dataSensorTemp.pressure));
#endif





}


void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_wifi, "ESP_WIFI_MODE_STA");
    Wifi_initMode_STA();
}

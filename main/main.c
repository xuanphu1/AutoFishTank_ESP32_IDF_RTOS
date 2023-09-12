/**
 * @file main.c
 * @author Ho Xuan Phu (@Phu20214041)
 * @brief Main file of IOT FishTank project
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
#include "time.h"

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
#include "ds18b20.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds3231.h"

/*------------------------------------------DEFINE--------------------------------------------*/

__attribute__((unused)) static const char *TAG_main = "Main";
__attribute__((unsued)) static const char *TAG_wifi = "Wifi";
__attribute__((unsued)) static const char *TAG_bme280 = "BME280";
__attribute__((unsued)) static const char *TAG_ds18b20 = "DS18B20";

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
QueueHandle_t dataSensorIntermediate_queue;

static EventGroupHandle_t s_wifi_event_group;

static struct statusDevice_st statusDevice = {0};


static int s_retry_num = 0 ;

/*------------------------------------------------------------------------------------------------------------------------*/

i2c_dev_t ds3231_device ;
SSD1306_t SSD1306_device ;
bmp280_t bme280_device ;
bmp280_params_t bme280_params ;
struct DS18B20_Info * ds18b20_info;
struct tm Time ;


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
    struct moduleError_st moduleErrorTemp;
    
    while (1)
    {

#if (CONFIG_USING_RTC)
    moduleErrorTemp.ds3231Error = ds3231_get_time(&ds3231_device,&Time);
    if (moduleErrorTemp.ds3231Error != ESP_OK) {
        ESP_LOGE(TAG_ds18b20,"Read data from DS3231 failed") 
    } else {
        ESP_LOGI(TAG_ds18b20,"Read data from DS3231 successfull")
    }
#endif



#if (CONFIG_USING_BME280)
    moduleErrorTemp.bme280Error = bme280_readSensorData(&bme280_device,&(dataSensorTemp.environment_temperature),
                                                        &(dataSensorTemp.humidity),&(dataSensorTemp.pressure)); 

#if (CONFIG_USING_D18B20)
    moduleErrorTemp.ds18b20Error = ds18b20_read_temp(ds18b20_info, &dataSensorTemp.water_temperature)
    if (moduleErrorTemp.ds18b20Error != ESP_OK) {
        ESP_LOGE(TAG_ds18b20,"Read data from DS18B20 failed") 
    } else {
        ESP_LOGI(TAG_ds18b20,"Read data from DS18B20 successfull")
    }
#endif

    ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.bme280Error);
    ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.ds18b20Error);

    ESP_LOGI(__func__, "Read data from sensors completed!");

    if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)        
    {
        if(xQueueSendToBack(dataSensorIntermediate_queue, (void *)&dataSensorTemp, WAIT_10_TICK * 5) != pdPASS){
            ESP_LOGE(__func__,"Failed to send the data sensor to dataSensorIntermidiate_queue");
        } else {
            ESP_LOGI(__func__,"Successfull to send the data sensor to dataSensorIntermidiate_queue");
        }
    }
    xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);

    memset(&dataSensorTemp, 0, sizeof(struct dataSensor_st));
    memset(&moduleErrorTemp, 0, sizeof(struct moduleError_st));
    vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSOR);
    }
};


void allocateDatatoDisplay_task(void *argument){
    struct dataSensor_st dataSensorReceiveFromQueue ;
    char Humidity[20] , Temprature_environment[20], Temprature_water[20];
    while (1)
    {
        if(uxQueueMessagesWaiting(dataSensorIntermidiate_queue) != 0){
           if(xQueueReceive(dataSensorIntermidate_queue, (void*)&dataSensorReceiveFromQueue, portMAX_DELAY) != pdPASS){
            ESP_LOGE(__func__,"Failed to receive data from dataSensorIntermidiate_queue");
           }else {
            ESP_LOGI(__func__,"Successfull to receive data from dataSensorIntermidiate_queue");
           }
            sprintf(Temprature_water,"Temp_2:%0.2f",dataSensorReceiveFromQueue.water_temperature);
            sprintf(Temprature_environment,"Temp:%0.2f",dataSenorReceiveFromQueue.environment_temperature);
            sprintf(Humidity."Humid:%0.2f",dataSenorReceiveFromQueue.humidity);
            ssd1306_display_text(&SSD1306_device,0,Temprature_environment,10, false);
            ssd1306_display_text(&SSD1306_device,1,Humidity,11, false);
            ssd1306_display_text(&SSD1306_device,2,Temprature_water,12, false);
        }
    }
}

void allocateDatatoUsingLightMotorFanHeater(void *arugment){
    struct dataSensor_st dataSensorReceiveFromQueue ;
    char Auto_system[20], stateAutoSystem[3];
    char Auto_light_fan[20], stateAutoLight[3], stateAutoFan[3];
    char Auto_heater_feed[20], stateAutoHeater[3], stateAutoFeed[3]; 

    while (1)
    {
    if(CONFIG_ON_OFF_AUTO_LIGHT){
        // ssd1306_clear_line(&SSD1306_device,4,false);
        stateAutoSystem = "ON";
    } else {
        // ssd1306_clear_line(&SSD1306_device,4,false);
        stateAutoSystem = "OFF";
        }

    sprintf(Auto_system,"Auto : %c");
    sprintf(Auto_light_fan,"L : %dh F 0.1%f C: ",CONFIG_HOUR_ON_LIGHT,CONFIG_TEMPRATURE_ON_FAN);

    ssd1306_display_text(&SSD1306_device,4,Auto_system,false);
#if (CONFIG_ON_OFF_AUTO_LIGHT)
    
    ssd1306_display_text();
    ssd1306_display_text();
    if (Time.tm_hour == CONFIG_HOUR_ON_LIGHT)
    {
        
        gpio_set_level(CONFIG_GPIO_LIGHT,true);
    }

    if(dataSensorReceiveFromQueue.water_temperature > CONFIG_TEMPRATURE_ON_FAN){
        gpio_set_level(CONFIG_GPIO_FAN,true);
    }

    if(dataSensorReceiveFromQueue.environment_temprature < CONFIG_TEMPRATURE_ON_HEATER){
        gpio_set_level(CONFIG_GPIO_HEATER,true);
    }
    



#else

    // Set up state logic of light 
    gpio_set_level(CONFIG_GPIO_LIGHT,CONFIG_STATE_LIGHT);
    // Set up state logic of fan
    gpio_set_level(CONFIG_GPIO_FAN,CONFIG_STATE_FAN);
    // Set up state logic of heater
    gpio_set_level(CONFIG_GPIO_HEATER,CONFIG_STATE_HEATER);


#endif
        
    }
    
}

void app_main(void)
{
/*-----------------------------------------Initialize NVS-------------------------*/
    esp_err_t nvs_error = nvs_flash_init();
    if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES || nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_error);

/*-----------------------------------------Initialize BME280-----------------------*/
#if (CONFIG_USING_BME280)
        // Initialize I2C
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());

    // Initialize BME280
    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(  &bme280_device, &bme280_params, BME280_ADDRESS, 
                        CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL ));
#endif

/*-----------------------------------------Initialize DS18B20-----------------------*/
#if (CONFIG_USING_DS18B20)
    // Override global log level
    esp_log_level_set("*", ESP_LOG_INFO);

    // Khởi tạo và cấu hình giao tiếp 1-Wire
    owb_rmt_driver_info rmt_driver_info;
    OneWireBus * owb = owb_rmt_initialize(&rmt_driver_info, CONFIG_ONE_WIRE_GPIO, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // Bật kiểm tra CRC cho mã ROM

    // Khởi tạo cảm biến DS18B20
    ds18b20_info = ds18b20_malloc();  // Cấp phát heap
    ds18b20_init_solo(ds18b20_info, owb);            // Khởi tạo cảm biến (chỉ có một cảm biến)
    ds18b20_use_crc(ds18b20_info, true);             // Bật kiểm tra CRC cho tất cả các lần đọc
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_12_BIT);

#endif
    
/*-----------------------------------------Initialize GPIO-----------------------*/
    gpio_pad_select_gpio(CONFIG_GPIO_LIGHT);
    gpio_set_direction(CONFIG_GPIO_LIGHT,GPIO_MODE_OUPUT);

    gpio_pad_select_gpio(CONFIG_GPIO_FAN);
    gpio_set_direction(CONFIG_GPIO_FAN,GPIO_MODE_OUPUT);
    
    gpio_pad_select_gpio(CONFIG_GPIO_HEATER);
    gpio_set_direction(CONFIG_GPIO_HEATER,GPIO_MODE_OUPUT);
    
#if (CONFIG_USING_WIFI)
ESP_LOGI(TAG_wifi, "ESP_WIFI_MODE_STA");
Wifi_initMode_STA();
#endif
}

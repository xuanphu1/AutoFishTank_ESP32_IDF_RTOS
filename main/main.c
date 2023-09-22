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

#include "../component/SNTP_Sync/sntp_sync.h"
#include "i2cdev.h"
#include "bmp280.h"
#include "bme280.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "DeviceManager.h"
#include "datamanager.h"
#include "../component/esp32_ds18b20/include/ds18b20.h"
#include "../component/esp32_owb/owb.h"
#include "../component/esp32_owb/owb_rmt.h"
#include "ds3231.h"
#include "DS3231Time.h"
#include "../component/motor/servo/iot_servo.h"
#include "sdkconfig.h"

/*------------------------------------------DEFINE--------------------------------------------*/
#define TAG_main  "Main"
#define TAG_wifi  "Wifi"
#define TAG_bme280 "BME280"
#define TAG_ds18b20 "DS18B20"
#define TAG_ssd1306 "SSD1306"
#define TAG_ds3231 "DS3231"
#define TAG_mqtt "MQTT"

#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(1000 / portTICK_RATE_MS)
#define PERIOD_DATA_DISPLAY_ON_OFF_SERVICE (TickType_t)(1000 / portTICK_RATE_MS)


#define NO_WAIT ((TickType_t)0)
#define WAIT_10_TICK (TickType_t)(10 / portTICK_RATE_MS)
#define WAIT_100_TICK (TickType_t)(100 / portTICK_RATE_MS) 

#define QUEUE_SIZE 10U

#define WIFI_CONNECTED_SUCCESSFULLY_BIT BIT0
#define WIFI_CONNECTED_FAILED_BIT BIT1





esp_mqtt_client_handle_t mqttClient_handle = NULL;

TaskHandle_t getDataFromSensorTask_handle = NULL ;
TaskHandle_t allocateDatatoDisplay_task_handle = NULL ;
TaskHandle_t allocateDatatoUsingLightMotorFanHeater_handle = NULL;
TaskHandle_t sntpGetTimeTask_handle = NULL;
TaskHandle_t mqttPublishMessageTask_handle = NULL ;

SemaphoreHandle_t getDataSensor_semaphore = NULL;
SemaphoreHandle_t sentDataToMQTT_semaphore = NULL;
SemaphoreHandle_t allocateDataToMQTTandSDQueue_semaphore = NULL;

QueueHandle_t dataSensorSentToSSD1306_queue;
QueueHandle_t dataSensorSentToMQTT_queue;
QueueHandle_t dataSensorIntermediate_queue;

static EventGroupHandle_t s_wifi_event_group;

static struct statusDevice_st statusDevice = {0};

// Whether send data to MQTT queue or not (depend on WIFI connection)
bool sendToMQTTQueue = false;

static int retry_cnt = 0;
int center, top, bottom;
char lineChar[20];


/*------------------------------------------------------------------------------------------------------------------------*/

i2c_dev_t ds3231_device ;
SSD1306_t SSD1306_device ;
bmp280_t bme280_device ;
bmp280_params_t bme280_params ;
const DS18B20_Info *ds18b20_info;
struct tm Time ;


/*--------------------------------------------WIFI---------------------------------------------------*/

static void mqtt_app_start(void);
static void sntp_app_start(void);



static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG_wifi, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG_wifi, "Wi-Fi connected AP SSID: %s PASSWORD: %s\n",CONFIG_SSID,CONFIG_PASSWORD);
        break;

    case IP_EVENT_STA_GOT_IP:

        
        ESP_LOGI(TAG_wifi, "Got ip starting MQTT Client: " IPSTR "\n", IP2STR(&event->ip_info.ip));

#ifdef CONFIG_RTC_TIME_SYNC
        if (sntpGetTimeTask_handle == NULL)
        {
            sntp_app_start();
        }
#endif
        /* When connect/reconnect wifi, esp32 take an IP address and this
         * event become active. If it's the first-time connection, create
         * task mqttPublishMessageTask, else resume that task. */
        if (mqttPublishMessageTask_handle == NULL){
            mqtt_app_start();
            statusDevice.mqttClient = CONNECTED;
        }else {
            if (eTaskGetState(mqttPublishMessageTask_handle) == eSuspended){
                vTaskResume(mqttPublishMessageTask_handle);
                ESP_LOGI(__func__,"MQTT resume publish message");
            }
        }
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        /* When esp32 disconnect to wifi, this event become
         * active. We suspend the task mqttPublishMessageTask. */
        ESP_LOGI(TAG_wifi, "disconnected: Retrying Wi-Fi SSID: %s PASSWORD: %s\n",CONFIG_SSID,CONFIG_PASSWORD);
        if(mqttPublishMessageTask_handle != NULL && eTaskGetState(mqttPublishMessageTask_handle) != eSuspended){
            vTaskSuspend(mqttPublishMessageTask_handle);
            statusDevice.mqttClient = DISCONNECTED;
            sendToMQTTQueue = false;
            if (retry_cnt++ < CONFIG_MAXIMUM_RETRY_CONNECT_WIFI)
            {
                ESP_LOGI(TAG_wifi,"Reconnect %d",retry_cnt);
                esp_wifi_connect();
            }
            else
                ESP_LOGI(TAG_wifi, "Max Retry Failed: Wi-Fi Connection\n");
        }
        
        break;

    default:
        break;
    }
    return ESP_OK;
}

void Wifi_initMode_STA(void)
{
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

/*--------------------------------------- MQTT -------------------------------------------------*/


/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(__func__, "MQTT_EVENT_CONNECTED");

        statusDevice.mqttClient = CONNECTED;

        sendToMQTTQueue = true;
        if (eTaskGetState(mqttPublishMessageTask_handle) == eSuspended)
        {
            vTaskResume(mqttPublishMessageTask_handle);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(__func__, "MQTT_EVENT_DISCONNECTED");
        statusDevice.mqttClient = DISCONNECTED;
        
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(__func__, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(__func__, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(__func__, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(__func__, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                                                                strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGE(__func__, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(__func__, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(__func__, "Other event id:%d", event->event_id);
        break;
    }
}

void mqttPublishMessage_task(void *arugment){
    
}

static void mqtt_app_start(void)
{
    ESP_LOGI(__func__, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://YOUR_RPI_IP_ADDRESS:1883"};

    mqttClient_handle = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(mqttClient_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, mqttClient_handle);
    esp_mqtt_client_start(mqttClient_handle);
}


/**
 * @brief SNTP Get time task : init sntp, then get time from ntp and save time to DS3231,
 *        finally delete itself (no loop task)
 *
 * @param parameter
 */
void sntpGetTime_task(void *parameter)
{
    time_t timeNow = 0;
    struct tm timeInfo = {0};
    do
    {
        sntp_init_func();
        ESP_ERROR_CHECK_WITHOUT_ABORT(sntp_setTime(&timeInfo, &timeNow));
        mktime(&timeInfo);
        if (timeInfo.tm_year < 130 && timeInfo.tm_year > 120)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_setTime(&ds3231_device, &timeInfo));
        }
        vTaskDelete(NULL);
    } while (0);
}

/**
 * @brief  This function initialize SNTP, then get time from ntp and save time to DS3231
 *
 *
 */
static void sntp_app_start()
{
    xTaskCreate(sntpGetTime_task, "SNTP Get Time", (1024 * 4), NULL, (UBaseType_t)19, &sntpGetTimeTask_handle);
}

void getDataFromSensor_task(void *argument){
    struct dataSensor_st dataSensorTemp = {0} ;
    struct moduleError_st moduleErrorTemp;
    TickType_t task_lastWakeTime = xTaskGetTickCount();  
    while (1)
    {

#if (CONFIG_USING_RTC)
        moduleErrorTemp.ds3231Error = ds3231_get_time(&ds3231_device,&Time);
        if (moduleErrorTemp.ds3231Error != ESP_OK) {
            ESP_LOGE(TAG_ds3231,"Read data from DS3231 failed") ;
        } else {
            ESP_LOGI(TAG_ds3231,"Read data from DS3231 successfull");
        }
#endif



#if (CONFIG_USING_BME280)
        moduleErrorTemp.bme280Error = bme280_readSensorData(&bme280_device,&(dataSensorTemp.environment_temperature),
                                                            &(dataSensorTemp.humidity),&(dataSensorTemp.pressure)); 
#endif
#if (CONFIG_USING_DS18B20)
        moduleErrorTemp.ds18b20Error = ds18b20_read_temp(ds18b20_info, &dataSensorTemp.water_temperature);
        if (moduleErrorTemp.ds18b20Error != ESP_OK) {
            ESP_LOGE(TAG_ds18b20,"Read data from DS18B20 failed") ;
        } else {
            ESP_LOGI(TAG_ds18b20,"Read data from DS18B20 successfull");
        }
#endif

        ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.bme280Error);
        // ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.ds18b20Error);

        ESP_LOGI(__func__, "Read data from sensors completed!");

        // if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)        
        // {
        //     if(xQueueSendToBack(dataSensorIntermediate_queue, (void *)&dataSensorTemp, WAIT_10_TICK * 5) != pdPASS){
        //         ESP_LOGE(__func__,"Failed to send the data sensor to dataSensorIntermidiate_queue");
        //     } else {
        //         ESP_LOGI(__func__,"Successfull to send the data sensor to dataSensorIntermidiate_queue");
        //     }
        // }
        // xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);

        if(xQueueSendToBack(dataSensorIntermediate_queue, (void *)&dataSensorTemp, WAIT_10_TICK * 5) != pdPASS){
                ESP_LOGE(__func__,"Failed to send the data sensor to dataSensorIntermidiate_queue");
            } else {
                ESP_LOGI(__func__,"Successfull to send the data sensor to dataSensorIntermidiate_queue");
            }

        memset(&dataSensorTemp, 0, sizeof(struct dataSensor_st));
        memset(&moduleErrorTemp, 0, sizeof(struct moduleError_st));
        vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSOR);
    }
};



void allocateDatatoDisplayAndOnOffService_task(void *argument){
    struct dataSensor_st dataSensorReceiveFromQueue ;
    char Humidity[20] , Temprature_environment[20], Temprature_water[20];
    char Auto_system[20], stateAutoSystem[4];
    char Auto_light_fan[20], stateAutoLight[4], stateAutoFan[4];
    char Auto_heater_feed[20], stateAutoHeater[4], stateAutoFeed[4];
    char realTime[20]; 
    while (1)
    {
        if(uxQueueMessagesWaiting(dataSensorIntermediate_queue) != 0){
           if(xQueueReceive(dataSensorIntermediate_queue, (void*)&dataSensorReceiveFromQueue, portMAX_DELAY) != pdPASS){
            ESP_LOGE(__func__,"Failed to receive data from dataSensorIntermidiate_queue");
           }else {
            ESP_LOGI(__func__,"Successfull to receive data from dataSensorIntermidiate_queue");
           }
            sprintf(Temprature_water,"Temp_water:%0.2f",dataSensorReceiveFromQueue.water_temperature);
            sprintf(Temprature_environment,"Temp:%0.2f",dataSensorReceiveFromQueue.environment_temperature);
            sprintf(realTime,"%d-%d-%d",Time.tm_hour,Time.tm_min,Time.tm_sec);
            // sprintf(Humidity,"Humid:%0.2f",dataSensorReceiveFromQueue.humidity);
            ssd1306_display_text(&SSD1306_device,0,Temprature_environment,10, false);
            // ssd1306_display_text(&SSD1306_device,1,Humidity,11, false);
            ssd1306_display_text(&SSD1306_device,1,Temprature_water,16, false);
            ssd1306_display_text(&SSD1306_device,4,realTime,9,false);



            #if(CONFIG_ON_OFF_AUTO)
                    // ssd1306_clear_line(&SSD1306_device,4,false);`                                                        
                    strcpy(stateAutoSystem,"ON");
                #else 
                    // ssd1306_clear_line(&SSD1306_device,4,false);
                    strcpy(stateAutoSystem,"OFF");
                #endif   

                sprintf(Auto_system,"Auto : %s",stateAutoSystem);

                ssd1306_display_text(&SSD1306_device,5,Auto_system,10,false);
            #if (CONFIG_ON_OFF_AUTO)
                sprintf(Auto_light_fan,"L: %dh | F: %dC ",CONFIG_HOUR_TURN_ON_LIGHT,CONFIG_TEMPERATURE_TURN_ON_FAN);
                sprintf(Auto_heater_feed,"M: %dh | H: %dC ",CONFIG_HOUR_TURN_ON_MOTOR,CONFIG_TEMPERATURE_TURN_ON_HEATER);

                ssd1306_display_text(&SSD1306_device,6,Auto_light_fan,16,false);
                ssd1306_display_text(&SSD1306_device,7,Auto_heater_feed,16,false);
                
                if (Time.tm_hour == CONFIG_HOUR_TURN_ON_MOTOR)
                {

                    gpio_set_level(CONFIG_GPIO_FILTER,false);

                    for (float i = 0 ; i <= 180 ; i++){
                        iot_servo_write_angle(LEDC_LOW_SPEED_MODE,CONFIG_GPIO_CHANNEL_MOTOR,i);
                    }
                    for (float i = 180 ; i >=0 ;i--){
                        iot_servo_write_angle(LEDC_LOW_SPEED_MODE,CONFIG_GPIO_CHANNEL_MOTOR,i);
                    }



                    if (Time.tm_min == 10) gpio_set_level(CONFIG_GPIO_FILTER,true);
                }

                if (Time.tm_hour >= CONFIG_HOUR_TURN_ON_LIGHT && Time.tm_hour <= CONFIG_HOUR_TURN_OFF_LIGHT)
                {
                    
                    gpio_set_level(CONFIG_GPIO_LIGHT,true);
                }

                if(dataSensorReceiveFromQueue.water_temperature > CONFIG_TEMPERATURE_TURN_ON_FAN){
                    gpio_set_level(CONFIG_GPIO_FAN,true);
                } else {
                    gpio_set_level(CONFIG_GPIO_FAN,false);
                }
                

                if(dataSensorReceiveFromQueue.environment_temperature < CONFIG_TEMPERATURE_TURN_ON_HEATER){
                    gpio_set_level(CONFIG_GPIO_HEATER,true);
                } else {
                    gpio_set_level(CONFIG_GPIO_HEATER,false);
                }
            #else 
                
                if (CONFIG_LOGIC_STATE_LIGHT){
                    strcpy(stateAutoLight,"ON");
                    gpio_set_level(CONFIG_GPIO_LIGHT,CONFIG_LOGIC_STATE_LIGHT);
                }
                else{
                    strcpy(stateAutoLight,"OFF");
                    gpio_set_level(CONFIG_GPIO_LIGHT,CONFIG_LOGIC_STATE_LIGHT);
                }

                if (CONFIG_LOGIC_STATE_FAN){
                    strcpy(stateAutoFan,"ON");
                    // Set up state logic of fan
                    gpio_set_level(CONFIG_GPIO_FAN,CONFIG_LOGIC_STATE_FAN);
                }
                else{
                    strcpy(stateAutoFan,"OFF");
                    // Set up state logic of fan
                    gpio_set_level(CONFIG_GPIO_FAN,CONFIG_LOGIC_STATE_FAN);
                }
                    

                if(CONFIG_LOGIC_STATE_HEATER){
                    strcpy(stateAutoHeater,"ON");
                    // Set up state logic of heater
                    gpio_set_level(CONFIG_GPIO_HEATER,CONFIG_LOGIC_STATE_HEATER);
                }
                else{
                    strcpy(stateAutoHeater,"OFF");
                    // Set up state logic of heater
                    gpio_set_level(CONFIG_GPIO_HEATER,CONFIG_LOGIC_STATE_HEATER);
                }
                    
                if(CONFIG_LOGIC_STATE_MOTOR){
                    strcpy(stateAutoFeed,"ON");
                    while (CONFIG_LOGIC_STATE_MOTOR)
                {
                    for (float i = 0 ; i <= 180 ; i++){
                    iot_servo_write_angle(LEDC_LOW_SPEED_MODE,CONFIG_GPIO_CHANNEL_MOTOR,i);
                    }
                    for (float i = 180 ; i >=0 ;i--){
                        iot_servo_write_angle(LEDC_LOW_SPEED_MODE,CONFIG_GPIO_CHANNEL_MOTOR,i);
                    }
                }
                }
                else{
                    strcpy(stateAutoFeed,"OFF");
                }
                sprintf(Auto_light_fan,"L: %s | F: %s",stateAutoLight,stateAutoFan);
                sprintf(Auto_heater_feed,"M: %s | H: %s",stateAutoFeed,stateAutoHeater);

                ssd1306_display_text(&SSD1306_device,6,Auto_light_fan,15,false);
                ssd1306_display_text(&SSD1306_device,7,Auto_heater_feed,15,false);
                // Set up state logic of light 

            #endif

        }
        vTaskDelay(WAIT_10_TICK);
    }
}



void app_main(void)
{

// Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(200));

/*-----------------------------------------Initialize NVS-------------------------*/
    esp_err_t nvs_error = nvs_flash_init();
    if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES || nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_error = nvs_flash_init();
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_error);
/*-----------------------------------------Initialize MG996----------------------*/

servo_config_t servo_config = {
    .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {          
                CONFIG_GPIO_CHANNEL_MOTOR,
            },
            .ch = {
                LEDC_CHANNEL_6,
            },
        },
        .channel_number = 1,
};
ESP_ERROR_CHECK_WITHOUT_ABORT(iot_servo_init(LEDC_LOW_SPEED_MODE,&servo_config));

/*-----------------------------------------Initialize BME280-----------------------*/
#if (CONFIG_USING_BME280)
        // Initialize I2C

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    ESP_LOGI(TAG_bme280, "Initialize BME280 sensor(I2C/Wire: %d).", CONFIG_BME_I2C_PORT);

    // Initialize BME280
    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(  &bme280_device, &bme280_params, BME280_ADDRESS, 
                        CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL ));
#endif

/*-----------------------------------------Initialize RTC---------------------------*/
#if(CONFIG_USING_RTC)

    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);

    memset(&ds3231_device, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
#endif

/*-----------------------------------------Initialize SSD1306----------------------*/
#if (CONFIG_USING_SSD1306)

	ESP_LOGI(TAG_ssd1306, "INTERFACE is i2c");
	ESP_LOGI(TAG_ssd1306, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(TAG_ssd1306, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(TAG_ssd1306, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&SSD1306_device, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

#if CONFIG_FLIP
	SSD1306_device._flip = true;
	ESP_LOGW(TAG_ssd1306, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(TAG_ssd1306, "Panel is 128x64");
    ESP_LOGI(TAG_ssd1306,"Initialize SSD1306 ");
	ssd1306_init(&SSD1306_device, 128, 64);
#endif

ssd1306_clear_screen(&SSD1306_device, false);
	ssd1306_contrast(&SSD1306_device, 0xff);
#endif

/*-----------------------------------------Initialize DS18B20-----------------------*/
#if (CONFIG_USING_DS18B20)
    // Override global log level
    esp_log_level_set("*", ESP_LOG_INFO);

    // Khởi tạo và cấu hình giao tiếp 1-Wire
    owb_rmt_driver_info rmt_driver_info;
    ESP_LOGI(TAG_ds18b20,"Initialize DS18B20 sensor (one/Wire %d)",CONFIG_ONE_WIRE_GPIO);
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
gpio_set_direction(CONFIG_GPIO_LIGHT, GPIO_MODE_OUTPUT);

gpio_pad_select_gpio(CONFIG_GPIO_FAN);
gpio_set_direction(CONFIG_GPIO_FAN, GPIO_MODE_OUTPUT);

gpio_pad_select_gpio(CONFIG_GPIO_HEATER);
gpio_set_direction(CONFIG_GPIO_HEATER, GPIO_MODE_OUTPUT);

gpio_pad_select_gpio(CONFIG_GPIO_FILTER);
gpio_set_direction(CONFIG_GPIO_FILTER, GPIO_MODE_OUTPUT);
/*-----------------------------------------Initialize WIFI-----------------------*/
#if (CONFIG_USING_WIFI)
    ESP_LOGI(TAG_wifi, "ESP_WIFI_MODE_STA");
    Wifi_initMode_STA();
#endif

/*-----------------------------------------Initialize Queue-----------------------*/
dataSensorIntermediate_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
while (dataSensorIntermediate_queue == NULL)
{
    ESP_LOGE(__func__, "Create dataSensorIntermediate Queue failed.");
    ESP_LOGI(__func__, "Retry to create dataSensorIntermediate Queue...");
    vTaskDelay(500 / portTICK_RATE_MS);
    dataSensorIntermediate_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
};
ESP_LOGI(__func__, "Create dataSensorIntermediate Queue success.");


/*-----------------------------------------Create task freertos-----------------------*/

xTaskCreate(getDataFromSensor_task,"Get data from sensor",(1024*16),NULL,(UBaseType_t)25,&getDataFromSensorTask_handle);

xTaskCreate(allocateDatatoDisplayAndOnOffService_task,"Display data and turn on turn off service",(1024*16),NULL,(UBaseType_t)20,&allocateDatatoDisplay_task_handle);



}

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "light_sleep.h"

void light_sleep(void)
{
        printf("Entering light sleep\n");
        /* To make sure the complete line is printed before entering sleep mode,
         * need to wait until UART TX FIFO is empty:
         */
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        /* Get timestamp before entering sleep */
        int64_t t_before_us = esp_timer_get_time();

        /* Enter sleep mode */
        esp_light_sleep_start();

        /* Get timestamp after waking up from sleep */
        int64_t t_after_us = esp_timer_get_time();

        /* Determine wake up reason */
        const char* wakeup_reason;
        switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                wakeup_reason = "pin";
                break;
            case ESP_SLEEP_WAKEUP_UART:
                wakeup_reason = "uart";
                /* Hang-up for a while to switch and execuse the uart task
                 * Otherwise the chip may fall sleep again before running uart task */
                vTaskDelay(1);
                break;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
            case ESP_SLEEP_WAKEUP_TOUCHPAD:
                wakeup_reason = "touch";
                break;
#endif
            default:
                wakeup_reason = "other";
                break;
        }
#if CONFIG_NEWLIB_NANO_FORMAT
        /* printf in newlib-nano does not support %ll format, causing example test fail */
        printf("Returned from light sleep, reason: %s, t=%d ms, slept for %d ms\n",
                wakeup_reason, (int) (t_after_us / 1000), (int) ((t_after_us - t_before_us) / 1000));
#else
        printf("Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
                wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
#endif
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
            /* Waiting for the gpio inactive, or the chip will continously trigger wakeup*/
            wait_gpio_inactive();
        }
}

//------------------------------GPIO wake up ----------------------------------------------------------------
/* Most development boards have "boot" button attached to GPIO0.
 * You can also change this to another pin.
 */
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32H2 \
    || CONFIG_IDF_TARGET_ESP32C6
#define BOOT_BUTTON_NUM         9
#else
#define BOOT_BUTTON_NUM         0
#endif

/* Use boot button as gpio input */
#define GPIO_WAKEUP_NUM         BOOT_BUTTON_NUM
/* "Boot" button is active low */
#define GPIO_WAKEUP_LEVEL       0

static const char *TAG_gpio = "gpio_wakeup";

void wait_gpio_inactive(void)
{
    printf("Waiting for GPIO%d to go high...\n", GPIO_WAKEUP_NUM);
    while (gpio_get_level(GPIO_WAKEUP_NUM) == GPIO_WAKEUP_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t register_gpio_wakeup(void)
{
    /* Initialize GPIO */
    gpio_config_t config = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = false,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG_gpio, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM);

    /* Enable wake up from GPIO */
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM, GPIO_WAKEUP_LEVEL == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG_gpio, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG_gpio, "Configure gpio as wakeup source failed");

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    wait_gpio_inactive();
    ESP_LOGI(TAG_gpio, "gpio wakeup source is ready");

    return ESP_OK;
}
//-----------------------------------------------------------------------------------------------------------

//----------------------------------- Timer GPIO wake up ----------------------------------------------------

static const char *TAG_timer = "timer_wakeup";

esp_err_t register_timer_wakeup(int timerWakeupTimeUs)
{
    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup(timerWakeupTimeUs), TAG_timer, "Configure timer as wakeup source failed");
    ESP_LOGI(TAG_timer, "timer wakeup source is ready");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------------------------------

//----------------------------------- UART wake up ----------------------------------------------------
#define EXAMPLE_UART_NUM        0
/* Notice that ESP32 has to use the iomux input to configure uart as wakeup source
 * Please use 'UxRXD_GPIO_NUM' as uart rx pin. No limitation to the other target */
#define EXAMPLE_UART_TX_IO_NUM  U0TXD_GPIO_NUM
#define EXAMPLE_UART_RX_IO_NUM  U0RXD_GPIO_NUM

#define EXAMPLE_UART_WAKEUP_THRESHOLD   3

#define EXAMPLE_READ_BUF_SIZE   1024
#define EXAMPLE_UART_BUF_SIZE   (EXAMPLE_READ_BUF_SIZE * 2)

static const char *TAG_uart = "uart_wakeup";

static QueueHandle_t uart_evt_que = NULL;

static void uart_wakeup_task(void *arg)
{
    uart_event_t event;
    if (uart_evt_que == NULL) {
        ESP_LOGE(TAG_uart, "uart_evt_que is NULL");
        abort();
    }

    uint8_t* dtmp = (uint8_t*) malloc(EXAMPLE_READ_BUF_SIZE);

    while(1) {
        // Waiting for UART event.
        if(xQueueReceive(uart_evt_que, (void * )&event, (TickType_t)portMAX_DELAY)) {
            ESP_LOGI(TAG_uart, "uart%d recved event:%d", EXAMPLE_UART_NUM, event.type);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGI(TAG_uart, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EXAMPLE_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG_uart, "[DATA EVT]:");
                    uart_write_bytes(EXAMPLE_UART_NUM, (const char *)dtmp, event.size);
                    break;
                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG_uart, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EXAMPLE_UART_NUM);
                    xQueueReset(uart_evt_que);
                    break;
                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG_uart, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EXAMPLE_UART_NUM);
                    xQueueReset(uart_evt_que);
                    break;
                // Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG_uart, "uart rx break");
                    break;
                // Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG_uart, "uart parity error");
                    break;
                // Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG_uart, "uart frame error");
                    break;
                // ESP32 can wakeup by uart but there is no wake up interrupt
#if SOC_UART_SUPPORT_WAKEUP_INT
                // Event of waking up by UART
                case UART_WAKEUP:
                    ESP_LOGI(TAG_uart, "uart wakeup");
                    break;
#endif
                default:
                    ESP_LOGI(TAG_uart, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

static esp_err_t uart_initialization(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_REF_TICK,
    };
    //Install UART driver, and get the queue.
    ESP_RETURN_ON_ERROR(uart_driver_install(EXAMPLE_UART_NUM, EXAMPLE_UART_BUF_SIZE, EXAMPLE_UART_BUF_SIZE, 20, &uart_evt_que, 0),
                        TAG_uart, "Install uart failed");
    if (EXAMPLE_UART_NUM == CONFIG_ESP_CONSOLE_UART_NUM) {
        /* temp fix for uart garbled output, can be removed when IDF-5683 done */
        ESP_RETURN_ON_ERROR(uart_wait_tx_idle_polling(EXAMPLE_UART_NUM), TAG_uart, "Wait uart tx done failed");
    }
    ESP_RETURN_ON_ERROR(uart_param_config(EXAMPLE_UART_NUM, &uart_cfg), TAG_uart, "Configure uart param failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(EXAMPLE_UART_NUM, EXAMPLE_UART_TX_IO_NUM, EXAMPLE_UART_RX_IO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG_uart, "Configure uart gpio pins failed");
    return ESP_OK;
}

static esp_err_t uart_wakeup_config(void)
{
    /* UART will wakeup the chip up from light sleep if the edges that RX pin received has reached the threshold
     * Besides, the Rx pin need extra configuration to enable it can work during light sleep */
    ESP_RETURN_ON_ERROR(gpio_sleep_set_direction(EXAMPLE_UART_RX_IO_NUM, GPIO_MODE_INPUT), TAG_uart, "Set uart sleep gpio failed");
    ESP_RETURN_ON_ERROR(gpio_sleep_set_pull_mode(EXAMPLE_UART_RX_IO_NUM, GPIO_PULLUP_ONLY), TAG_uart, "Set uart sleep gpio failed");
    ESP_RETURN_ON_ERROR(uart_set_wakeup_threshold(EXAMPLE_UART_NUM, EXAMPLE_UART_WAKEUP_THRESHOLD),
                        TAG_uart, "Set uart wakeup threshold failed");
    /* Only uart0 and uart1 (if has) support to be configured as wakeup source */
    ESP_RETURN_ON_ERROR(esp_sleep_enable_uart_wakeup(EXAMPLE_UART_NUM),
                        TAG_uart, "Configure uart as wakeup source failed");
    return ESP_OK;
}

esp_err_t register_uart_wakeup(void)
{
    /* Initialize uart1 */
    ESP_RETURN_ON_ERROR(uart_initialization(), TAG_uart, "Initialize uart%d failed", EXAMPLE_UART_NUM);
    /* Enable wakeup from uart */
    ESP_RETURN_ON_ERROR(uart_wakeup_config(), TAG_uart, "Configure uart as wakeup source failed");

    xTaskCreate(uart_wakeup_task, "uart_wakeup_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG_uart, "uart wakeup source is ready");
    return ESP_OK;
}
//-----------------------------------------------------------------------------------------------------------

//----------------------------------- Touch wake up ----------------------------------------------------
static const char *TAG_touch = "touch_wakeup";

#define TOUCH_BUTTON_NUM            5

/* Touch buttons handle */
static touch_button_handle_t button_handle[TOUCH_BUTTON_NUM];


/* Touch buttons channel array */
static const touch_pad_t channel_array[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM1,
    TOUCH_PAD_NUM2,
    TOUCH_PAD_NUM3,
    TOUCH_PAD_NUM4,
    TOUCH_PAD_NUM5,
};

/* Touch buttons channel sensitivity array */
static const float channel_sens_array[TOUCH_BUTTON_NUM] = {
    0.03F,
    0.03F,
    0.03F,
    0.03F,
    0.03F,
};

/* Button event handler task */
static void button_handler_task(void *arg)
{
    (void) arg; //Unused
    touch_elem_message_t element_message;
    while (1) {
        /* Waiting for touch element messages */
        touch_element_message_receive(&element_message, portMAX_DELAY);
        if (element_message.element_type != TOUCH_ELEM_TYPE_BUTTON) {
            continue;
        }
        /* Decode message */
        const touch_button_message_t *button_message = touch_button_get_message(&element_message);
        if (button_message->event == TOUCH_BUTTON_EVT_ON_PRESS) {
            ESP_LOGI(TAG_touch, "Button[%"PRIu32"] Press", (uint32_t)element_message.arg);
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) {
            ESP_LOGI(TAG_touch, "Button[%"PRIu32"] Release", (uint32_t)element_message.arg);
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) {
            ESP_LOGI(TAG_touch, "Button[%"PRIu32"] LongPress", (uint32_t)element_message.arg);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t register_touch_wakeup(void)
{
    /* Initialize Touch Element library */
    touch_elem_global_config_t global_config = TOUCH_ELEM_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_element_install(&global_config));
    ESP_LOGI(TAG_touch, "Touch element library installed");

    touch_button_global_config_t button_global_config = TOUCH_BUTTON_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_button_install(&button_global_config));
    ESP_LOGI(TAG_touch, "Touch button installed");
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_button_config_t button_config = {
            .channel_num = channel_array[i],
            .channel_sens = channel_sens_array[i]
        };
        /* Create Touch buttons */
        ESP_ERROR_CHECK(touch_button_create(&button_config, &button_handle[i]));
        /* Set EVENT as the dispatch method */
        ESP_ERROR_CHECK(touch_button_set_dispatch_method(button_handle[i], TOUCH_ELEM_DISP_EVENT));
        /* Subscribe touch button events (On Press, On Release, On LongPress) */
        ESP_ERROR_CHECK(touch_button_subscribe_event(button_handle[i],
                                                     TOUCH_ELEM_EVENT_ON_PRESS |
                                                     TOUCH_ELEM_EVENT_ON_RELEASE |
                                                     TOUCH_ELEM_EVENT_ON_LONGPRESS,
                                                     (void *)channel_array[i]));
    }
    ESP_LOGI(TAG_touch, "Touch buttons created");

    touch_elem_sleep_config_t sleep_config = {
        .sample_count = global_config.hardware.sample_count,
        .sleep_cycle = global_config.hardware.sleep_cycle,
    };
    /* Enable one of registered touch button as light/deep sleep wake-up source */
    ESP_ERROR_CHECK(touch_element_enable_light_sleep(&sleep_config));

    touch_element_start();
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * 1024, NULL, 6, NULL);
    ESP_LOGI(TAG_touch, "touch wakeup source is ready");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------------------------------
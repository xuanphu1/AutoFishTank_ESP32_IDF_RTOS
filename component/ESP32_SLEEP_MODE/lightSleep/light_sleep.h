/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "touch_element/touch_button.h"
#include "soc/uart_pins.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void wait_gpio_inactive(void);

esp_err_t register_gpio_wakeup(void);

esp_err_t register_timer_wakeup(int);

esp_err_t register_uart_wakeup(void);

esp_err_t register_touch_wakeup(void);

void light_sleep(void);

#ifdef __cplusplus
}
#endif

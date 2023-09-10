/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
// #include "nvs_flash.h"
// #include "nvs.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <inttypes.h>



#if CONFIG_EXAMPLE_GPIO_WAKEUP
void deep_sleep_register_gpio_wakeup(void);
#endif

#if CONFIG_EXAMPLE_EXT0_WAKEUP
void deep_sleep_register_ext0_wakeup(void);
#endif

#if CONFIG_EXAMPLE_EXT1_WAKEUP
void deep_sleep_register_ext1_wakeup(void);
#endif

#if CONFIG_EXAMPLE_TOUCH_WAKEUP
void deep_sleep_register_touch_wakeup(void);
#endif

#ifdef __cplusplus
}
#endif
void deep_sleep(void);

void deep_sleep_register_rtc_timer_wakeup(int);


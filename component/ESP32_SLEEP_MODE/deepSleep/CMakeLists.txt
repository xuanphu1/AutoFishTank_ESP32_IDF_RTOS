idf_component_register(SRCS "deep_sleep.c"
                            "gpio_wakeup.c"
                            "ext_wakeup.c"
                            "touch_wakeup.c"
                            ${app_src}
                            INCLUDE_DIRS "."
                            REQUIRES ${pre_req})

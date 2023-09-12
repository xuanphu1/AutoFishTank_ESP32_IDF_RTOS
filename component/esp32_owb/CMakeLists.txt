idf_component_register(SRCS "owb_gpio.c"
                            "owb_rmt.c"
                            "owb.c"
                            ${app_src}
                            INCLUDE_DIRS "."
                            REQUIRES ${pre_req})

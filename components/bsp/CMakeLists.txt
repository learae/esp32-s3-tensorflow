file(GLOB_RECURSE srcs *.c
                    )

idf_component_register(
    SRCS  ${srcs}
    INCLUDE_DIRS    "."
    REQUIRES    esp_lcd
                lvgl
                esp_timer
                sdmmc
                fatfs
                freertos
                driver
                esp32-camera
)
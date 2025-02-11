# set(requires spi_flash esp_partition mbedtls wpa_supplicant esp_adc esp_eth http_parser esp_ringbuf esp_driver_gptimer esp_driver_usb_serial_jtag driver)
# set(priv_requires fatfs nvs_flash app_update spiffs bootloader_support bt esp_hid usb esp_psram ${ARDUINO_LIBRARIES_REQUIRES})

idf_component_register(
        SRCS
        src/app_main.cpp

        INCLUDE_DIRS
        inc

        REQUIRES
        esp_timer
        driver
        max_bl_flash
        expected
        esp-nimble-cpp
        app_utils
        llcc68
        )

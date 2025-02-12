file(GLOB_RECURSE RADIOLIB_ESP_SOURCES "RadioLib/src/*.cpp")

idf_component_register(
        SRCS
        ${RADIOLIB_ESP_SOURCES}

        INCLUDE_DIRS
        RadioLib/src
        hal

        REQUIRES
        esp_timer
        driver)

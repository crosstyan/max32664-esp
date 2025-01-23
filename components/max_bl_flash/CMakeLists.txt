idf_component_register(
        SRCS
        flash.cpp

        INCLUDE_DIRS
        inc

        EMBED_FILES
        app.msbl

        REQUIRES
        esp_timer
        driver)

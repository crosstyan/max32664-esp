idf_component_register(
        SRCS src/llcc68.cpp src/hal_spi.cpp
        INCLUDE_DIRS inc/
        REQUIRES app_utils etl
)

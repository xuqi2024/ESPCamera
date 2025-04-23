# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_camera)

# 添加必要的组件
idf_build_set_property(COMPILE_DEFINITIONS "-DCONFIG_ESP_WIFI_SSID=\"CMCC-T6Ey\"" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DCONFIG_ESP_WIFI_PASSWORD=\"6fu5k2ue\"" APPEND)

# 设置默认编译目标为ESP32-S3
set(IDF_TARGET "esp32s3")
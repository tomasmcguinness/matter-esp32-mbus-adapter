# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(matter-esp32-mbus-adapter)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)

# main/CHIPProjectConfig.h overrides the vendor/product name shown by Matter
# controllers (see that file). It has to be on every component's include path
# and CHIP_DEVICE_PROJECT_CONFIG_INCLUDE has to be defined everywhere
# CHIPDeviceConfig.h gets compiled -- which happens deep inside the esp_matter
# component, not just in main -- so both are set globally here rather than on
# main's own target.
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_LIST_DIR}/main;-DCHIP_DEVICE_PROJECT_CONFIG_INCLUDE=\"CHIPProjectConfig.h\"" APPEND)

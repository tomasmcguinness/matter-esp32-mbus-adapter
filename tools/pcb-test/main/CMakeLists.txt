# The M-Bus link layer and record decoder are compiled straight from the
# KamstrupMultical403 example rather than copied, so this tool always tests the
# same code the firmware runs.
set(MBUS_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../examples/matter/KamstrupMultical403/main")

idf_component_register(SRCS "main.cpp"
                            "${MBUS_SRC_DIR}/mbus.cpp"
                            "${MBUS_SRC_DIR}/mbus_parser.cpp"
                    INCLUDE_DIRS "." "${MBUS_SRC_DIR}"
                    REQUIRES esp_driver_gpio esp_driver_uart console)

# C++ designated initializers that leave trailing fields zeroed are intended here.
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-missing-field-initializers")

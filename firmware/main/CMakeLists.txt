idf_component_register(SRCS "main.cpp" "mbus.cpp" "mbus_parser.cpp"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")

set(PROJECT_NAME "as-volmeas")

idf_component_register(SRCS "modbus_params.c" "app_main.c"
                        INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

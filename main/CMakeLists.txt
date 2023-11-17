set(priv_requires fatfs console)

if(CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()

idf_component_register(
    SRCS "modbus_params.c" "app_main.c" "tusb_msc.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "${priv_requires}"
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

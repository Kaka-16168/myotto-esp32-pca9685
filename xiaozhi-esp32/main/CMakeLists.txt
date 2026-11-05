idf_component_register(SRCS ${SOURCES}
                    EMBED_FILES ${LANG_SOUNDS} ${COMMON_SOUNDS}
                    INCLUDE_DIRS ${INCLUDE_DIRS}
                    WHOLE_ARCHIVE
                    PRIV_REQUIRES
                        esp_pm
                        esp_psram
                        esp_netif
                        esp_eth
                        esp_driver_gpio
                        esp_driver_uart
                        esp_driver_spi
                        esp_driver_i2c 
                        esp_driver_i2s
                        esp_driver_ledc
                        esp_driver_jpeg
                        esp_driver_ppa
                        esp_driver_tsens
                        esp_app_format
                        app_update
                        esp_https_ota             # 补充 OTA 组件
                        esp_bootloader_format     # 补充 bootloader 头文件提供者，解决 esp_https_ota.c 报错
                        spi_flash
                        mbedtls
                        console
                        efuse
                        bt
                        fatfs
                        ${MAIN_PRIV_REQUIRES_EXTRA}
                    )

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>)
    target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wno-error>)
endif()

# ... 保留中间 board 定义与 assets 构建逻辑 ...

# 兼容 GCC 15 严格指针转换报错
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=int-conversion" APPEND)

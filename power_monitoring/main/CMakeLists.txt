idf_component_register(
    SRCS "power_monitoring.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event nvs_flash mqtt json freertos modbus_master
)

idf_component_register(
    SRCS
        "main.cpp"
        "net/wifi_sta.cpp"
        "net/udp_sender.cpp"
        "sensors/adc_sensors.cpp"
        "sensors/i2c_sensors.cpp"
        "signal/ecg_processor.cpp"
        "telemetry/telemetry.cpp"
        "sensors/i2c_sensors.cpp"
        "third_party/max3010x_algorithms/heartRate.cpp"

    INCLUDE_DIRS
        "include"
        "private"
        "third_party/max3010x_algorithms"

    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_adc
        esp_driver_gpio
        lwip
        esp_timer
        esp_driver_i2c
)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
idf_component_register(
    SRCS
        "app_main.c"
        "net/wifi_sta.c"
        "net/udp_sender.c"
        "sensors/adc_sensors.c"
        "sensors/max30102.c"
        "signal/ecg.c"
        "signal/spo2.c"
        "telemetry/telemetry.c"

    INCLUDE_DIRS
        "include"
        "private"
)
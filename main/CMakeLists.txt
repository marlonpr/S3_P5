idf_component_register(
    SRCS "main.cpp" "clock_settings.cpp"
    INCLUDE_DIRS "."
    REQUIRES led_panel DS3231 DS18B20 nvs_flash
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
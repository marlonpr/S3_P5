idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hub75 led_panel DS3231 DS18B20
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
idf_component_register(SRCS "main.cpp" "src/sensors.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES arduino-esp32 MMC5983MA ISM330DHCX GNSS_Lib)

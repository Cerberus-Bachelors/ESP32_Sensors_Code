set(SPARKFUN_MMC_DIR "SparkFun_MMC5983MA_Magnetometer_Arduino_Library/src")
file(GLOB SOURCES ${SPARKFUN_MMC_DIR}/*.cpp)

idf_component_register( SRCS
                            ${SOURCES}
                        INCLUDE_DIRS  
                            ${SPARKFUN_MMC_DIR}
                        REQUIRES arduino-esp32
                      )
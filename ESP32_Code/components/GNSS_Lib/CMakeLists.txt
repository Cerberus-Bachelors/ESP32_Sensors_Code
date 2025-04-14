set(SPARKFUN_GNSS_DIR "SparkFun_u-blox_GNSS_Arduino_Library/src")
file(GLOB SOURCES ${SPARKFUN_GNSS_DIR}/*.cpp)

idf_component_register( SRCS
                            ${SOURCES}
                        INCLUDE_DIRS  
                            ${SPARKFUN_GNSS_DIR}
                        REQUIRES arduino-esp32
                      )
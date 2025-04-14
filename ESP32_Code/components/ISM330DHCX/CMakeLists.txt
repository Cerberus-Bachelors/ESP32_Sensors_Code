set(SPARKFUN_ISM_DIR "SparkFun_6DoF_ISM330DHCX_Arduino_Library/src")
file(GLOB SOURCES 
        ${SPARKFUN_ISM_DIR}/*.cpp 
        ${SPARKFUN_ISM_DIR}/st_src/*.c)

idf_component_register( SRCS
                            ${SOURCES}
                        INCLUDE_DIRS  
                            ${SPARKFUN_ISM_DIR}
                            ${SPARKFUN_ISM_DIR}/st_src
                        REQUIRES arduino-esp32
                      )
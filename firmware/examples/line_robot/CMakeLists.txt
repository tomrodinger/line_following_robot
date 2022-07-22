set(BSP_COMMON_DIR ${CMAKE_SOURCE_DIR}/bsp/bsp_common)
set(TARGET_REQUIRED_LIBS freertos ble mbedtls)
set(mains main.c)

list(APPEND TARGET_REQUIRED_SRCS sensor.c )
list(APPEND TARGET_REQUIRED_SRCS motor.c )
list(APPEND TARGET_REQUIRED_SRCS robot.c )
list(APPEND TARGET_REQUIRED_SRCS pid.c )
list(APPEND TARGET_REQUIRED_SRCS ble_app.c )

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/bl702_flash_ble.ld)
generate_bin()



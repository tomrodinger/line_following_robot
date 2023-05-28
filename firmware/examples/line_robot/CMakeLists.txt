set(BSP_COMMON_DIR ${CMAKE_SOURCE_DIR}/bsp/bsp_common)
set(TARGET_REQUIRED_LIBS freertos ble mbedtls)
set(mains main.c)

set(TARGET_REQUIRED_PRIVATE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/mp3/)

set(TARGET_REQUIRED_SRCS
	${CMAKE_CURRENT_LIST_DIR}/mp3/bitstream.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/buffers.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/dct32.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/dequant.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/dqchan.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/huffman.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/hufftabs.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/imdct.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/mp3dec.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/mp3tabs.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/polyphase.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/scalfact.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/stproc.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/subband.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3/trigtabs.c
    ${CMAKE_CURRENT_LIST_DIR}/sensor.c
    ${CMAKE_CURRENT_LIST_DIR}/motor.c
    ${CMAKE_CURRENT_LIST_DIR}/robot.c
    ${CMAKE_CURRENT_LIST_DIR}/ir.c
    ${CMAKE_CURRENT_LIST_DIR}/pid.c
    ${CMAKE_CURRENT_LIST_DIR}/ble_app.c
    ${CMAKE_CURRENT_LIST_DIR}/mp3_data.c
    ${CMAKE_CURRENT_LIST_DIR}/audio.c)

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/bl702_flash_ble.ld)
generate_bin()



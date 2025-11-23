# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(APP_NAME MORSE_PROJ) 

set(MAIN_TARGET MORSE_PROJ)
set(FREERTOS_KERNEL_PATH "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel-main")
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")


include(pico_sdk_import.cmake)

project(MORSE_PROJ C CXX ASM)


pico_sdk_init()

include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(libs/TKJHAT)
add_executable(${MAIN_TARGET})

target_link_libraries(${MAIN_TARGET} PRIVATE
  pico_stdlib
  pico_cyw43_arch_none
  pico_lwip
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  TKJHAT_SDK
)


target_sources (${MAIN_TARGET}
    PRIVATE
        src/main.c 
        src/wifi.c
        src/imu.c 
        src/morse_map.c 
        src/morse_utils.c
        src/led.c
)
target_include_directories(${MAIN_TARGET} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
  ${PICO_SDK_PATH}/src/rp2_common/pico_rand/include
  ${PICO_SDK_PATH}/lib/lwip/contrib/ports/freertos/include
  ${PICO_SDK_PATH}/src/rp2_common/pico_stdlib/include
  )




pico_enable_stdio_usb(${MAIN_TARGET} 1)
pico_enable_stdio_uart(${MAIN_TARGET} 0)

pico_add_extra_outputs(${MAIN_TARGET})
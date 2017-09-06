
cmake_minimum_required(VERSION 3.5)
SET(PROJECT_NAME lpc11u_dfu)

set(CMAKE_SYSTEM_NAME Generic CACHE INTERNAL "")

set(CMAKE_SYSTEM_NAME Generic CACHE INTERNAL "")
set(CMAKE_SYSTEM_PROCESSOR ${CORTEX_TYPE} CACHE INTERNAL "")

SET(ARM_COMMON_FLAGS "-mthumb -ffunction-sections -fdata-sections -g3 -fno-common -fmessage-length=0 -Os")
SET(ARM_COMMON_LD_FLAGS "-Os -flto -nostartfiles -Wl,-gc-sections --specs=nano.specs -T ${CMAKE_CURRENT_SOURCE_DIR}/lpc11xx/linkscript.ld")
SET(ASM_OPTIONS "-x assembler-with-cpp")

SET(COMMON_FLAGS "${ARM_COMMON_FLAGS} -mcpu=cortex-m0")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -flto -std=gnu++11 -fno-rtti -fno-exceptions" CACHE INTERNAL "")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -flto -std=gnu99" CACHE INTERNAL "")
set(CMAKE_EXE_LINKER_FLAGS "${ARM_COMMON_LD_FLAGS}" CACHE INTERNAL "")

SET(CMAKE_CXX_COMPILER arm-none-eabi-g++ CACHE INTERNAL "" )
SET(CMAKE_C_COMPILER arm-none-eabi-gcc CACHE INTERNAL "" )
SET(CMAKE_ASM_COMPILER arm-none-eabi-gcc CACHE INTERNAL "")
SET(CMAKE_AR arm-none-eabi-gcc-ar CACHE INTERNAL "")
SET(CMAKE_RANLIB arm-none-eabi-gcc-ranlib CACHE INTERNAL "")
SET(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE INTERNAL "")
SET(CMAKE_OBJSIZE arm-none-eabi-size CACHE INTERNAL "")
SET(CMAKE_OBJDUMP arm-none-eabi-objdump CACHE INTERNAL "")

include_directories(
   inc
   inc/cmsis
   lib
   .
)

SET(SOURCES
  src/main.c
  lpc11xx/cmsis/system_LPC11xx.c
  lpc11xx/crt0_v6m.S
  lpc11xx/LPC11Uxx_handlers.c
  src/lpc11uxx_usbd_dfu.c
  src/dfu_lpc11uxx_usbdesc.c
  src/sbl_iap.c
  #src/print.cpp
  lib/uart.c
)

project(${PROJECT_NAME} CXX C ASM)

add_executable(${PROJECT_NAME} ${SOURCES})

add_custom_command(TARGET lpc11u_dfu POST_BUILD
                        COMMAND ${CMAKE_OBJCOPY}
                        ARGS     -O ihex
                                ${PROJECT_NAME}
                                ${PROJECT_NAME}.hex
                        COMMENT "Generating HEX image ${PROJECT_NAME}.hex"
                        VERBATIM)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                        COMMAND ${CMAKE_OBJCOPY}
                        ARGS     -O binary
                                ${PROJECT_NAME}
                                ${PROJECT_NAME}.bin
                        COMMENT "Generating BIN image ${PROJECT_NAME}.bin"
                        VERBATIM)

add_custom_target(dump_assembly
                        COMMAND arm-none-eabi-strip --strip-debug "${PROJECT_NAME}" -o "${PROJECT_NAME}_nodbg"
                        COMMAND ${CMAKE_OBJDUMP} -C -D "${PROJECT_NAME}_nodbg" > "${PROJECT_NAME}.S"
                        COMMENT "Dumping Assembly listing"
                        VERBATIM)

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND ${CMAKE_OBJSIZE}
                        ARGS    ${PROJECT_NAME}
                        COMMENT "Image Size"
                        VERBATIM)

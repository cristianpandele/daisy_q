cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.20)

project(dubby_q VERSION 0.0.1)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TARGET dubby_q)

set(LIBDAISY_DIR lib/libDaisy)
set(LIBDAISY_CORE_DIR ${LIBDAISY_DIR}/core)
set(Q_DIR lib/q)
set(DUBBY_DIR dubby/DubbyPlayground)

# DaisySP is a CMake project as well, and can be included here...
set(DAISYSP_DIR lib/DaisySP)

# Wavefolder directory
set(WAVEFOLDER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/)

option(Q_BUILD_EXAMPLES "" OFF)
option(Q_BUILD_TEST "" OFF)
option(Q_BUILD_IO "" OFF)

# open-ocd -  might need to be adapted for your environment...
set(CHIPSET stm32h7x)
set(OCD openocd)
set(OCD_DIR /usr/local/share/openocd/scripts)
set(PGM_DEVICE interface/stlink.cfg)
set(OCDFLAGS -f ${PGM_DEVICE} -f target/${CHIPSET}.cfg)
set(OCD_PROGRAM ${OCD} -s ${OCD_DIR} ${OCDFLAGS} -c "program ${TARGET}.elf verify reset exit")

add_subdirectory(${LIBDAISY_DIR})
add_subdirectory(${Q_DIR})
add_subdirectory(${DUBBY_DIR})
add_subdirectory(${DAISYSP_DIR})
add_subdirectory(${WAVEFOLDER_DIR})

add_executable(
  ${TARGET}
    ${WAVEFOLDER_DIR}/main.cpp
)

target_include_directories(
    ${TARGET}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

target_link_libraries(
  ${TARGET}
  PUBLIC
    libq
    DaisySP
    dubby
    wavefolder
)

get_filename_component(LIBDAISY_CORE_DIR_ABS "${LIBDAISY_CORE_DIR}" REALPATH BASE_DIR "${CMAKE_SOURCE_DIR}")

set (INTERNAL_ADDRESS 0x08000000)
set (QSPI_ADDRESS 0x90040000)

# For the time being, we'll keep the Daisy Bootloader's PID as df11
# DAISY_PID = a360
set (DAISY_PID df11)
set (STM_PID df11)


if (${APP_TYPE} STREQUAL "BOOT_NONE")
  set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR_ABS}/STM32H750IB_flash.lds)
  set (USBPID $(STM_PID))
  set (FLASH_ADDRESS $(INTERNAL_ADDRESS))
elseif(${APP_TYPE} STREQUAL "BOOT_SRAM")
  set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR_ABS}/STM32H750IB_sram.lds)
  set (USBPID $(DAISY_PID))
  set (FLASH_ADDRESS $(QSPI_ADDRESS))
  set (BOOT_DEFS -DBOOT_APP)
elseif(${APP_TYPE} STREQUAL "BOOT_QSPI")
  set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR_ABS}/STM32H750IB_qspi.lds)
  set (LDSCRIPT $(SYSTEM_FILES_DIR)/STM32H750IB_qspi.lds)
  set (USBPID $(DAISY_PID))
  set (FLASH_ADDRESS $(QSPI_ADDRESS))
  set (BOOT_DEFS -DBOOT_APP)
else()
  message(FATAL_ERROR "Unkown app type ${APP_TYPE}")
endif()


# set(LINKER_SCRIPT ${LIBDAISY_CORE_DIR_ABS}/STM32H750IB_flash.lds)

set_target_properties(
  ${TARGET}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    SUFFIX ".elf"
    LINK_DEPENDS ${LINKER_SCRIPT}
)

target_link_options(
  ${TARGET}
  PUBLIC
    -T ${LINKER_SCRIPT}
    -Wl,-Map=${TARGET}.map,--cref
    -Wl,--check-sections
    -Wl,--unresolved-symbols=report-all
    -Wl,--warn-common
    -Wl,--warn-section-align
    -Wl,--print-memory-usage
)

target_compile_options(
  ${TARGET}
  PUBLIC
    -Os
    -Wall
    -Wno-attributes
    -Wno-strict-aliasing
    -Wno-maybe-uninitialized
    -Wno-missing-attributes
    -Wno-stringop-overflow
    -Wno-error=reorder
    -Wno-error=sign-compare
    -fexceptions
    -DQ_DONT_USE_THREADS=1
    ${BOOT_DEFS}
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>
)

add_custom_command(
  TARGET ${TARGET}
  POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
    ARGS -O ihex
    -S ${TARGET}.elf
    ${TARGET}.hex
  BYPRODUCTS
    ${TARGET}.hex
    COMMENT "Generating HEX image"
VERBATIM)

add_custom_command(
  TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
    ARGS -O binary
    -S ${TARGET}.elf
    ${TARGET}.bin
    BYPRODUCTS
    ${TARGET}.bin
    COMMENT "Generating binary image"
VERBATIM)

add_custom_target(program
  COMMAND ${OCD_PROGRAM}
)

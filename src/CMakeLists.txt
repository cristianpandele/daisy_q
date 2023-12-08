set(TARGET wavefolder)

add_library(
    ${TARGET}
    WavefolderParallel.cpp
    WavefolderStage.cpp
)

target_include_directories(
    ${TARGET}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/lib/libDaisy/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/lib/libDaisy/Drivers/CMSIS/Device/ST/STM32H7xx/Include/
)

target_link_libraries(
    ${TARGET}
    PUBLIC
    daisy
)
cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME VBCore_PowerBoard)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

set(REGULAR_FIRMWARE ${CMAKE_PROJECT_NAME}_Regular)
set(HIGHPOWER_FIRMWARE ${CMAKE_PROJECT_NAME}_HighPower)
SET(TARGETS_LIST "${REGULAR_FIRMWARE};${HIGHPOWER_FIRMWARE}")

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    add_executable(${TARGET_NAME})
endforeach()

add_subdirectory(cmake/stm32cubemx)

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    target_link_directories(${TARGET_NAME} PRIVATE
        # Add user defined library search paths
    )
endforeach()

file(GLOB_RECURSE USER_SOURCES "user/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp")
foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    target_sources(${TARGET_NAME} PRIVATE
        Core/Src/power.c
        ${USER_SOURCES}
        ${LIBVOLTBRO_SOURCES}
        ${LIBCXXCANARD_SOURCES}
    )
endforeach()

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    target_include_directories(${TARGET_NAME} PRIVATE
        user
        Drivers/libcxxcanard
        Drivers/libcxxcanard/libs
        Drivers/libvoltbro
    )
endforeach()

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    target_compile_definitions(${TARGET_NAME} PRIVATE
    # Add user defined symbols
    )
endforeach()
target_compile_definitions(${HIGHPOWER_FIRMWARE} PUBLIC
    -DHP_board
)

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    target_link_libraries(${TARGET_NAME} PRIVATE
        stm32cubemx
    )
endforeach()

add_compile_options(
    -mfloat-abi=hard   # Use hardware FPU
    -mfpu=fpv4-sp-d16  # For Cortex-M4
)
set(
    CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float"
)

foreach (TARGET_NAME IN LISTS TARGETS_LIST)
    add_custom_command(
        POST_BUILD
        TARGET ${TARGET_NAME}
        COMMAND arm-none-eabi-objcopy -O ihex ${TARGET_NAME}.elf ${TARGET_NAME}.hex
    )
endforeach()

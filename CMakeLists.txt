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

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE USER_SOURCES "user/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/power.c
    ${USER_SOURCES}
    ${LIBVOLTBRO_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    user
    Drivers/libcxxcanard
    Drivers/libcxxcanard/libs
    Drivers/libvoltbro
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

add_compile_options(
    -mfloat-abi=hard   # Use hardware FPU
    -mfpu=fpv4-sp-d16  # For Cortex-M4
)
set(
    CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float"
)

add_custom_command(
    POST_BUILD
    TARGET ${CMAKE_PROJECT_NAME}
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
)

cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H7_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 递归收集 User_File 目录下所有 .c/.cpp 文件
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User_File/*.c
    ${CMAKE_SOURCE_DIR}/User_File/*.cpp
    ${CMAKE_SOURCE_DIR}/App/*.cpp
)

# 递归收集 User_File 目录下所有包含 .h 文件的目录，并去重
file(GLOB_RECURSE USER_HEADER_PATHS CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User_File/*.h
    ${CMAKE_SOURCE_DIR}/App/*.h
)
set(USER_INCLUDE_DIRS "")
foreach(header ${USER_HEADER_PATHS})
    get_filename_component(dir ${header} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# 自动递归收集 User_File 下所有 .h 文件目录
file(GLOB_RECURSE USER_HEADER_PATHS CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User_File/*.h
    ${CMAKE_SOURCE_DIR}/App/*.h
)
set(USER_INCLUDE_DIRS "")
foreach(header ${USER_HEADER_PATHS})
    get_filename_component(dir ${header} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# 全局 include
include_directories(${USER_INCLUDE_DIRS})

# 添加源文件到 target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    # Add user sources here
)

# 添加 include 路径
# 建议：全部用 target_include_directories，且用绝对路径
# 自动递归收集的 include 路径
# 你也可以在下方继续手动添加其他 include 路径

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/Target
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# target_include_directories(USB_Device_Library PRIVATE
#     ${CMAKE_SOURCE_DIR}/User_File/4_Task
# )

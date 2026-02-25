# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)


project(Oscilloscope C CXX ASM)

set(USE_DMA)

# Include Libraries
add_subdirectory(lib/Overclock)
add_subdirectory(lib/Seesaw)
add_subdirectory(lib/ADS704x)
add_subdirectory(lib/DisplayDrivers)
# include_directories(lib/Overclock) # Header only library

# Include Examples
add_subdirectory(Examples/Seesaw)
add_subdirectory(Examples/ADS704x)
add_subdirectory(Examples/Overclock)

# Include Main Code
add_subdirectory(src)

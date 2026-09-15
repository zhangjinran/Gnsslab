###################################################################
#
# gnssLab top-level CMake configuration
#
###################################################################

cmake_minimum_required(VERSION 3.10.2)

project(gnssLab VERSION 1.2)

# Preserve existing build configuration.
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 20)

if(MSVC)
    add_compile_options(/utf-8)
endif()

# Preserve the existing release flags for Stage 3A-1.
# They can be normalized in a later dedicated CMake cleanup.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Keep build artifacts in the same location as the Stage 2 baseline.
# Multi-config generators such as Visual Studio will append Debug/Release.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

option(GNSSLAB_BUILD_EXAMPLES "Build gnssLab examples" ON)

add_subdirectory(lib)

if(GNSSLAB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

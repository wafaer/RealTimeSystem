cmake_minimum_required(VERSION 3.16)
project(RealTimeSystem LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Compiler selection — pass CMAKE_C_COMPILER / CMAKE_CXX_COMPILER on the
# command line or let CMake auto-detect from PATH.
#
#   cmake -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ ...
#
# ---------------------------------------------------------------------------

# Language standards.
set(CMAKE_C_STANDARD           99)
set(CMAKE_CXX_STANDARD         17)
set(CMAKE_C_STANDARD_REQUIRED  ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type default.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Debug flags — AddressSanitizer is enabled only on Linux where it is
# well-supported.  On MinGW/Windows the flags are reduced to basic
# debug symbols.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_C_FLAGS_DEBUG        "-g -O0 -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_DEBUG      "-g -O0 -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-fsanitize=address")
else()
    set(CMAKE_C_FLAGS_DEBUG        "-g -O0")
    set(CMAKE_CXX_FLAGS_DEBUG      "-g -O0")
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
# Boost.Lockfree (used by uspace_rtapi_app.cpp) is header-only, so we only
# need the include path.  find_package(Boost) still ensures Boost is
# installed on the system.
find_package(Boost REQUIRED)

# ---------------------------------------------------------------------------
# Include paths — all source directories are relative to PROJECT_SOURCE_DIR.
# ---------------------------------------------------------------------------
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Source files for the SoemMotroControl executable.
# ---------------------------------------------------------------------------
set(MOTRO_CONTROL_SOURCES
    # ExampleTask — real-time task (C)
    ExampleTask/task.c
    ExampleTask/funct_command.c
    ExampleTask/funct_control.c

    # ExampleTask — C++ RAII lifecycle wrapper
    ExampleTask/operate.cpp

    # Main entry point — process lifecycle manager
    main/taskmain.cpp

    # Stubs for external symbols from the full LinuxCNC codebase

        # HAL — Hardware Abstraction Layer (C)
    hal/hal_lib.c

    # RTAPI — Real-Time API userspace layer
    rtapi/uspace_rtapi_app.cpp
    rtapi/uspace_common.c
)

set(MOTRO_CONTROL_HEADERS
    ExampleTask/task.h
    rtapi/rtapi_common.h
    rtapi/rtapi_mutex.h
    rtapi/rtapi_bitops.h
    rtapi/rtapi_stdint.h
    hal/hal.h
    hal/hal_priv.h
    rtapi/rtapi_uspace.h
    rtapi/uspace_common.h
    rtapi/rtapi.h
        time/timer.h
        time/timer.cpp
        time/_timer.h
        time/_timer.cpp
)

add_executable(SoemMotroControl ${MOTRO_CONTROL_SOURCES} ${MOTRO_CONTROL_HEADERS})

# ---------------------------------------------------------------------------
# Link libraries
# ---------------------------------------------------------------------------
target_link_libraries(SoemMotroControl PRIVATE
    ${Boost_LIBRARIES}   # Boost libraries (usually empty for header-only usage).
    pthread               # POSIX threads.
    rt                    # POSIX real-time extensions (clock_gettime, etc.).
)

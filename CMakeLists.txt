cmake_minimum_required(VERSION 3.16)
project(RealTimeSystem LANGUAGES C CXX)

set(CMAKE_C_STANDARD           99)
set(CMAKE_CXX_STANDARD         17)
set(CMAKE_C_STANDARD_REQUIRED  ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()


if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_C_FLAGS_DEBUG        "-g -O0 -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_DEBUG      "-g -O0 -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-fsanitize=address")
else()
    set(CMAKE_C_FLAGS_DEBUG        "-g -O0")
    set(CMAKE_CXX_FLAGS_DEBUG      "-g -O0")
endif()

find_package(Boost REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
)

set(MOTRO_CONTROL_SOURCES
    ExampleTask/task.c
    ExampleTask/funct_command.c
    ExampleTask/funct_control.c
    ExampleTask/simple_tp.c

    main/taskmain.cpp

    hal/hal_core.c
    hal/hal_shmem.c
    hal/hal_rt.c
    hal/hal_io.c

    rtapi/uspace_rtapi_app.cpp
    rtapi/uspace_common.c
)

set(MOTRO_CONTROL_HEADERS
    ExampleTask/task.h
    ExampleTask/simple_tp.h
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

target_link_libraries(SoemMotroControl PRIVATE
    ${Boost_LIBRARIES}
    pthread
    rt
)

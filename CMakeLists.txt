cmake_minimum_required(VERSION 3.16)
project(RealTimeSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_C_FLAGS_DEBUG "-g -O0 -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fno-omit-frame-pointer -fsanitize=address")

find_package(Boost REQUIRED)

set(SOEM140_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/soem140/include)
set(SOEM140_LIB_DIR     ${PROJECT_SOURCE_DIR}/soem140/lib)
set(SOEM140_LIB         ${SOEM140_LIB_DIR}/libsoem.a)
set(SOCKET_LIB_DIR      ${PROJECT_SOURCE_DIR}/hpsocket/lib)
set(HPSOCKET_LIB        ${SOCKET_LIB_DIR}/libhpsocket4c.a)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${SOEM140_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)

# MotroControl
set(MOTRO_CONTROL_SOURCES
        ExampleTask/task.c
        tp/tp.c
        tp/tc.c
        tp/tcq.c
        ExampleTask/funct_command.c
        kinematics/cubic.cpp
        ExampleTask/funct_control.c
        main/taskmain.cpp
        libnml/_timer.c
        hal/hal_lib.c
        libnml/timer.cpp
        hal/ServoEnable.c
        libnml/emcpos.c
        tp/blendmath.c
        rtapi/uspace_rtapi_app.cpp
        libnml/_posemath.c
        main/watchdog.c
        tp/spherical_arc.c
        rtapi/uspace_common.c
        libnml/posemath.cpp
        main/cleanup.cpp
)

set(MOTRO_CONTROL_HEADERS
        ExampleTask/task.h
        rtapi/rtapi_common.h
        rtapi/rtapi_mutex.h
        rtapi/rtapi_bitops.h
        tp/tp.h
        tp/tp_types.h
        tp/tc.h
        tp/tc_types.h
        ExampleTask/emcmotcfg.h
        tp/tcq.h
        hal/hal.h
        libnml/emcpos.h
        rtapi/rtapi_stdint.h
        kinematics/cubic.h
        hal/hal_priv.h
        libnml/emcglb.h
        libnml/emccfg.h
        ExampleTask/usrmotintf.h
        libnml/emc.h
        libnml/_timer.h
        main/task.h
        libnml/timer.h
        hal/ServoEnable.h
        tp/blendmath.h
        tp/spherical_arc.h
        rtapi/rtapi_uspace.h
        rtapi/uspace_common.h
        rtapi/rtapi.h
        libnml/posemath.h
        hpsocket/include/GlobalDef.h
        hpsocket/include/HPSocket4C.h
        hpsocket/include/HPTypeDef.h
        hal/servo_priv.h
)

add_executable(SoemMotroControl ${MOTRO_CONTROL_SOURCES} ${MOTRO_CONTROL_HEADERS})
target_link_libraries(SoemMotroControl PRIVATE ${SOEM140_LIB} ${HPSOCKET_LIB} ${Boost_LIBRARIES} pthread rt)
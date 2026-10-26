cmake_minimum_required(VERSION 3.10)
project(OS4dynamics CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(
        kernel/global
        kernel/interfaces
        modules/os4
        main/dispOS4
        service/debug
        service/sys
        )

find_package(Threads REQUIRED)

# ядро и модель квадрокоптера
add_library(os4kernel STATIC
        kernel/global/constants.h
        kernel/global/globalNameID.h
        kernel/global/math_func.h
        kernel/interfaces/IMsg.h
        kernel/interfaces/ICalculateElement.h
        kernel/interfaces/ICalculateElement.cpp
        modules/os4/os4Data.h
        modules/os4/os4Init.h
        modules/os4/os4Init.cpp
        modules/os4/os4AeroValue.h
        modules/os4/os4AeroValue.cpp
        modules/os4/os4Gravity.h
        modules/os4/os4Gravity.cpp
        modules/os4/os4Fuselage.h
        modules/os4/os4Fuselage.cpp
        modules/os4/os4Propeller.h
        modules/os4/os4Propeller.cpp
        modules/os4/os4Equations.h
        modules/os4/os4Equations.cpp
        modules/os4/os4Vehicle.h
        modules/os4/os4Vehicle.cpp
        service/debug/report.h
        service/debug/report.cpp
        service/sys/threadManager.h
        service/sys/threadManager.cpp
        service/sys/udpSocket.h
        service/sys/udpSocket.cpp
        service/sys/mppm.h
        service/sys/mppm.cpp
        main/dispOS4/dispOS4.h
        main/dispOS4/dispOS4.cpp
        )

# программа моделирования
add_executable(os4sim main/main.cpp)
if(WIN32)
    target_link_libraries(os4kernel ws2_32)
endif()
target_link_libraries(os4sim os4kernel Threads::Threads)

# проверки модели
enable_testing()
add_executable(os4test test/os4test.cpp)
target_link_libraries(os4test os4kernel Threads::Threads)
add_test(NAME os4test COMMAND os4test)

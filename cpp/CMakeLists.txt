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
        kernel/os4
        main/dispOS4
        service/debug
        service/sys
        )

find_package(Threads REQUIRED)

# ядро и модель квадрокоптера
add_library(os4kernel STATIC
        kernel/global/constants.h
        kernel/global/globalNameID.h
        kernel/global/globalData.h
        kernel/global/math_func.h
        kernel/interfaces/ICalculateElement.h
        kernel/interfaces/ICalculateElement.cpp
        kernel/os4/os4Init.h
        kernel/os4/os4Init.cpp
        kernel/os4/os4AeroValue.h
        kernel/os4/os4AeroValue.cpp
        kernel/os4/os4Gravity.h
        kernel/os4/os4Gravity.cpp
        kernel/os4/os4Fuselage.h
        kernel/os4/os4Fuselage.cpp
        kernel/os4/os4Propeller.h
        kernel/os4/os4Propeller.cpp
        kernel/os4/os4Equations.h
        kernel/os4/os4Equations.cpp
        kernel/os4/os4Vehicle.h
        kernel/os4/os4Vehicle.cpp
        service/debug/report.h
        service/debug/report.cpp
        service/sys/threadManager.h
        service/sys/threadManager.cpp
        main/dispOS4/dispOS4.h
        main/dispOS4/dispOS4.cpp
        )

# программа моделирования
add_executable(os4sim main/main.cpp)
target_link_libraries(os4sim os4kernel Threads::Threads)

# проверки модели
enable_testing()
add_executable(os4test test/os4test.cpp)
target_link_libraries(os4test os4kernel Threads::Threads)
add_test(NAME os4test COMMAND os4test)

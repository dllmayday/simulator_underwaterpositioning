cmake_minimum_required(VERSION 3.10)
project(console_screencast_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core DBus)

add_executable(console_screencast_demo
    main.cpp
    screencast.cpp
    screencast.h
)

target_link_libraries(console_screencast_demo
    Qt5::Core
    Qt5::DBus
)

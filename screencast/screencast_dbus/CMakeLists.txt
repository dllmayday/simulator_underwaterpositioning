cmake_minimum_required(VERSION 3.10)
project(qt_portal_screencast_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets DBus)

add_executable(qt_portal_screencast_demo
    main.cpp
    portalrecorder.cpp
    portalrecorder.h
)

target_link_libraries(qt_portal_screencast_demo
    Qt5::Widgets
    Qt5::DBus
)

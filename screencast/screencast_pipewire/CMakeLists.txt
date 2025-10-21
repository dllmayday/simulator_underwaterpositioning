cmake_minimum_required(VERSION 3.10)
project(console_pipewire_screencast LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core DBus)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)

include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

add_executable(console_pipewire_screencast
    main.cpp
    screencast.cpp
    screencast.h
)

target_link_libraries(console_pipewire_screencast
    Qt5::Core Qt5::DBus
    ${GST_LIBRARIES}
)

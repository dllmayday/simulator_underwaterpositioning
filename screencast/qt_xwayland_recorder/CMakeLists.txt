cmake_minimum_required(VERSION 3.10)
project(QtWaylandXwaylandRecorder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

add_executable(qt_wayland_xwayland_recorder
    main.cpp
    x11recorder.cpp
    x11recorder.h
)

target_link_libraries(qt_wayland_xwayland_recorder
    Qt5::Core Qt5::Gui Qt5::Widgets
    ${GST_LIBRARIES}
    X11
)

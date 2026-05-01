cmake_minimum_required(VERSION 3.16)

project(app VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_add_executable(app
    src/main.cpp
    src/MainWindow.cpp
    include/MainWindow.hpp
)

target_include_directories(app PRIVATE include)
target_link_libraries(app PRIVATE Qt6::Widgets)

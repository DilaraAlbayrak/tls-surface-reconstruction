cmake_minimum_required(VERSION 3.15)
project(TlsSurfaceReconstruction VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    src/main.cpp
)

add_executable(TlsSurfaceReconstruction ${SOURCES})
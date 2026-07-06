cmake_minimum_required(VERSION 3.15)
project(TlsSurfaceReconstruction VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add all source files from the new modular structure
set(SOURCES
    src/main.cpp
    src/PointCloud.cpp
    src/PointCloudIO.cpp
    src/Reconstructor.cpp
)

add_executable(TlsSurfaceReconstruction ${SOURCES})

# Include the headers directory so the compiler can find the .hpp files
target_include_directories(TlsSurfaceReconstruction PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
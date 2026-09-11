cmake_minimum_required(VERSION 3.15)
project(TlsSurfaceReconstruction VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    src/main.cpp
    src/PointCloud.cpp
    src/PointCloudIO.cpp
    src/Reconstructor.cpp
)

add_executable(TlsSurfaceReconstruction ${SOURCES})

# Define the base directory for Open3D once. 
# CACHE PATH allows overriding this via CMake GUI or CLI on different machines.
set(OPEN3D_BASE_DIR "C:/Users/k09100da/Libs/Open3D" CACHE PATH "Base directory for Open3D installation")

set(Open3D_DIR "${OPEN3D_BASE_DIR}/CMake")
find_package(Open3D REQUIRED)

target_include_directories(TlsSurfaceReconstruction PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    "${OPEN3D_BASE_DIR}/include"
)

target_link_libraries(TlsSurfaceReconstruction PRIVATE Open3D::Open3D)

# Copy the Open3D.dll to the executable directory post-build
# Copy required runtime DLLs to the executable directory post-build
add_custom_command(TARGET TlsSurfaceReconstruction POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPEN3D_BASE_DIR}/bin/Open3D.dll"
    $<TARGET_FILE_DIR:TlsSurfaceReconstruction>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPEN3D_BASE_DIR}/bin/tbb12.dll"
    $<TARGET_FILE_DIR:TlsSurfaceReconstruction>
)
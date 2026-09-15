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
file(TO_CMAKE_PATH "$ENV{USERPROFILE}/Libs/Open3D" OPEN3D_DEFAULT_BASE_DIR)
set(OPEN3D_BASE_DIR "${OPEN3D_DEFAULT_BASE_DIR}" CACHE PATH "Base directory for Open3D installation")
set(Open3D_DIR "${OPEN3D_BASE_DIR}/CMake" CACHE PATH "Directory containing Open3DConfig.cmake")

# Visual Studio/CMake keeps old cache values after a project is moved between users.
# If the cached path is stale, fall back to the current user's standard Libs/Open3D path.
if(NOT EXISTS "${Open3D_DIR}/Open3DConfig.cmake" AND
   EXISTS "${OPEN3D_DEFAULT_BASE_DIR}/CMake/Open3DConfig.cmake")
    set(OPEN3D_BASE_DIR "${OPEN3D_DEFAULT_BASE_DIR}" CACHE PATH "Base directory for Open3D installation" FORCE)
    set(Open3D_DIR "${OPEN3D_DEFAULT_BASE_DIR}/CMake" CACHE PATH "Directory containing Open3DConfig.cmake" FORCE)
endif()

find_package(Open3D CONFIG REQUIRED)

# The local Open3D package provides Release import files. Reuse them for Debug-like builds.
foreach(open3d_target IN ITEMS Open3D::Open3D Open3D::tbb)
    if(TARGET ${open3d_target})
        set_target_properties(${open3d_target} PROPERTIES
            MAP_IMPORTED_CONFIG_DEBUG Release
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
            MAP_IMPORTED_CONFIG_MINSIZEREL Release
        )
    endif()
endforeach()

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

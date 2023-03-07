# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.8)
project(spatial_transformation)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# libraries need to be position independent for building Python modules
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors -Werror=return-type)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
# add mpi_cmake_modules first as it provides FindX-files for some libraries
find_package(mpi_cmake_modules REQUIRED)

find_package(serialization_utils REQUIRED)

find_package(cereal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(pybind11 REQUIRED)
find_package(spdlog REQUIRED)

ament_export_dependencies(fmt spdlog vicon-datastream-sdk)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${PROJECT_NAME})


add_library(transformation
    src/transform.cpp
)
target_include_directories(transformation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(transformation
    cereal::cereal
    Eigen3::Eigen
)

add_library(pointcloud
    src/pointcloud.cpp
)
target_include_directories(pointcloud PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pointcloud
    Eigen3::Eigen
)


## Python Bindings
add_pybind11_module(cpp srcpy/bindings.cpp
    LINK_LIBRARIES 
        transformation
)

install(DIRECTORY include/ DESTINATION include)

install(
    TARGETS
        transformation
        pointcloud
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)

    # Python tests
    # Note: The default working directory for tests run by colon is the package
    # root. Python tests that use the C++ bindings need to be run in a different
    # directory, otherwise the local Python source will shadow the installed
    # package and thus the pybind11 modules will be missing.
    ament_add_pytest_test(test_transform_py tests/test_transform.py
        WORKING_DIRECTORY /tmp)

    # C++ tests
    ament_add_gmock(test_transform_cpp
        tests/test_transform.cpp
    )
    target_include_directories(test_transform_cpp PRIVATE include)
    target_link_libraries(test_transform_cpp
        transformation
        fmt::fmt
        serialization_utils::serialization_utils
    )

    ament_add_gmock(test_pointcloud
        tests/test_pointcloud.cpp
    )
    target_include_directories(test_pointcloud PRIVATE include)
    target_link_libraries(test_pointcloud
        pointcloud
    )

endif()


ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

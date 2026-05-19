# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 3.20)
project(gen_goldens CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
pkg_check_modules(EIGEN REQUIRED eigen3)
pkg_check_modules(URDFDOM REQUIRED urdfdom)

add_executable(gen_goldens gen_goldens.cpp)
target_include_directories(gen_goldens PRIVATE
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${URDFDOM_INCLUDE_DIRS}
)
target_compile_options(gen_goldens PRIVATE
    ${PINOCCHIO_CFLAGS_OTHER}
    ${EIGEN_CFLAGS_OTHER}
    -Wno-deprecated-declarations
    -Wno-unused-parameter
)
target_link_libraries(gen_goldens PRIVATE
    ${PINOCCHIO_LDFLAGS}
    ${URDFDOM_LDFLAGS}
)

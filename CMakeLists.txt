cmake_minimum_required(VERSION 3.10)
project(iLQR)

set(CMAKE_CXX_STANDARD 17)

# 引入 Eigen（假设用系统安装的 Eigen）
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 包含头文件目录
include_directories(include)

# 编译源文件
file(GLOB SOURCES src/*.cc models/*.cc)

add_executable(ilqr main.cc ${SOURCES})

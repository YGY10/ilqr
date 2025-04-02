cmake_minimum_required(VERSION 3.10)
project(iLQR)

set(CMAKE_CXX_STANDARD 17)

# 引入 Eigen（假设用系统安装的 Eigen）
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 引入 Python3，自动设置头文件和库
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

# 加入你自己的头文件
include_directories(include models plot)  # plot 是 matplotlibcpp.h 所在位置

# 编译源文件
file(GLOB SOURCES src/*.cc models/*.cc)

# 创建可执行程序
add_executable(ilqr main.cc ${SOURCES})

# 链接 Python 动态库
target_link_libraries(ilqr PRIVATE ${Python3_LIBRARIES})

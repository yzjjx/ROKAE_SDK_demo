cmake_minimum_required(VERSION 3.12)

project(ROKAE_run)

# 设置 C++ 标准，set为设置变量的值的函数，ROKAE官方推荐C++17
set(CMAKE_CXX_STANDARD 17)

# 可执行文件输出到 build/ 根目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 查找库
find_package(Threads REQUIRED)
# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# ROKAE SDK 路径
set(XCORE_SDK_DIR "/home/yzj_robot/xCoreSDK_cpp-v0.6.0")

# 库文件目录
set(XCORE_LIB_DIR "${XCORE_SDK_DIR}/lib/Linux/x86_64")

# 查找 xCoreSDK
find_library(ROKAE_LIB
    NAMES xCoreSDK
    PATHS ${XCORE_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

# 查找 xMateModel
find_library(XMATE_MODEL_LIB
    NAMES xMateModel
    PATHS ${XCORE_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

message(STATUS "ROKAE_LIB = ${ROKAE_LIB}")
message(STATUS "XMATE_MODEL_LIB = ${XMATE_MODEL_LIB}")
message(STATUS "OROCOS_KDL_LIBRARIES = ${OROCOS_KDL_LIBRARIES}")

# 生成可执行程序，目标名字为 ROKAE_rt_tau_out
add_executable(ROKAE_rt_tau_out
    src/ROKAE_rt_tau_out.cpp
)

# 给这个目标添加头文件目录
# 给目标 ROKAE_rt_tau_out 添加头文件搜索路径，也就是源文件存在
# #include "rokae/robot.h"，用这个指定该目标在编译时，要去哪些目录里找头文件
# 头文件目录
target_include_directories(ROKAE_rt_tau_out
    PRIVATE
    ${XCORE_SDK_DIR}/include
    ${OROCOS_KDL_INCLUDE_DIRS}
)

# KDL 的库目录
target_link_directories(ROKAE_rt_tau_out
    PRIVATE
    ${OROCOS_KDL_LIBRARY_DIRS}
)

# find_library之后还需要链接，也就是找到库之后需要把库链接到程序
# PRIVATE表示这个头文件路径只给当前目标自己使用，不会传递给依赖它的别的目标
# 注意顺序：
# 先 xCoreSDK
# 再 xMateModel
# 最后 KDL
target_link_libraries(ROKAE_rt_tau_out
    PRIVATE
    ${ROKAE_LIB}
    ${XMATE_MODEL_LIB}
    Threads::Threads
    Eigen3::Eigen
    ${OROCOS_KDL_LIBRARIES}
)

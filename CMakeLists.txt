###################################################################
#
# CMake配置文件
#
# 参考资料：
#
# https://www.hahack.com/codes/cmake/
#
# Copyright(c)
#
# Shoujian Zhang, Wuhan University, 2020
###################################################################

# 编译该软件时使用的cmake版本
cmake_minimum_required(VERSION 3.10.2)

# 项目名称
project(gnssLab VERSION 1.2)

# 编译选项
# "Debug" 表明，构建系统时给出中间过程提示
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")




# Eigen矩阵库的位置
include_directories( ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0/ )
    #PROJECT_SOURCE_DIR是 CMake自动提供的变量，表示：当前项目的根目录（包含顶层 CMakeLists.txt 的目录）
    #这里的 include（准确说是 include_directories）的作用是：告诉编译器去哪些目录查找头文件（.h / .hpp）

# 添加头文件的路径
include_directories( ${PROJECT_SOURCE_DIR}/lib/ )

# 添加源文件名到变量中
aux_source_directory( ${PROJECT_SOURCE_DIR}/lib/ DIR_LIB_SRCS )

# 根据源文件创建库文件
add_library( gnss SHARED ${DIR_LIB_SRCS}
        lib/Time2020Convert.h
        lib/Time2020Convert.cpp)
    #DIR_LIB_SRCS是我们自己定义的变量。

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

target_link_libraries(gnss ${VTK_LIBRARIES})

target_link_libraries(gnss ${OpenCV_LIBRARIES})   # 加上这一行

# 创建可执行程序
add_executable(parse_opt examples/exam-1.1-parse_opt.cpp)
target_link_libraries(parse_opt gnss)

add_executable(parse_config examples/exam-1.2-parse_config.cpp)
target_link_libraries(parse_config gnss)
#
add_executable(eigen examples/exam-2.1-eigen.cpp)
target_link_libraries(eigen gnss)
#
add_executable(time_convert examples/exam-2.2-time_convert.cpp)
target_link_libraries(time_convert gnss)
#
add_executable(coord_myself examples/exam-2.3-myself.cpp)
target_link_libraries(coord_myself gnss)
#add_executable(coord_convert1 examples/exam-2.3-coord_convert1.cpp)
#target_link_libraries(coord_convert1 gnss)
#
#add_executable(coord_convert2 examples/exam-2.3-coord_convert2.cpp)
#target_link_libraries(coord_convert2 gnss)
#
#add_executable(coord_convert3 examples/exam-2.3-coord_convert3.cpp)
#target_link_libraries(coord_convert3 gnss)
#
#add_executable(coord_convert4 examples/exam-2.3-coord_convert4.cpp)
#target_link_libraries(coord_convert4 gnss)
#
#add_executable(coord_convert5 examples/exam-2.3-coord_convert5.cpp)
#target_link_libraries(coord_convert5 gnss)
#
#
#add_executable(skyplot examples/exam-2.4-skyplot.cpp)
#target_link_libraries(skyplot gnss)
#
add_executable(satid examples/exam-3.1-satid.cpp)
target_link_libraries(satid gnss)
#
add_executable(read_rinex_data examples/exam-3.2-read_rinex_data.cpp)
target_link_libraries(read_rinex_data gnss)
#
add_executable(gps_eph examples/exam-4.1-gps_eph.cpp)
target_link_libraries(gps_eph gnss ${PCL_LIBRARIES})
#
add_executable(system_bias examples/exam-5.1-system_bias.cpp)
target_link_libraries(system_bias gnss)
#
#add_executable(sppif examples/exam-6.1-sppif.cpp)
#target_link_libraries(sppif gnss)
#
#add_executable(cs_detect_mw examples/exam-7.1-cs_detect_mw.cpp)
#target_link_libraries(cs_detect_mw gnss)
#
#add_executable(lambda examples/exam-8.3-lambda.cpp)
#target_link_libraries(lambda gnss)
#
#add_executable(rtk_lsq examples/exam-8.4-rtk_lsq.cpp)
#target_link_libraries(rtk_lsq gnss)
#
#add_executable(rtk_kal examples/exam-8.5-rtk_kal.cpp)
#target_link_libraries(rtk_kal gnss)


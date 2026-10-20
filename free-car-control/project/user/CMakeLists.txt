cmake_minimum_required(VERSION 3.5.0)

# 基础路径与项目名称配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)           # 取上级目录名作为项目名
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

option(TOF_DISTANCE_TEST "Build the standalone terminal ToF distance test mode" OFF)
if(TOF_DISTANCE_TEST)
    add_compile_definitions(TOF_DISTANCE_TEST=1)
endif()

# 编译参数（龙芯LoongArch适配）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 通用编译优化与龙芯架构参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# 交叉编译工具链
include(cross.cmake)

# OpenCV依赖配置
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE: ${OpenCV_INCLUDE_DIRS}, OpenCV_LIBS: ${OpenCV_LIBS}")

#ncnn依赖配置
set(NCNN_ROOT ${CMAKE_SOURCE_DIR}/../../libraries/zf_components/ncnn)
set(NCNN_INCLUDE_DIR ${NCNN_ROOT}/include)
set(NCNN_LIBRARY ${NCNN_ROOT}/lib/libncnn.a)

# 头文件路径汇总
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    # 业务代码头文件
    ../user 
    ../code 
    # ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components/seekfree_assistant

    # # TFLM核心头文件（完全拆分）
    # ../../libraries/zf_components/tflm
    # ../../libraries/zf_components/tflm/third_party/flatbuffers/include
    # ../../libraries/zf_components/tflm/third_party/gemmlowp
    # ../../libraries/zf_components/tflm/third_party/gemmlowp/fixedpoint
    # ../../libraries/zf_components/tflm/third_party/ruy
    # ../../libraries/zf_components/tflm/third_party/ruy/ruy
    # ../../libraries/zf_components/tflm/tensorflow/lite/micro
    # ../../libraries/zf_components/tflm/tensorflow/lite/micro/kernels
    # ../../libraries/zf_components/tflm/tensorflow/lite/kernels/internal
    # ../../libraries/zf_components/tflm/tensorflow/lite/core/api
    # ../../libraries/zf_components/tflm/tensorflow/lite/core/c
    # ../../libraries/zf_components/tflm/tensorflow/lite/schema

    ../../libraries/zf_components/ncnn/include/ncnn

    ${NCNN_INCLUDE_DIR}   
)

# 静态库路径
link_directories(
    # ../../libraries/zf_components/tflm/lib
    ../../libraries/zf_components/ncnn/lib
)

# 业务源文件
aux_source_directory(../user DIR_SRCS)
aux_source_directory(../code DIR_SRCS)
# aux_source_directory(../model DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components/seekfree_assistant DIR_SRCS)

# 生成可执行文件 & 配置属性
add_executable(${PROJ_NAME} ${DIR_SRCS})
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 链接依赖库
target_link_libraries(
${PROJ_NAME} 
${OpenCV_LIBS}

${NCNN_LIBRARY}

# tflm          # 链接TFLM静态库（必须）
pthread       # 线程库
m             # 数学库
)

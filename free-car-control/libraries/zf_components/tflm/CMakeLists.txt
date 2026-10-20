cmake_minimum_required(VERSION 3.5.0)
project(tflm_static_lib)

# ===================== 编译参数（龙芯LoongArch64适配）=====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

# 龙芯编译优化参数（和业务工程保持一致）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# 交叉编译工具链（复用业务工程的配置）
include(${CMAKE_CURRENT_SOURCE_DIR}/cross.cmake)

# ===================== 头文件路径（相对tflm目录）=====================
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/flatbuffers/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/gemmlowp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/gemmlowp/fixedpoint)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/ruy)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/ruy/ruy)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/kernels)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/kernels/internal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/core/api)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/core/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/schema)

# ===================== TfLM源文件配置 =====================
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/compiler/mlir/lite/core/api DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/compiler/mlir/lite/schema DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/core/api DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/core/c DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/kernels DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/kernels/internal DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/cortex_m_generic DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/arena_allocator DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/memory_planner DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/tflite_bridge DIR_TFLM_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/micro/kernels DIR_TFLM_SRCS)

# ===================== 编译静态库 =====================
add_library(tflm STATIC ${DIR_TFLM_SRCS})

# 设置静态库输出路径（tflm目录下的lib文件夹）
set_target_properties(tflm PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
    OUTPUT_NAME "tflm"
    CLEAN_DIRECT_OUTPUT 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    C_STANDARD 99
)

# 链接必要的系统库
target_link_libraries(tflm pthread m)
# =========================================================
# 数学库
#
# 依赖：
#   无
#
# 能力：
#   向量
#   矩阵
#   四元数
#   坐标系
# =========================================================


# 收集当前目录下的数学库源文件。
file(GLOB MATH_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)


# 创建数学静态库。
add_library(MyMath STATIC
    ${MATH_SRC}
)


# 设置公共头文件搜索路径。
target_include_directories(MyMath PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)


# 在IDE中保持源码目录结构。
source_group(TREE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        ${MATH_SRC}
)


# 设置IDE工程分类。
set_target_properties(MyMath PROPERTIES
    FOLDER "MyVoxel"
)
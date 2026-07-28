# =========================================================
# 体素核心库
#
# 能力：
#   基础类型
#   几何
#   层级
#   存储
#   森林
#   算法
# =========================================================

file(GLOB_RECURSE VOXEL_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

add_library(MyVoxel STATIC
    ${VOXEL_SRC}
)

target_link_libraries(MyVoxel PUBLIC
    MyMath
)

target_include_directories(MyVoxel PUBLIC
    "${PROJECT_SOURCE_DIR}"
)

source_group(TREE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        ${VOXEL_SRC}
)

set_target_properties(MyVoxel PROPERTIES
    FOLDER "MyVoxel"
)
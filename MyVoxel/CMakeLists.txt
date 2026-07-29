# =========================================================
# 体素核心库
# =========================================================

file(GLOB_RECURSE VOXEL_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# =========================================================
# 正式库
#
# 不定义MYVOXEL_TEST，不包含性能统计代码。
# =========================================================

add_library(MyVoxel STATIC
    ${VOXEL_SRC}
)

target_link_libraries(MyVoxel PUBLIC
    MyMath
)

target_include_directories(MyVoxel PUBLIC
    "${PROJECT_SOURCE_DIR}"
)

set_target_properties(MyVoxel PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
    FOLDER "MyVoxel"
)

# =========================================================
# 测试库
#
# 使用相同源码重新编译，并定义MYVOXEL_TEST。
# 布尔运算统计接口和统计实现只在该目标中生效。
# =========================================================

if(MYVOXEL_BUILD_SANDBOX)
    add_library(MyVoxelTest STATIC
        ${VOXEL_SRC}
    )

    target_compile_definitions(MyVoxelTest PUBLIC
        MYVOXEL_TEST
    )

    target_link_libraries(MyVoxelTest PUBLIC
        MyMath
    )

    target_include_directories(MyVoxelTest PUBLIC
        "${PROJECT_SOURCE_DIR}"
    )

    set_target_properties(MyVoxelTest PROPERTIES
        AUTOMOC OFF
        AUTOUIC OFF
        AUTORCC OFF
        FOLDER "MyVoxel"
    )
endif()

# =========================================================
# Visual Studio源码分组
# =========================================================

source_group(TREE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        ${VOXEL_SRC}
)
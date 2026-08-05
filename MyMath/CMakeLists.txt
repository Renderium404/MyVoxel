# =========================================================
# MyMath 数学库
#
# 依赖：
#   无
#
# 能力：
#   基础数学函数
#   三维向量
#   三阶矩阵
#   四阶矩阵
#   四元数
#   三维坐标系
# =========================================================

# =========================================================
# 源文件
# =========================================================

# 收集数学库实现文件。
file(GLOB MYMATH_SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)

# 收集数学库头文件。
file(GLOB MYMATH_HEADER_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

# =========================================================
# 数学库目标
# =========================================================

# 创建纯C++数学静态库。
add_library(MyMath STATIC
    ${MYMATH_SOURCE_FILES}
    ${MYMATH_HEADER_FILES}
)

# 提供带命名空间的目标别名。
add_library(MySimulation::MyMath ALIAS MyMath)

# 对外公开数学库头文件目录。
target_include_directories(MyMath PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# 设置数学库编译属性。
set_target_properties(MyMath PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
    FOLDER "Libraries"
)

# =========================================================
# IDE工程组织
# =========================================================

source_group("Source Files" FILES
    ${MYMATH_SOURCE_FILES}
)

source_group("Header Files" FILES
    ${MYMATH_HEADER_FILES}
)
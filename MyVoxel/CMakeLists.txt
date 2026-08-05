# =========================================================
# 体素核心库源码收集
# =========================================================

# 递归收集当前目录及其所有子目录中的 .cpp 和 .h 文件。
#
# CMAKE_CURRENT_SOURCE_DIR：
# 当前 CMakeLists.txt 所在目录。
#
# GLOB_RECURSE：
# 不仅搜索当前目录，还会递归搜索所有子目录。
#
# CONFIGURE_DEPENDS：
# 当匹配范围内新增或删除源码文件时，CMake 会尝试自动重新配置工程，
# 从而更新 VOXEL_SRC 文件列表。
#
# 注意：
# 使用 GLOB_RECURSE 后，新增源码文件通常不需要手动修改此 CMakeLists.txt，
# 但文件是否被正确归入目标仍取决于其所在目录和扩展名。
file(GLOB_RECURSE VOXEL_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# =========================================================
# 正式体素库
#
# MyVoxel 是正式版本使用的静态库。
#
# 正式库不定义以下宏：
#
#   MYVOXEL_TEST
#   MYVOXEL_ENABLE_OPERATION_STATISTICS
#
# 因此，通过这些宏控制的测试接口、测试辅助代码和性能统计代码，
# 不会被编译进正式版本。
# =========================================================

# 使用 VOXEL_SRC 中的全部源码创建静态库 MyVoxel。
#
# STATIC 表示生成静态链接库：
#
#   Windows + MSVC：通常生成 MyVoxel.lib
#
# 使用 MyVoxel 的可执行程序会在链接阶段将所需代码链接进最终程序。
add_library(MyVoxel STATIC
    ${VOXEL_SRC}
)

# 声明 MyVoxel 依赖 MyMath。
#
# PUBLIC 表示：
#
# 1. 编译 MyVoxel 本身时需要 MyMath；
# 2. 链接 MyVoxel 的下游目标也会自动继承 MyMath 依赖。
#
# 例如：
#
#   target_link_libraries(SomeApplication PRIVATE MyVoxel)
#
# 此时 SomeApplication 会同时获得 MyVoxel 和 MyMath 的链接依赖，
# 不需要再次显式链接 MyMath。
target_link_libraries(MyVoxel PUBLIC
    MyMath
)

# 设置 MyVoxel 对外公开的头文件搜索路径。
#
# PROJECT_SOURCE_DIR：
# 工程最顶层 project() 所在目录。
#
# PUBLIC 表示：
#
# 1. 编译 MyVoxel 自身时使用该头文件路径；
# 2. 链接 MyVoxel 的下游目标也会继承该头文件路径。
#
# 因此，其他模块可以使用从项目根目录开始的包含方式，例如：
#
#   #include "MyVoxel/Core/VoxelShape.h"
target_include_directories(MyVoxel PUBLIC
    "${PROJECT_SOURCE_DIR}"
)

# 设置 MyVoxel 目标属性。
#
# AUTOMOC OFF：
# 禁止 Qt 元对象编译器自动处理。
#
# AUTOUIC OFF：
# 禁止 Qt 自动处理 .ui 文件。
#
# AUTORCC OFF：
# 禁止 Qt 自动处理 .qrc 资源文件。
#
# MyVoxel 是纯 C++ 体素库，不依赖 Qt，因此明确关闭这些功能。
#
# FOLDER "MyVoxel"：
# 在支持目标文件夹的 IDE 中，将该目标放入 MyVoxel 分组。
# 在 Visual Studio 解决方案中，MyVoxel 项目会显示在 MyVoxel 文件夹下。
set_target_properties(MyVoxel PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
    FOLDER "MyVoxel"
)

# =========================================================
# 测试体素库
#
# MyVoxelTest 使用与正式库 MyVoxel 完全相同的源码重新编译，
# 但会额外定义测试宏和性能统计宏。
#
# 单独建立测试库的主要原因是：
#
# 1. 正式库保持纯净，不包含测试接口和统计代码；
# 2. 测试版本可以访问仅在测试宏下开放的接口；
# 3. 测试程序和静态库使用完全一致的预处理宏；
# 4. 避免头文件声明和静态库实现因宏不一致而产生 ABI 或链接问题。
#
# MyVoxelTest 仅在 MYVOXEL_BUILD_SANDBOX 为真时创建。
# =========================================================

# 判断是否启用 MyVoxel 沙盒和测试目标。
#
# MYVOXEL_BUILD_SANDBOX 通常应在顶层 CMakeLists.txt 中定义，例如：
#
#   option(MYVOXEL_BUILD_SANDBOX "Build MyVoxel sandbox tests." OFF)
#
# 配置工程时可以通过命令行启用：
#
#   cmake -DMYVOXEL_BUILD_SANDBOX=ON ...
#
# 当该选项为 OFF 时：
#
#   不创建 MyVoxelTest；
#   不编译测试专用版本；
#   通常也不创建相关测试可执行程序。
if(MYVOXEL_BUILD_SANDBOX)

    # 使用与正式库相同的 VOXEL_SRC 创建测试静态库。
    #
    # MyVoxel 和 MyVoxelTest 是两个独立目标。
    # 即使源码文件相同，它们也会分别执行一次编译。
    #
    # 这样可以保证：
    #
    #   MyVoxel     使用正式宏配置；
    #   MyVoxelTest 使用测试宏配置。
    add_library(MyVoxelTest STATIC
        ${VOXEL_SRC}
    )

    # 为 MyVoxelTest 定义预处理宏。
    #
    # 这不是在源码中直接插入：
    #
    #   #define MYVOXEL_TEST
    #   #define MYVOXEL_ENABLE_OPERATION_STATISTICS
    #
    # 而是向编译器传递对应的宏定义参数。
    #
    # MSVC 编译命令中通常表现为：
    #
    #   /DMYVOXEL_TEST
    #   /DMYVOXEL_ENABLE_OPERATION_STATISTICS
    #
    # PUBLIC 表示这些宏同时作用于：
    #
    # 1. MyVoxelTest 自身源码；
    # 2. 链接 MyVoxelTest 的下游测试目标。
    #
    # 例如：
    #
    #   target_link_libraries(VoxelTest PRIVATE MyVoxelTest)
    #
    # VoxelTest 在编译时也会自动定义这两个宏。
    #
    # 这样可以确保测试程序看到的头文件声明，
    # 与 MyVoxelTest 静态库中的实际实现保持一致。
    target_compile_definitions(MyVoxelTest PUBLIC
        MYVOXEL_TEST
        MYVOXEL_ENABLE_OPERATION_STATISTICS
    )

    # 声明测试库依赖 MyMath。
    #
    # PUBLIC 的传播规则与正式库相同：
    #
    #   MyVoxelTest 自身链接 MyMath；
    #   链接 MyVoxelTest 的测试程序也会继承 MyMath。
    target_link_libraries(MyVoxelTest PUBLIC
        MyMath
    )

    # 设置测试库公开的头文件搜索路径。
    #
    # 测试程序链接 MyVoxelTest 后，也会自动获得项目根目录头文件路径。
    target_include_directories(MyVoxelTest PUBLIC
        "${PROJECT_SOURCE_DIR}"
    )

    # 设置测试库目标属性。
    #
    # 测试库同样是纯 C++ 库，因此关闭 Qt 自动处理功能。
    #
    # FOLDER "MyVoxel"：
    # 在 Visual Studio 中将 MyVoxelTest 与 MyVoxel 放入同一目标文件夹。
    set_target_properties(MyVoxelTest PROPERTIES
        AUTOMOC OFF
        AUTOUIC OFF
        AUTORCC OFF
        FOLDER "MyVoxel"
    )
endif()

# =========================================================
# Visual Studio 源码分组
# =========================================================

# 按照磁盘上的真实目录结构，在 Visual Studio 中组织源码文件。
#
# TREE：
# 指定源码目录树的根目录。
#
# FILES：
# 指定需要进行分组的文件列表。
#
# 例如磁盘目录：
#
#   MyVoxel/
#       Core/
#           VoxelShape.cpp
#       Modeling/
#           VoxelizationWorkspace.cpp
#
# 在 Visual Studio 解决方案资源管理器中会显示为：
#
#   Core
#       VoxelShape.cpp
#   Modeling
#       VoxelizationWorkspace.cpp
#
# source_group() 只影响 IDE 中的显示方式，
# 不影响实际编译、头文件搜索或链接行为。
source_group(TREE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        ${VOXEL_SRC}
)


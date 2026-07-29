cmake_minimum_required(VERSION 3.21)

project(MyVoxel LANGUAGES CXX)

# =========================================================
# 全局编译配置
# =========================================================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 在Visual Studio中按模块显示目标。
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(MSVC)
    add_compile_definitions(
        NOMINMAX
        _CRT_SECURE_NO_WARNINGS
    )
endif()

# 是否编译SandBox测试程序。
option(MYVOXEL_BUILD_SANDBOX
    "Build MyVoxel sandbox test programs."
    ON
)

# 是否编译Qt OpenGL显示模块。
option(MYVOXEL_BUILD_VIEWER
    "Build the Qt OpenGL MyVoxel viewer."
    ON
)

# =========================================================
# Qt
# =========================================================

if(MYVOXEL_BUILD_VIEWER)
    find_package(Qt5 REQUIRED COMPONENTS
        Core
        Gui
        Widgets
    )
endif()

# =========================================================
# 子模块
# =========================================================
    
add_subdirectory(MyMath)
add_subdirectory(MyVoxel)

# MyMath为纯C++模块。
if(TARGET MyMath)
    set_target_properties(MyMath PROPERTIES
        AUTOMOC OFF
        AUTOUIC OFF
        AUTORCC OFF
        FOLDER "MyVoxel"
    )
endif()

# MyVoxel为纯C++模块。
if(TARGET MyVoxel)
    set_target_properties(MyVoxel PROPERTIES
        AUTOMOC OFF
        AUTOUIC OFF
        AUTORCC OFF
        FOLDER "MyVoxel"
    )
endif()
# =========================================================
# Qt OpenGL可视化模块
#
# 依赖：
#   MyVoxel
#   Qt
#
# 能力：
#   将标准连续Shape转换为显示网格
#   将体素表面网格转换为统一显示网格
#   将VoxelShape材料节点转换为GPU显示实例
#   显示标准Shape、体素表面和体素实例
# =========================================================

# if(MYVOXEL_BUILD_VIEWER)

#     # -----------------------------------------------------
#     # OpenGL显示数据
#     # -----------------------------------------------------

#     set(MYVOXEL_VIEWER_OPENGL_SRC
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/DisplayColor.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/DisplayColor.h"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/DisplayMesh.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/DisplayMesh.h"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/VoxelDisplayInstance.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/VoxelDisplayInstance.h"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/VoxelInstanceCollector.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/OpenGL/VoxelInstanceCollector.h"
#     )

#     # -----------------------------------------------------
#     # 显示数据构建
#     # -----------------------------------------------------

#     set(MYVOXEL_VIEWER_BUILDER_SRC
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Builder/ShapeDisplayBuilder.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Builder/ShapeDisplayBuilder.h"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Builder/VoxelSurfaceDisplayBuilder.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Builder/VoxelSurfaceDisplayBuilder.h"
#     )

#     # -----------------------------------------------------
#     # 显示控件
#     # -----------------------------------------------------

#     set(MYVOXEL_VIEWER_WIDGET_SRC
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Widget/VoxelInstanceViewWidget.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Widget/VoxelInstanceViewWidget.h"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Widget/VoxelMeshViewWidget.cpp"
#         "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/Widget/VoxelMeshViewWidget.h"
#     )

#     set(MYVOXEL_VIEWER_SRC
#         ${MYVOXEL_VIEWER_OPENGL_SRC}
#         ${MYVOXEL_VIEWER_BUILDER_SRC}
#         ${MYVOXEL_VIEWER_WIDGET_SRC}
#     )

#     add_library(MyVoxelOpenGLViewer STATIC
#         ${MYVOXEL_VIEWER_SRC}
#     )

#     add_library(MyVoxel::Viewer ALIAS MyVoxelOpenGLViewer)

#     target_include_directories(MyVoxelOpenGLViewer PUBLIC
#         "${CMAKE_CURRENT_SOURCE_DIR}"
#     )

#     target_link_libraries(MyVoxelOpenGLViewer PUBLIC
#         MyVoxel
#         Qt5::Core
#         Qt5::Gui
#         Qt5::Widgets
#     )

#     set_target_properties(MyVoxelOpenGLViewer PROPERTIES
#         AUTOMOC OFF
#         AUTOUIC OFF
#         AUTORCC OFF
#         FOLDER "Viewer"
#     )

#     source_group(
#         TREE "${CMAKE_CURRENT_SOURCE_DIR}/Viewer"
#         PREFIX "Viewer"
#         FILES ${MYVOXEL_VIEWER_SRC}
#     )

# endif()
    
# =========================================================
# SandBox测试
#
# 普通测试：
#   链接MyVoxel
#
# 性能统计测试：
#   链接MyVoxelTest
#
# Viewer测试：
#   链接MyVoxel::Viewer
# =========================================================

if(MYVOXEL_BUILD_SANDBOX)

    # 这些测试需要MYVOXEL_TEST和带性能统计的MyVoxelTest库。
    set(MYVOXEL_STATISTICS_SANDBOX_TARGETS
        boolean_operation_statistics_test
        voxel_boolean_cut_test
        voxel_cut_performance_test
    )

    file(GLOB SANDBOX_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/SandBox/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/SandBox/*.cxx"
    )

    # 这些测试需要Qt OpenGL可视化模块。
    # set(MYVOXEL_VIEWER_SANDBOX_TARGETS
    #     voxel_display_test
    #     shape_display_test
    #     voxel_surface_display_test
    # )

    foreach(SOURCE_FILE IN LISTS SANDBOX_SRC)
        get_filename_component(SOURCE_NAME "${SOURCE_FILE}" NAME_WE)
        string(MAKE_C_IDENTIFIER "${SOURCE_NAME}" TARGET_NAME)

        if(TARGET ${TARGET_NAME})
            message(FATAL_ERROR
                "Duplicate sandbox target name: ${TARGET_NAME}\n"
                "Source: ${SOURCE_FILE}"
            )
        endif()

        list(FIND MYVOXEL_VIEWER_SANDBOX_TARGETS "${TARGET_NAME}" VIEWER_TARGET_INDEX)
        list(FIND MYVOXEL_STATISTICS_SANDBOX_TARGETS "${TARGET_NAME}" STATISTICS_TARGET_INDEX)

        if(VIEWER_TARGET_INDEX GREATER -1 AND NOT MYVOXEL_BUILD_VIEWER)
            message(STATUS "Skip sandbox: ${TARGET_NAME}, viewer is disabled.")
            continue()
        endif()

        add_executable(${TARGET_NAME}
            "${SOURCE_FILE}"
        )

        target_include_directories(${TARGET_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}"
        )

        if(VIEWER_TARGET_INDEX GREATER -1)
            target_link_libraries(${TARGET_NAME} PRIVATE
                MyVoxel::Viewer
            )
        elseif(STATISTICS_TARGET_INDEX GREATER -1)
            target_link_libraries(${TARGET_NAME} PRIVATE
                MyVoxelTest
            )
        else()
            target_link_libraries(${TARGET_NAME} PRIVATE
                MyVoxel
            )
        endif()

        set_target_properties(${TARGET_NAME} PROPERTIES
            FOLDER "SandBox"
            VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TARGET_NAME}>"
        )

        message(STATUS "Sandbox: ${TARGET_NAME}")
    endforeach()

endif()
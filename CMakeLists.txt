cmake_minimum_required(VERSION 3.21)

project(MyVoxel LANGUAGES CXX)

# =========================================================
# 全局编译配置
# =========================================================

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# =========================================================
# Qt
# =========================================================

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
)

# =========================================================
# 核心库
# =========================================================

add_subdirectory(MyMath)
add_subdirectory(MyVoxel)

# MyMath为纯C++模块。
if(TARGET MyMath)
    set_target_properties(MyMath PROPERTIES
        FOLDER "MyVoxel"
    )
endif()

# MyVoxel为纯C++模块。

# MyVoxel为纯C++模块。
if(TARGET MyVoxel)
    set_target_properties(MyVoxel PROPERTIES FOLDER "MyVoxel")
endif()

# =========================================================
# OpenGL Viewer
#
# Viewer只负责Qt/OpenGL显示，不固定链接正式版或测试版MyVoxel，
# 最终由具体可执行程序负责链接需要的MyVoxel版本。
# =========================================================

file(GLOB_RECURSE VIEWER_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Viewer/*.hpp"
)

if(VIEWER_SRC)
    add_library(Viewer STATIC
        ${VIEWER_SRC}
    )

    target_include_directories(Viewer PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/Viewer"
    )

    target_link_libraries(Viewer PUBLIC
        MyMath
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
    )

    set_target_properties(Viewer PROPERTIES
        FOLDER "MyVoxel/Viewer"
        AUTOMOC ON
        AUTOUIC OFF
        AUTORCC OFF
    )

    source_group(
        TREE "${CMAKE_CURRENT_SOURCE_DIR}/Viewer"
        PREFIX "Viewer"
        FILES ${VIEWER_SRC}
    )
endif()

# =========================================================
# SandBox测试
#
# SandBox目录下每个cpp或cxx文件生成一个独立可执行程序。
#
# 默认链接正式MyVoxel库。
# 文件名包含_qt时，额外链接Viewer和Qt。
# =========================================================

if(MYVOXEL_BUILD_SANDBOX)

    file(GLOB SANDBOX_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/SandBox/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/SandBox/*.cxx"
    )

    foreach(SOURCE_FILE IN LISTS SANDBOX_SRC)
        get_filename_component(
            SOURCE_NAME
            "${SOURCE_FILE}"
            NAME_WE
        )

        string(
            MAKE_C_IDENTIFIER
            "${SOURCE_NAME}"
            TARGET_NAME
        )

        if(TARGET ${TARGET_NAME})
            message(FATAL_ERROR
                "Duplicate sandbox target name: ${TARGET_NAME}\n"
                "Source: ${SOURCE_FILE}"
            )
        endif()

        add_executable(${TARGET_NAME}
            "${SOURCE_FILE}"
        )

        target_include_directories(${TARGET_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}"
        )

        target_link_libraries(${TARGET_NAME} PRIVATE
            MyVoxel
        )

        # 文件名包含_qt的程序需要Qt和OpenGL Viewer。
        string(FIND "${SOURCE_NAME}" "_qt" QT_NAME_POSITION)

        if(NOT QT_NAME_POSITION EQUAL -1)
            if(NOT TARGET Viewer)
                message(FATAL_ERROR
                    "Qt sandbox target requires Viewer, but no Viewer source files were found.\n"
                    "Target: ${TARGET_NAME}"
                )
            endif()

            target_link_libraries(${TARGET_NAME} PRIVATE
                Viewer
                Qt5::Core
                Qt5::Gui
                Qt5::Widgets
            )

            set_target_properties(${TARGET_NAME} PROPERTIES
                AUTOMOC ON
                AUTOUIC OFF
                AUTORCC OFF
            )
        else()
            set_target_properties(${TARGET_NAME} PROPERTIES
                AUTOMOC OFF
                AUTOUIC OFF
                AUTORCC OFF
            )
        endif()

        set_target_properties(${TARGET_NAME} PROPERTIES
            FOLDER "SandBox"
            VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TARGET_NAME}>"
        )

        message(STATUS
            "Sandbox: ${TARGET_NAME}"
        )
    endforeach()

endif()
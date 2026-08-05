cmake_minimum_required(VERSION 3.21)

project(MyVoxel LANGUAGES CXX)

# =========================================================
# 全局编译配置
# =========================================================

set(CMAKE_CXX_STANDARD 14)
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
if(TARGET MyVoxel)
    set_target_properties(MyVoxel PROPERTIES
        FOLDER "MyVoxel"
    )
endif()

# =========================================================
# SandBox测试
#
# SandBox目录下每个cpp或cxx文件生成一个独立可执行程序，
# 所有程序统一链接正式MyVoxel库。
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

        set_target_properties(${TARGET_NAME} PROPERTIES
            FOLDER "SandBox"
            VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TARGET_NAME}>"
        )

        message(STATUS
            "Sandbox: ${TARGET_NAME}"
        )
    endforeach()

endif()
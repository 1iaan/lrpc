cmake_minimum_required(VERSION 3.10)
project(network VERSION 1.0.0 LANGUAGES CXX)

# ============================================================
# 编译选项
# ============================================================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

# add_compile_options(-Wall -Wno-deprecated -Wno-unused-but-set-variable)

# ============================================================
# 输出目录
# ============================================================
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmacro-prefix-map=${CMAKE_SOURCE_DIR}=.")

# ============================================================
# 子目录
# ============================================================
add_subdirectory(lrpc)
add_subdirectory(testcases)

# ============================================================
# 安装规则
# ============================================================
# 头文件
install(DIRECTORY ${CMAKE_SOURCE_DIR}/lrpc/common/
        DESTINATION /usr/include/lrpc/common
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/lrpc/net/
        DESTINATION /usr/include/lrpc/net
        FILES_MATCHING PATTERN "*.h")
# 库文件
install(TARGETS lrpc
        ARCHIVE DESTINATION /usr/lib)

# ============================================================
# 自定义 clean 命令
# ============================================================
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "bin;lib;obj")

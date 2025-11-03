# lrpc/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# ============================================================
# 源文件
# ============================================================
set(PATH_COMM ${CMAKE_CURRENT_SOURCE_DIR}/common)
set(PATH_NET  ${CMAKE_CURRENT_SOURCE_DIR}/net)
set(PATH_TCP  ${CMAKE_CURRENT_SOURCE_DIR}/net/tcp)

file(GLOB COMM_SRC ${PATH_COMM}/*.cc)
file(GLOB NET_SRC  ${PATH_NET}/*.cc)
file(GLOB TCP_SRC  ${PATH_TCP}/*.cc)

# ============================================================
# 静态库
# ============================================================
add_library(lrpc STATIC ${COMM_SRC} ${NET_SRC} ${TCP_SRC})

# ============================================================
# 头文件路径
# ============================================================
target_include_directories(lrpc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${PATH_COMM}
    # ${PATH_NET}
    # ${PATH_TCP}
    ${CMAKE_SOURCE_DIR}
)

# ============================================================
# 额外依赖
# ============================================================
target_link_libraries(lrpc
    /usr/lib/libprotobuf.a
    /usr/lib/libtinyxml.a
)

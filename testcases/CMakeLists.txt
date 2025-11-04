# testcases/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

set(PATH_TESTCASES ${CMAKE_CURRENT_SOURCE_DIR})

# ============================================================
# 测试程序
# ============================================================
add_executable(test_log ${PATH_TESTCASES}/test_log.cc)
add_executable(test_eventloop ${PATH_TESTCASES}/test_eventloop.cc)
add_executable(test_io ${PATH_TESTCASES}/test_io.cc)
add_executable(test_tcp ${PATH_TESTCASES}/test_tcp.cc)
add_executable(test_connect ${PATH_TESTCASES}/test_connect.cc)

# 自动继承 lrpc 的 include 路径（因为是 PUBLIC）
target_link_libraries(test_log          lrpc pthread dl)
target_link_libraries(test_eventloop    lrpc pthread dl)
target_link_libraries(test_io           lrpc pthread dl)
target_link_libraries(test_tcp           lrpc pthread dl)
target_link_libraries(test_connect          lrpc pthread dl)

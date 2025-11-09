# testcases/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

set(PATH_TESTCASES ${CMAKE_CURRENT_SOURCE_DIR})

# ============================================================
# 测试程序
# ============================================================
add_executable(test_log ${PATH_TESTCASES}/test_log.cc)
add_executable(test_eventloop ${PATH_TESTCASES}/test_eventloop.cc)
add_executable(test_io ${PATH_TESTCASES}/test_io.cc)
add_executable(test_server ${PATH_TESTCASES}/test_server.cc)
add_executable(test_client ${PATH_TESTCASES}/test_client.cc)

# 自动继承 lrpc 的 include 路径（因为是 PUBLIC）
target_link_libraries(test_log          lrpc pthread dl)
target_link_libraries(test_eventloop    lrpc pthread dl)
target_link_libraries(test_io           lrpc pthread dl)
target_link_libraries(test_server           lrpc pthread dl)
target_link_libraries(test_client          lrpc pthread dl)

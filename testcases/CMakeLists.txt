# testcases/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

set(PATH_TESTCASES ${CMAKE_CURRENT_SOURCE_DIR})

set(PROTO_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/order.pb.cc ${CMAKE_CURRENT_SOURCE_DIR}/ping.pb.cc)

# ============================================================
# 测试程序
# ============================================================
add_executable(test_log ${PATH_TESTCASES}/test_log.cc)
add_executable(test_eventloop ${PATH_TESTCASES}/test_eventloop.cc)
add_executable(test_io ${PATH_TESTCASES}/test_io.cc)
add_executable(test_server ${PATH_TESTCASES}/test_server.cc)
add_executable(test_client ${PATH_TESTCASES}/test_client.cc)
add_executable(test_rpc_server ${PATH_TESTCASES}/test_rpc_server.cc ${PROTO_SRCS})
add_executable(test_rpc_client ${PATH_TESTCASES}/test_rpc_client.cc ${PROTO_SRCS})
add_executable(ping_server ${PATH_TESTCASES}/ping_server.cc ${PROTO_SRCS})
add_executable(ping_client ${PATH_TESTCASES}/ping_client.cc ${PROTO_SRCS})

# 自动继承 lrpc 的 include 路径（因为是 PUBLIC）
target_link_libraries(test_log          lrpc pthread dl)
target_link_libraries(test_eventloop    lrpc pthread dl)
target_link_libraries(test_io           lrpc pthread dl)
target_link_libraries(test_server       lrpc pthread dl)
target_link_libraries(test_client       lrpc pthread dl)
target_link_libraries(test_rpc_client   lrpc pthread dl)
target_link_libraries(test_rpc_server   lrpc pthread dl)
target_link_libraries(ping_server   lrpc pthread dl)
target_link_libraries(ping_client   lrpc pthread dl)

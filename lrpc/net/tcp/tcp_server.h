#pragma once

#include "lrpc/net/tcp/tcp_acceptor.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/io_thread_group.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include <unordered_set>

namespace lrpc{

class TcpServer{

public:
    TcpServer(NetAddr::s_ptr local_addr);

    ~TcpServer();

public:
    void start();

private:
    void init();
    
    // 有新客户端连接需要执行
    void onAccept();

private:
    TcpAcceptor::s_ptr acceptor_;
    NetAddr::s_ptr local_addr_;             // 本地监听的地址
    FdEvent *listen_fd_event_{0};
    
    EventLoop* main_event_loop_{NULL};      // main reactor
    
    int io_thread_num_{2};
    IOThreadGroup* io_thread_group_{NULL};  // subReactor 组

    int worker_thread_num_{2};
    
    std::unordered_set<TcpConnection::s_ptr> client_;
    int client_counts_{0};
};

} // namespace lrpc
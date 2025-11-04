#pragma once

#include "lrpc/net/fd_event.h"
#include "lrpc/net/io_thread.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_buffer.h"
#include <cstddef>
#include <memory>

namespace lrpc{

class TcpConnection{
public:
    enum TcpState{
        NotConnected = 1,
        Connected = 2,
        HalfConnected = 3,
        Closed = 4,
    };

public:
    typedef std::shared_ptr<TcpConnection> s_ptr;

    TcpConnection(IOThread* io_thread, int fd, int buffer_size, NetAddr::s_ptr peer_addr_);

    ~TcpConnection();


public:
    void onRead();

    void execute();
    
    void onWrite();

    void setState(const TcpState s) { state_ = s; }

    TcpState getState() { return state_ ; }

    void clear();

    void shutdown();

private:
    // 通信的两个对端
    NetAddr::s_ptr local_addr_;
    NetAddr::s_ptr peer_addr_;

    // 输入输出缓冲区
    TcpBuffer::s_ptr in_buffer;
    TcpBuffer::s_ptr out_buffer;

    // 持有该连接的IO线程IO， 也就是本对象属于哪个IO线程
    IOThread* io_thread_{NULL};
    
    // 一个连接只用一个 FdEvent监听
    FdEvent* fd_event_{NULL};
    int fd_{-1};

    TcpState state_;
};

}   // namespace lrpc
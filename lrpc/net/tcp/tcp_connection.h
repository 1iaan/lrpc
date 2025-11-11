#pragma once

#include "lrpc/net/coder/abs_coder.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/io_thread.h"
#include "lrpc/net/coder/abs_protocol.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_buffer.h"
#include <cstddef>
#include <functional>
#include <memory>
#include <map>

namespace lrpc{

class RpcDispatcher;

class TcpConnection{
public:
    enum TcpState{
        NotConnected = 1,
        Connected = 2,
        HalfConnected = 3,
        Closed = 4,
    };

    enum TcpConnectionType{
        ClientConnectionByServer = 1,   // 服务端使用，代表对端为客户端连接
        ServerConnectionByClient = 2,   // 客户端使用，代表对端为服务端连接
    };

public:
    typedef std::shared_ptr<TcpConnection> s_ptr;

    // TcpConnection(IOThread* io_thread, int fd, int buffer_size, NetAddr::s_ptr peer_addr);
    TcpConnection(EventLoop* event_loop, int fd, int buffer_size, NetAddr::s_ptr local_addr, NetAddr::s_ptr peer_addr, TcpConnectionType type);

    ~TcpConnection();


public:
    void onRead();

    void execute();
    
    void onWrite();

    void setState(const TcpState s) { state_ = s; }

    TcpState getState() { return state_ ; }

    void clear();

    void shutdown();

    void setConnectionType(const TcpConnectionType type) { connection_type_ = type; }

    void listenRead();

    void listenWrite();

    void pushSendMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback);

    void pushReadMessage(std::string msg_id, std::function<void(AbstractProtocol::s_ptr)> callback);

    IPNetAddr::s_ptr getLocalAddr(){ return local_addr_; }
    IPNetAddr::s_ptr getPeerAddr(){ return peer_addr_; }
private:
    // 通信的两个对端
    NetAddr::s_ptr local_addr_;
    NetAddr::s_ptr peer_addr_;

    // 输入输出缓冲区
    TcpBuffer::s_ptr in_buffer;
    TcpBuffer::s_ptr out_buffer;

    // 持有该连接的IO线程IO， 也就是本对象属于哪个IO线程
    // IOThread* io_thread_{NULL};
    EventLoop* event_loop_{NULL};
    
    // 一个连接只用一个 FdEvent监听
    FdEvent* fd_event_{NULL};
    int fd_{-1};

    TcpState state_;

    TcpConnectionType connection_type_{TcpConnectionType::ServerConnectionByClient};

    // key = message->msg_id_ value = callback
    std::vector<
        std::pair<AbstractProtocol::s_ptr, std::function<void(AbstractProtocol::s_ptr)>>
    > write_callbacks_;

    std::map<
        std::string, std::function<void(AbstractProtocol::s_ptr)>
    > read_callbacks_;

    AbstractCoder* coder_{NULL};
};

}   // namespace lrpc
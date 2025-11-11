#pragma once
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/coder/abs_protocol.h"
#include <memory>
namespace lrpc {

class TcpClient {

public: 
    typedef std::shared_ptr<TcpClient> s_ptr;
    TcpClient(NetAddr::s_ptr peer_addr);

    ~TcpClient();

public:
    // 异步connect
    // 如果调用connect成功，callback会执行
    void connect(std::function<void()> callback);

    // 异步发送 message
    // 发送成功会调用callback，callback入参是message对象
    void writeMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback);

    // 异步接收 message
    // 接收成功会调用callback， callback入参是message对象
    void readMessage(const std::string &msg_id, std::function<void(AbstractProtocol::s_ptr)> callback);

    void stop();
private:
    NetAddr::s_ptr peer_addr_;
    EventLoop* event_loop_{NULL};

    int fd_{-1};
    FdEvent* fd_event_{NULL};
    TcpConnection::s_ptr connection_{NULL};
};

} // namespace lrpc
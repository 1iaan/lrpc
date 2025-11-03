#pragma once

#include "lrpc/net/tcp/net_addr.h"
#include <memory>

namespace lrpc{

class TcpAcceptor {

public:
    typedef  std::shared_ptr<TcpAcceptor> s_ptr;

    TcpAcceptor(NetAddr::s_ptr local_addr);

    ~TcpAcceptor();

public:
    int accept();

    int getListenFd() { return listenfd_; }

private:
    NetAddr::s_ptr local_addr_; // 服务端监听的地址 addr -> ip:port
    int family_{-1};            // 地址协议族
    int listenfd_{-1};           // listenfd
    
};

} // namespace lrpc
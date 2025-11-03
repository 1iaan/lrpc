#pragma once

#include "lrpc/net/tcp/net_addr.h"

namespace lrpc{

class TcpAcceptor {

public:
    TcpAcceptor(NetAddr::s_ptr local_addr);

    ~TcpAcceptor();

public:
    int accept();

private:
    NetAddr::s_ptr local_addr_; // 服务端监听的地址 addr -> ip:port
    int family_{-1};            // 地址协议族
    int listenfd_{-1};           // listenfd
    
};

} // namespace lrpc
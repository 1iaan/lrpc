#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_server.h"
#include <memory>
#include <sys/socket.h>

void test_tcpServer(){
    lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    
    lrpc::TcpServer tcp_server(addr);
    tcp_server.start();
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    test_tcpServer();
    return 0;
}
#include "lrpc/net/tcp/tcp_acceptor.h"
#include "lrpc/common/log.h"
#include "lrpc/net/tcp/net_addr.h"
#include <asm-generic/socket.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>

namespace lrpc {

TcpAcceptor::TcpAcceptor(NetAddr::s_ptr local_addr):local_addr_(local_addr){
    if(!local_addr_->checkValid()){
        ERRORLOG("invalid local addr %s", local_addr_->toString().c_str());
        exit(0);
    }
    family_ = local_addr_->getFamily();
    listenfd_ = socket(family_, SOCK_STREAM, 0);
    if(listenfd_ == 0){
        ERRORLOG("invalid listenfd %d", listenfd_);
        exit(0);
    }

    int val = 1;
    // SO_REUSEADDR 监听一个套接字，然后服务器关闭，重启这个服务，如果是同一个端口可能会报bind错误，addr已经被绑定。
    // 因为tcp在主动关闭连接的一方，套接字会变成timewait状态，处于timewait状态会持续占用端口，如果新程序在这个端口启动，bind就会出错。
    // 该选项可以重新绑定这个端口。
    if(setsockopt(listenfd_, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0){
        ERRORLOG("setsockopt REUSEADDR error, errno=%d, error=%s", errno, strerror(errno));
    }

    socklen_t len = local_addr_->getSockLen();
    int rt = bind(listenfd_, local_addr->getSockAddr(), len);
    if(rt != 0){
        ERRORLOG("bind error, errno=%d, error=%s", errno, strerror(errno));
        exit(0);
    }

    rt = listen(listenfd_, 1000);
    if(rt != 0){
        ERRORLOG("listen error, errno=%d, error=%s", errno, strerror(errno));
        exit(0);
    }
}

TcpAcceptor::~TcpAcceptor(){

}

int TcpAcceptor::accept(){
    if(family_ == AF_INET){
        sockaddr_in client_addr;
        bzero(&client_addr,sizeof(client_addr));
        socklen_t client_addr_len = sizeof(client_addr);

        int client_fd = ::accept(listenfd_, reinterpret_cast<sockaddr*>(&client_addr), &client_addr_len);
        if(client_fd < 0){
            ERRORLOG("accept error, errno=%d, error=%s", errno, strerror(errno));
        }

        IPNetAddr addr(client_addr);
        INFOLOG("[TcpServer] A client have accept success, peer addr [%s]", addr.toString().c_str());
        return client_fd;
        
    }
    return -1;
}


}   // namespace lrpc
#include "lrpc/common/log.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/fd_event_group.h"
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

namespace lrpc {

TcpClient::TcpClient(NetAddr::s_ptr peer_addr):peer_addr_(peer_addr){
    event_loop_ = EventLoop::GetCurEventLoop();
    fd_ = socket(peer_addr->getFamily(), SOCK_STREAM, 0);

    if(fd_ < 0){
        ERRORLOG("create socket error");
        return ;
    }

    fd_event_ = FdEventGroup::GetFdEventGroup()->getFdEvent(fd_);
    fd_event_->setNonBlock();

    connection_ = std::make_shared<TcpConnection>(event_loop_, fd_, 128, nullptr, peer_addr_, TcpConnection::ServerConnectionByClient);

}

TcpClient::~TcpClient(){
    if(fd_ > 0){
        close(fd_);
    }
}

void TcpClient::connect(std::function<void()> callback){
    int rt = ::connect(fd_, peer_addr_->getSockAddr(), peer_addr_->getSockLen());
    if(rt == 0){
        DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
    }else if(rt == -1){
        if(errno == EINPROGRESS){
            // epoll 监听可写
            DEBUGLOG("connect in progress");

            fd_event_->listen(FdEvent::OUT_EVENT, [this, callback]()->void{
                {}
                int err = 0;
                socklen_t len = sizeof(err);
                getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len);
                bool is_connected = false;
                if(err == 0){
                    DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
                    connection_->setState(TcpConnection::Connected);
                    is_connected = true; 
                } else {
                    ERRORLOG("connect error, errno=%d, error=%s", err, strerror(errno));
                }
                // 连接完成去掉可写事件监听，否则会一直除法
                DEBUGLOG("去掉可写事件监听");
                fd_event_->cancel(FdEvent::OUT_EVENT);
                event_loop_->addEpollEvent(fd_event_);

                // 连接成功才执行回调
                if(is_connected && callback) callback();
            });
            
            event_loop_->addEpollEvent(fd_event_);
            if(!event_loop_->isLooping()) {
                event_loop_->loop();
            }

        } else {
            ERRORLOG("connect error, errno=%d, error=%s", errno, strerror(errno));
            return ;
        }
    }
}

void TcpClient::writeMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback){
    //1. 把message编码写入到connection的buffer中, done 也要写入。
    //2. 启动connection可写事件。
    connection_->pushSendMessage(message, callback);
    connection_->listenWrite();
}

void TcpClient::readMessage(const std::string &req_id, std::function<void(AbstractProtocol::s_ptr)> callback){
    //1. 启动connection可读事件
    //2. 从buffer中解码读出message, 判断 req_id 是否相等，相等则读成功，执行其回调。
    connection_->pushReadMessage(req_id, callback);
    connection_->listenRead();
}

} // namespace lrpc
#include "lrpc/common/error_code.h"
#include "lrpc/common/log.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/fd_event_group.h"
#include "lrpc/net/io_thread.h"
#include "lrpc/net/tcp/net_addr.h"
#include <cerrno>
#include <memory>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

namespace lrpc {

TcpClient::TcpClient(NetAddr::s_ptr peer_addr):peer_addr_(peer_addr){
    // event_loop_ =
    io_thread_ = new IOThread(); 
    event_loop_ = io_thread_->getEventloop();
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
    DEBUGLOG("~TcpClient");
    if(fd_ > 0){
        close(fd_);
    }
}

void TcpClient::connect(std::function<void()> callback){
    int rt = ::connect(fd_, peer_addr_->getSockAddr(), peer_addr_->getSockLen());
    if(rt == 0){
        initLocalAddr();
        connection_->setState(TcpConnection::Connected);
        DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
        if(callback){
            callback();
        }
    }else if(rt == -1){
        if(errno == EINPROGRESS){
            DEBUGLOG("connect in progress");

            // epoll 监听可写，判断错误码
            fd_event_->listen(FdEvent::OUT_EVENT, 
                [this, callback]()->void{
                    int rt = ::connect(fd_, peer_addr_->getSockAddr(), peer_addr_->getSockLen());
                    if((rt < 0 && errno == EISCONN) || rt == 0){
                        initLocalAddr();
                        connection_->setState(TcpConnection::Connected);
                        DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
                    }else{
                        if(errno == ECONNREFUSED){
                            connect_error_code_ = ERROR_PEER_CLOSED;
                            connect_error_info_ = "connect refused, sys error = " + std::string(strerror(errno));
                        }else{
                            connect_error_code_ = ERROR_FAILED_CONNECT;
                            connect_error_info_ = "connect unknown error, sys error = " + std::string(strerror(errno));
                        }
                        ERRORLOG("connect error, errno=%d, error=%s", errno, strerror(errno));

                        if(callback){
                            callback();
                        }
                        
                        close(fd_);
                        fd_ = socket(peer_addr_->getFamily(), SOCK_STREAM, 0);
                    }

                    event_loop_->delEpollEvent(fd_event_);
                    DEBUGLOG("now begin to done");
                    if(callback){
                        callback();
                    }
                }
            );
            
            event_loop_->addEpollEvent(fd_event_);
            if(!event_loop_->isLooping()) {
                io_thread_->start();
                // event_loop_->loop();
            }

        } else {
            connect_error_code_ = ERROR_FAILED_CONNECT;
            connect_error_info_ = "connect error, sys error = " + std::string(strerror(errno));
            ERRORLOG("connect error, errno=%d, error=%s", errno, strerror(errno));
            if(callback){
                callback();
            }
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

void TcpClient::readMessage(const std::string &msg_id, std::function<void(AbstractProtocol::s_ptr)> callback){
    //1. 启动connection可读事件
    //2. 从buffer中解码读出message, 判断 msg_id 是否相等，相等则读成功，执行其回调。
    connection_->pushReadMessage(msg_id, callback);
    connection_->listenRead();
}

void TcpClient::stop(){
    if(event_loop_->isLooping()){
        event_loop_->stop();
    }
}

int TcpClient::getConnectErrorCode(){
    return connect_error_code_;
}

std::string TcpClient::getConnectErrorInfo(){
    return connect_error_info_;
}

NetAddr::s_ptr TcpClient::getPeerAddr(){
    return peer_addr_;
}

NetAddr::s_ptr TcpClient::getLocalAddr(){
    return local_addr_;
}

void TcpClient::initLocalAddr(){
    sockaddr_in local_addr;
    bzero(&local_addr, sizeof(local_addr));
    socklen_t len = sizeof(local_addr);
    int rt = getsockname(fd_, reinterpret_cast<sockaddr*>(&local_addr), &len);
    if(rt != 0){
        ERRORLOG("initLocalAddr error, getsockname errror, errno=%d, error=%s", errno, strerror(errno));
        return;
    }
   
    local_addr_ = std::make_shared<IPNetAddr>(local_addr);
    // DEBUGLOG("------local addr [%s]", local_addr_->toString().c_str());
}

void TcpClient::addTimerEvent(TimerEvent::s_ptr timer_event){
    event_loop_->addTimerEvent(timer_event);
}

} // namespace lrpc
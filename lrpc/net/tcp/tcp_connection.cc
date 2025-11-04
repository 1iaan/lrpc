#include "lrpc/common/log.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/fd_event_group.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/tcp/tcp_buffer.h"
#include <cerrno>
#include <memory>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace lrpc{

TcpConnection::TcpConnection(IOThread* io_thread, int fd, int buffer_size, NetAddr::s_ptr peer_addr)
    : peer_addr_(peer_addr), io_thread_(io_thread), fd_(fd), state_(NotConnected){
    in_buffer = std::make_shared<TcpBuffer>(buffer_size);
    out_buffer = std::make_shared<TcpBuffer>(buffer_size);

    fd_event_ = FdEventGroup::GetFdEventGroup()->getFdEvent(fd);
    fd_event_->setNonBlock();
    fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpConnection::onRead, this));

    io_thread->getEventloop()->addEpollEvent(fd_event_);
}

TcpConnection::~TcpConnection(){

}


void TcpConnection::onRead(){
    // 1. 从 socket 缓冲区调用 read 读取字节流到 inbuffer 里

    if(state_ != Connected){
        ERRORLOG("onRead error client has already disconnected, addr[%s], clientfd[%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    bool is_read_all = false;
    bool is_closed = false;
    while(!is_read_all){
        
        if(in_buffer->writeable() == 0){
            in_buffer->resizeBuffer(2 * in_buffer->buffer_.size());
        }

        int read_size = in_buffer->writeable();
        int write_index = in_buffer->getWriteIdx();

        int rt = read(fd_, &in_buffer->buffer_[write_index], read_size);
        DEBUGLOG("success read %d byte from addr[%s], clientfd[%d]", rt, peer_addr_->toString().c_str(), fd_);
        if(rt > 0){
            in_buffer->moveWriteIndex(rt);
            // 如果本次读取数据和writeable相同， 可能还有没读的， 继续读
            if(rt == read_size){
                continue;
            } else {
                is_read_all = true;
                break;
            }
        }else if(rt == 0){
            is_closed = true; 
            break;
        // 非阻塞读到没有可读会报错 EAGAIN
        }else if(rt == -1 && errno == EAGAIN){
            is_read_all = true;
            break;
        }
    }

    if(is_closed){
        // TODO: 处理关闭连接
        clear();
        DEBUGLOG("peer closed, peer addr [%d], clientfd [%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    if(!is_read_all){
        ERRORLOG("not read all data");
    }

    // TODO: 解析协议
    execute();
}

void TcpConnection::execute(){
    // 将RPC请求执行业务逻辑，获取RPC响应，发送回去
    std::vector<char> tmp;
    int size = in_buffer->readable();
    tmp.resize(size);

    in_buffer->readFromBuf(tmp, size);

    std::string msg;
    for(int i = 0;i < tmp.size(); ++ i){
        msg += tmp[i];
    }
    INFOLOG("success get request[%s] fomr client[%s]", tmp.data(), peer_addr_->toString().c_str());

    out_buffer->writeToBuf(tmp.data(), tmp.size());

    fd_event_->listen(FdEvent::OUT_EVENT, std::bind(&TcpConnection::onWrite, this));
    
    io_thread_->getEventloop()->addEpollEvent(fd_event_);
}

void TcpConnection::onWrite(){
    if(state_ != Connected){
        ERRORLOG("onWrite error client has already disconnected, addr[%s], clientfd[%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    bool is_write_all = false;
    while(true){
        if(out_buffer->readable() == 0){
            DEBUGLOG("no data need to send to client [%s]", peer_addr_->toString().c_str());
            is_write_all = true;
            break;
        }
        int write_size = out_buffer->readable();
        int read_index = out_buffer->getReadIdx();
        int rt = write(fd_, &out_buffer->buffer_[read_index], write_size);

        if(rt >= write_size){
            DEBUGLOG("send all data to client [%s]", peer_addr_->toString().c_str());
            is_write_all = true;
            break;
        }
        if(rt == -1 && errno == EAGAIN){
            // 缓冲区满了不能发送
            // 等待下次 fd 可写在发送
            DEBUGLOG("write data error, errno=EAGAIN, rt = -1");
            break;
        }
    }
    if(is_write_all){
        fd_event_->cancel(FdEvent::OUT_EVENT);
        io_thread_->getEventloop()->addEpollEvent(fd_event_);
    }
}

void TcpConnection::clear(){
    // 处理关闭连接后的清理动作
    if(state_ == Closed){
        return;
    }

    io_thread_->getEventloop()->delEpollEvent(fd_event_);

    state_ = Closed;

}

void TcpConnection::shutdown(){
    if(state_ == Closed || state_ == NotConnected){
        return;
    }

    // 处于半关闭
    state_ = HalfConnected;

    // 关闭读写， 服务器不会再对这个fd进行读写操作了
    // 发送FIN报文，触发四次挥手第一阶段
    // 当FD发生可读事件但是可读数据为0，即对端也发送了FIN报文
    // 服务器就会进入timewait
    ::shutdown(fd_, SHUT_RDWR);
}

}   // namespace lrpc
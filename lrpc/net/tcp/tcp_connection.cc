#include "lrpc/common/log.h"
#include "lrpc/common/mutex.h"
#include "lrpc/net/coder/tinypb_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/fd_event_group.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/coder/string_coder.h"
#include "lrpc/net/coder/abs_protocol.h"
#include "lrpc/net/rpc/rpc_dispatcher.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_buffer.h"
#include <cerrno>
#include <cstddef>
#include <functional>
#include <memory>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lrpc{

TcpConnection::TcpConnection(EventLoop* event_loop, int fd, int buffer_size,NetAddr::s_ptr local_addr, NetAddr::s_ptr peer_addr, TcpConnectionType type)
    :local_addr_(local_addr), peer_addr_(peer_addr), event_loop_(event_loop), fd_(fd), state_(NotConnected), connection_type_(type) {
    in_buffer = std::make_shared<TcpBuffer>(buffer_size);
    out_buffer = std::make_shared<TcpBuffer>(buffer_size);

    fd_event_ = FdEventGroup::GetFdEventGroup()->getFdEvent(fd);
    fd_event_->setNonBlock();

    // 服务端，对端为客户端，需要在初始化监听可读事件
    // server中onAccept会创建Connection，连接完成
    if(connection_type_ == ClientConnectionByServer){
        // listenRead();
    }
    // 客户端，对端为服务端，不需要一直监听可读事件
    // 他发送connect后必须等待回包才说明连接成功
    // 可读的OnRead可以执行的前提是连接已经建立

    // coder_ = new StringCoder();
    coder_ = new TinyPBCoder();
}

TcpConnection::~TcpConnection(){
    ERRORLOG("~TcpConnection");
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
        DEBUGLOG("read rt = %d", rt);
        if(rt > 0){
            in_buffer->moveWriteIndex(rt);
            // 如果本次读取数据和writeable相同， 可能还有没读的， 继续读
            if(rt == read_size){
                continue;
            } else {
                is_read_all = true;
                break;
            }
        // rt == 0 说明读到 EOF 对端关闭连接（EOF）
        }else if(rt == 0){
            is_closed = true; 
            break;
        // 非阻塞读到没有可读会报错 EAGAIN
        }else if(rt == -1 && errno == EAGAIN){
            is_read_all = true;
            break;
        }else if(rt < 0){
            is_closed = true; 
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

    // TODO: RPC解析协议
    
    // 2. 协议解析
    execute();
}

void TcpConnection::execute(){
    // 服务端逻辑，对端是客户端
    if(connection_type_ == ClientConnectionByServer){
        DEBUGLOG("server execute");
        // 将RPC请求执行业务逻辑，获取RPC响应，发送回去
        // std::vector<char> tmp;
        // int size = in_buffer->readable();
        // tmp.resize(size);
        // in_buffer->readFromBuf(tmp, size);
        std::vector<AbstractProtocol::s_ptr> result;
        std::vector<AbstractProtocol::s_ptr> response_messages;
        DEBUGLOG("decode buffer to protocol");
        coder_->decode(result, in_buffer);
        DEBUGLOG("decode complete");
        for(size_t i = 0;i < result.size(); ++ i){
            // 1. 针对每个请求调用rpc方法响应message
            // 2. 将message放入发送缓冲区监听可写事件
            INFOLOG("[TcpConnection] success get request[%s] fomr client[%s]", result[i]->msg_id_.c_str(), peer_addr_->toString().c_str());
            TinyPBProtocol::s_ptr message = std::make_shared<TinyPBProtocol>();
            // message->pb_data_ = (std::dynamic_pointer_cast<TinyPBProtocol>(result[i]))->pb_data_;
            // message->method_name_ = (std::dynamic_pointer_cast<TinyPBProtocol>(result[i]))->method_name_;
            // message->msg_id_ = (std::dynamic_pointer_cast<TinyPBProtocol>(result[i]))->msg_id_;
            
            RpcDispatcher::GetRpcDispatcher()->dispatch(result[i], message, this);
            response_messages.push_back(message);
        }

        DEBUGLOG("encode protocol to buffer");
        coder_->encode(response_messages, out_buffer);
        DEBUGLOG("encode complete");

        // INFOLOG("listen write %d", response_messages.size());
        listenWrite();
    }
    // 客户端逻辑，对端是服务端
    else {
        DEBUGLOG("client execute");
        // 从 buffer 里decode 得到 message 对象，执行其回调
        std::vector<AbstractProtocol::s_ptr> out_messages;
        coder_->decode(out_messages, in_buffer);

        for(size_t i = 0; i < out_messages.size(); ++ i){
            std::string msg_id = out_messages[i]->msg_id_;
            auto it = read_callbacks_.find(msg_id);
            if( it != read_callbacks_.end()){
                it->second(out_messages[i]->shared_from_this());
            }
        }
    }
}

void TcpConnection::onWrite(){
    if(state_ != Connected){
        ERRORLOG("onWrite error client has already disconnected, addr[%s], clientfd[%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    // ScopeMutex<Mutex> lock(mutex_);
    std::vector<
        std::pair<AbstractProtocol::s_ptr, std::function<void(AbstractProtocol::s_ptr)>>
    > tmp = write_callbacks_;
    write_callbacks_.clear();
    lock.unlock();

    // 客户端，对端是服务端
    // 1. 编码message到outbuffer
    if (connection_type_ == ServerConnectionByClient){
        // 将数据写入到 buffer 
        // 1. 将 message encoder
        // 2. 将 字节流写入到 buffer，全部发送

        std::vector<AbstractProtocol::s_ptr> messages;
        for(size_t i = 0;i < tmp.size(); ++ i){
            messages.push_back(tmp[i].first);
        }
        coder_->encode(messages, out_buffer);
    }

    // 2. 从 outbuffer 调用 write 写字节流到 socket 缓冲区
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
        if(rt > 0) out_buffer->moveReadIndex(write_size);
        if(rt >= write_size){
            INFOLOG("send all data to client [%s]", peer_addr_->toString().c_str());
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
    // 3. 如果全部发送完成，取消可写事件监听
    if(is_write_all && write_callbacks_.empty()){
        fd_event_->cancel(FdEvent::OUT_EVENT);
        event_loop_->addEpollEvent(fd_event_);
    }

    // 4. 为客户端，对端是服务端，执行回调函数
    if(connection_type_ == ServerConnectionByClient){
        for(size_t i = 0;i < tmp.size(); ++ i){
            tmp[i].second(tmp[i].first);
        }
    }
    tmp.clear(); 
}

void TcpConnection::clear(){
    // 处理关闭连接后的清理动作
    if(state_ == Closed){
        return;
    }


    event_loop_->delEpollEvent(fd_event_);
    fd_event_->cancel(FdEvent::IN_EVENT);
    fd_event_->cancel(FdEvent::OUT_EVENT);

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

void TcpConnection::listenRead(){
    // std::shared_ptr<TcpConnection> conn = shared_from_this();
    // fd_event_->listen(FdEvent::IN_EVENT, [conn]()->void{
    //     conn->onRead();
    // });
    // fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpConnection::onRead, this));
    fd_event_->listen(FdEvent::IN_EVENT, [this]()->void{
        onRead();
    });
    event_loop_->addEpollEvent(fd_event_);
}

void TcpConnection::listenWrite(){
    // std::shared_ptr<TcpConnection> conn = shared_from_this();
    // fd_event_->listen(FdEvent::OUT_EVENT, [conn]()->void{
    //     conn->onWrite();
    // });
    // fd_event_->listen(FdEvent::OUT_EVENT, std::bind(&TcpConnection::onWrite, this));
    fd_event_->listen(FdEvent::OUT_EVENT, [this]()->void{
        onWrite();
    });
    event_loop_->addEpollEvent(fd_event_);
}

void TcpConnection::pushSendMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback){
    std::unique_lock<std::mutex> lock(mutex_);
    // ScopeMutex<Mutex> lock(mutex_);
    write_callbacks_.push_back(std::make_pair(message, callback));
}

void TcpConnection::pushReadMessage(std::string msg_id, std::function<void(AbstractProtocol::s_ptr)> callback){
    std::unique_lock<std::mutex> lock(mutex_);
    // ScopeMutex<Mutex> lock(mutex_);
    read_callbacks_.insert(std::make_pair(msg_id, callback));
}

}   // namespace lrpc
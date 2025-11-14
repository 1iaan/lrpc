#include "lrpc/net/tcp/tcp_server.h"
#include "lrpc/common/config.h"
#include "lrpc/common/log.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/io_thread_group.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_acceptor.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include <memory>

namespace lrpc{

TcpServer::TcpServer(NetAddr::s_ptr local_addr):local_addr_(local_addr){
    if(Config::GetGlobalConfig()){
        io_thread_num_ = Config::GetGlobalConfig()->io_thread_num_;
        worker_thread_num_ = Config::GetGlobalConfig()->worker_thread_num_;
    }
    INFOLOG("[TcpServer] Start create");
    init();
    INFOLOG("[TcpServer] Success create on [%s]", local_addr->toString().c_str());
}

TcpServer::~TcpServer(){
    if(main_event_loop_){
        delete main_event_loop_;
        main_event_loop_ = NULL;
    }
    if(io_thread_group_){
        delete io_thread_group_;
        io_thread_group_ = NULL;
    }
}

void TcpServer::start(){
    io_thread_group_->start();
    main_event_loop_->loop();
}

void TcpServer::init(){
    acceptor_ = std::make_shared<TcpAcceptor>(local_addr_);

    DEBUGLOG("---------- tcpServer mian_event_loop create ----------");
    main_event_loop_ = EventLoop::GetCurEventLoop();

    DEBUGLOG("---------- tcpServer io_thread_group create ----------");
    io_thread_group_ = new IOThreadGroup(io_thread_num_);

    listen_fd_event_ = new FdEvent(acceptor_->getListenFd(), "Acceptor");
    listen_fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpServer::onAccept, this));

    main_event_loop_->addEpollEvent(listen_fd_event_);
    DEBUGLOG("----------       tcpServer init over        ----------");
}

// 有一个新连接来了， maineventloop处理，添加到 iothreadgroup中
void TcpServer::onAccept(){
    auto acc = acceptor_->accept();
    int client_fd = acc.first;
    NetAddr::s_ptr peer_addr = acc.second;

    ++client_counts_;
    
    // TODO 把 client_fd 添加到 io线程中
    // FdEvent *client_fd_event = new FdEvent(client_fd);
    // auto eventloop = io_thread_group_->getIOThread();
    // eventloop->getEventloop()->addEpollEvent(client_fd_event);
    IOThread* io_thread = io_thread_group_->getIOThread();
    
    TcpConnection::s_ptr connection = std::make_shared<TcpConnection>(io_thread->getEventloop(), client_fd, 128, local_addr_, peer_addr, TcpConnection::ClientConnectionByServer);
    connection->setState(TcpConnection::Connected);
    client_.insert(connection);
    

    INFOLOG("[TcpServer] Success get client fd=%d", client_fd);
}

} // namespace lrpc
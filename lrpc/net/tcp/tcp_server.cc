#include "lrpc/net/tcp/tcp_server.h"
#include "lrpc/common/log.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/io_thread_group.h"
#include "lrpc/net/tcp/tcp_acceptor.h"
#include <memory>

namespace lrpc{

TcpServer::TcpServer(NetAddr::s_ptr local_addr):local_addr_(local_addr){
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
    DEBUGLOG("tcpServer start - io_thread_group");
    io_thread_group_->start();
    main_event_loop_->loop();
}

void TcpServer::init(){
    acceptor_ = std::make_shared<TcpAcceptor>(local_addr_);

    DEBUGLOG("---------- tcpServer mian_event_loop create ----------");
    main_event_loop_ = EventLoop::GetCurEventLoop();

    DEBUGLOG("---------- tcpServer io_thread_group create ----------");
    io_thread_group_ = new IOThreadGroup(2);

    listen_fd_event_ = new FdEvent(acceptor_->getListenFd(), "监听主循环事件");
    listen_fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpServer::onAccept, this));

    main_event_loop_->addEpollEvent(listen_fd_event_);
    DEBUGLOG("----------        tcpServer init over        ----------");
}

// 有一个新连接来了， maineventloop处理，添加到 iothreadgroup中
void TcpServer::onAccept(){
    int client_fd = acceptor_->accept();
    ++client_counts_;
    
    // TODO 把 client_fd 添加到 io线程中
    // FdEvent *client_fd_event = new FdEvent(client_fd);

    // auto eventloop = io_thread_group_->getIOThread();
    // eventloop->getEventloop()->addEpollEvent(client_fd_event);

    INFOLOG("[TcpServer] Success get client fd=%d", client_fd);
}

} // namespace lrpc
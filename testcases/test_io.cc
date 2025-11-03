#include "lrpc/common/config.h"
#include "lrpc/common/util.h"
#include "lrpc/common/log.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/io_thread.h"
#include "lrpc/net/io_thread_group.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/net/timer_event.h"
#include <memory>
#include <strings.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


void test_io(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if(listenfd == -1){
        ERRORLOG("listenfd = -1");
        exit(0);
    }

    sockaddr_in addr;
    bzero(&addr, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = INADDR_ANY;

    int rt = bind(listenfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if(rt == -1){
        ERRORLOG("bind error");
        exit(1);
    }
    
    rt = listen(listenfd, 100);
    if(rt == -1){
        ERRORLOG("listen error");
        exit(1);
    }

    // 连接接收任务
    lrpc::FdEvent event(listenfd, "(port:12345)");
    event.listen(lrpc::FdEvent::IN_EVENT, [listenfd]()->void{
        sockaddr_in peer_addr;
        socklen_t addr_len = sizeof(peer_addr);

        bzero(&peer_addr, sizeof(peer_addr));
        int clientfd = accept(listenfd, reinterpret_cast<sockaddr*>(&peer_addr), &addr_len);
        clientfd += 0;

        DEBUGLOG("success get client [%s:%d]", inet_ntoa(peer_addr.sin_addr), ntohs(peer_addr.sin_port));

    });



    // 定时器任务
    int i = 0;
    lrpc::TimerEvent::s_ptr timer_event = std::make_shared<lrpc::TimerEvent>(
        1000, true , [&i]()->void{
            static int64_t last = lrpc::get_now_ms();
            int64_t now = lrpc::get_now_ms();
            printf("thread[%d] timer %d interval = %ld ms\n",lrpc::get_thread_id(),  i++, now - last);
            last = now;
        }
    );

    // loop 是本线程执行的，必须先添加event再loop, 其他线程可以等loop跑起来再添加event
    // lrpc::IOThread io_thread;
    // io_thread.getEventloop()->addEpollEvent(&event);
    // io_thread.getEventloop()->addTimerEvent(timer_event);
    // io_thread.start();
    // DEBUGLOG("start io eventloop");
    // io_thread.join();
    // DEBUGLOG("jion io eventloop");

    lrpc::IOThreadGroup io_thread_group(2);
    lrpc::IOThread *io_thread = io_thread_group.getIOThread();
    io_thread->getEventloop()->addEpollEvent(&event);
    io_thread->getEventloop()->addTimerEvent(timer_event);

    lrpc::IOThread *io_thread2 = io_thread_group.getIOThread();
    io_thread2->getEventloop()->addTimerEvent(timer_event);

    io_thread_group.start();
    io_thread_group.join();

}

int main(){
    test_io();
    return 0;
}
#include "lrpc/common/config.h"
#include "lrpc/net/fd_event.h"
#include "lrpc/common/util.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/common/log.h"
#include "timer_event.h"
#include <memory>
#include <strings.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    lrpc::EventLoop* eventloop = new lrpc::EventLoop();

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

    eventloop->addEpollEvent(&event);

    // 定时器任务
    int i = 0;
    lrpc::TimerEvent::s_ptr timer_event = std::make_shared<lrpc::TimerEvent>(
        1000, true , [&i]()->void{
            static int64_t last = lrpc::get_now_ms();
            int64_t now = lrpc::get_now_ms();
            printf("timer %d interval = %lld ms\n", i++, now - last);
            last = now;
        }
    );

    eventloop->addTimerEvent(timer_event);


    eventloop->loop();

    return 0;
}
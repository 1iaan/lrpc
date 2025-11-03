#include "lrpc/net/fd_event.h"
#include "lrpc/common/log.h"
#include <cstring>
#include <string>
#include <sys/epoll.h>
#include <unistd.h>

namespace lrpc{

FdEvent::FdEvent(){
    
}

FdEvent::FdEvent(int fd): fd_(fd){
    fd_name_ = "fd_" + std::to_string(fd);
    memset(&listen_events_, 0, sizeof(listen_events_));
    listen_events_.data.ptr = this;
}


FdEvent::FdEvent(int fd, std::string fd_name): fd_(fd), fd_name_(fd_name){
    memset(&listen_events_, 0, sizeof(listen_events_));
    listen_events_.data.ptr = this;
}

FdEvent::~FdEvent(){

}

std::function<void()> FdEvent::handler(TriggerEvent ev_t){
    switch (ev_t) {
        case FdEvent::IN_EVENT:{
            return read_callback_;
        }
        case FdEvent::OUT_EVENT:{
            return write_callback_;            
        }
        default:
        break;
    }
}

void FdEvent::listen(TriggerEvent ev_t, std::function<void()> callback){
    switch (ev_t) {
        case FdEvent::IN_EVENT:{
            listen_events_.events |= EPOLLIN;
            read_callback_ = callback;
            break;
        }
        case FdEvent::OUT_EVENT:{
            listen_events_.events |= EPOLLOUT;
            write_callback_ = callback;       
            break; 
        }
        default:
        break;
    }
}


WakeUpFdEvent::WakeUpFdEvent(int fd) : FdEvent(fd, "WAKEUP"){
    INFOLOG("[FdEvent] wakeup init,\t fd=%d", getFd());
}

WakeUpFdEvent::~WakeUpFdEvent(){

}

void WakeUpFdEvent::wakeup(){
    uint64_t one = 1;
    int rt = write(getFd(), &one, sizeof(one));
    if(rt != 8){
        ERRORLOG("write to wakeup fd less than 8 bytes, fd[%d]", getFd());
    }
}

} // namespace lrpc
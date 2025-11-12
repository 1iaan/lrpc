#include "lrpc/net/wakeup_fd_event.h"
#include "lrpc/common/log.h"
#include <unistd.h>

namespace lrpc{
    
WakeUpFdEvent::WakeUpFdEvent(int fd) : FdEvent(fd, "WAKEUP"){
    // DEBUGLOG("[FdEvent] wakeup init,\t fd=%d", getFd());
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

}   // namespace lrpc
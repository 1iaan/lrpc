#include "lrpc/net/fd_event_group.h"
#include "lrpc/common/log.h"
#include "lrpc/common/mutex.h"
#include "lrpc/net/fd_event.h"
#include <cstddef>
#include <mutex>

namespace lrpc{

static FdEventGroup* g_fd_event_group_ = NULL;

FdEventGroup::FdEventGroup(size_t size):size_(size){
    for(int i = 0;i < size_ ; ++ i){
        fd_group_.emplace_back(new FdEvent(i));
    }
}
    
FdEventGroup::~FdEventGroup(){
    DEBUGLOG("~FdEventGroup");
    for(int i = 0;i < size_ ; ++ i){
        if(fd_group_[i]){
            delete fd_group_[i];
            fd_group_[i] = NULL;
        }
    }
}

FdEvent* FdEventGroup::getFdEvent(int fd){
    // ScopeMutex<Mutex> lock(mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    if(fd < size_){
        return fd_group_[fd];
    }
    size_ = fd * 1.5;
    for(int i = fd_group_.size();i < size_; ++ i){
        fd_group_.emplace_back(new FdEvent(i));
    }
    return fd_group_[fd];
}

FdEventGroup* FdEventGroup::GetFdEventGroup(){
    if(!g_fd_event_group_){
        g_fd_event_group_ = new FdEventGroup(128);
    }
    return g_fd_event_group_;
}

} // namespace lrpc
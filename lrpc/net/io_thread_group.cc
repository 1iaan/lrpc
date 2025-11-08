#include "io_thread_group.h"
#include "lrpc/common/log.h"


namespace lrpc{

IOThreadGroup::IOThreadGroup(int size): size_(size){
    INFOLOG("[IOThreadGroup] Start create");
    io_thread_groups_.resize(size);
    for(int i =0;i < size; ++i){
        io_thread_groups_[i] = new IOThread();
    }
    INFOLOG("[IOThreadGroup] Success create");
}
    
IOThreadGroup::~IOThreadGroup(){

}

void IOThreadGroup::start(){
    for(int i = 0;i < size_; ++ i){
        io_thread_groups_[i]->start();
    }
}

void IOThreadGroup::join(){
    for(int i = 0;i < size_; ++ i){
        io_thread_groups_[i]->join();
    }
}

IOThread* IOThreadGroup::getIOThread(){
    if(index_ >= (int)io_thread_groups_.size() || index_ == -1){
        index_ = 0;
    }
    return io_thread_groups_[index_++];
}

} // namespace lrpc
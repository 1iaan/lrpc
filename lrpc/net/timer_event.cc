#include "lrpc/common/log.h"
#include "lrpc/common/util.h"
#include "lrpc/net/timer_event.h"

namespace lrpc{

TimerEvent::TimerEvent(int internal, bool is_repeated, std::function<void()> callback)
        :internal_(internal), is_repeated_(is_repeated), task_(callback){
    // 
    arrive_time_ = get_now_ms() + internal_;
    DEBUGLOG("success create timer event, will execute at [%lld]", arrive_time_);
}

void TimerEvent::setArriveTime(){
    arrive_time_ = arrive_time_ + internal_;
    DEBUGLOG("reset timer event, will execute at [%lld]", arrive_time_);
}
 
    
} // namespace lrpc
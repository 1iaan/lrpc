#include "lrpc/net/timer_event.h"
#include "log.h"
#include "util.h"

namespace lrpc{

TimerEvent::TimerEvent(int internal, bool is_repeated, std::function<void()> callback)
        :internal_(internal), is_repeated_(is_repeated), task_(callback){
    // 
    arrive_time_ = get_now_ms() + internal_;
    DEBUGLOG("success create timer event, will execute at [%lld]", arrive_time_);
}

void TimerEvent::setArriveTime(){
    arrive_time_ = arrive_time_ + internal_;
    // int64_t now = get_now_ms();
    // if (arrive_time_ < now) {
    //     arrive_time_ = now + (internal_ - (now - arrive_time_) % internal_);
    // }
    DEBUGLOG("reset timer event, will execute at [%lld]", arrive_time_);
}
 
    
} // namespace lrpc
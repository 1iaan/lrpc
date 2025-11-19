#pragma once
#include "lrpc/net/fd_event.h"
#include "lrpc/net/timer_event.h"
#include "lrpc/common/mutex.h"
#include <cstdint>
#include <map>
#include <mutex>


namespace lrpc{

class Timer:public FdEvent{
public:
    Timer();
    
    ~Timer();

public:
    void addTimerEvent(TimerEvent::s_ptr event);

    void deleteTimerEvent(TimerEvent::s_ptr event);

    void onTimer(); // 发生IO事件后, eventloop会执行这个函数

private:
    std::multimap<int64_t, TimerEvent::s_ptr> events_;
    // Mutex mutex_;
    std::mutex mutex_;

private:
    void resetArriveTime();
};

} // namespace lrpc
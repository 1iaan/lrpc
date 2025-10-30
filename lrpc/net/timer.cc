#include "lrpc/net/timer.h"
#include "fd_event.h"
#include "log.h"
#include "mutex.h"
#include "timer_event.h"
#include "util.h"
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <functional>
#include <strings.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lrpc{
/**
 * @brief Construct a new Timer:: Timer object
 * CLOCK_MONOTONIC 单调递增时钟（开机后一直增加）
 *
 * TFD_NONBLOCK 非阻塞模式，read() 在没有数据时立即返回 EAGAIN
 * TFD_CLOEXEC 在执行 execve() 时自动关闭 fd
 *
 * 会往fd中写入这是第几次通知，如果一直没有人读就会一直增长计数
 */
Timer::Timer(): FdEvent(timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC), "TIMER"){
    
    INFOLOG("timer init,\t fd=%d", getFd());

    // 把fd的可读事件放到event上监听
    listen(FdEvent::IN_EVENT, std::bind(&Timer::onTimer, this));
}

Timer::~Timer(){

}

void Timer::addTimerEvent(TimerEvent::s_ptr event){
    bool is_reset_timerfd = false;

    ScopeMutex<Mutex> lock(mutex_);
    if(events_.empty()){
        is_reset_timerfd = true;
    }else{
        auto it = events_.begin();
        if((*it).second->getArriveTime() > event->getArriveTime()){
            is_reset_timerfd = true;
        }
    }
    events_.emplace(event->getArriveTime(), event);
    lock.unlock();

    if(is_reset_timerfd){
        resetArriveTime();
    }
    DEBUGLOG("success add TimerEvent at arrivetime %lld", event->getArriveTime());
}

void Timer::deleteTimerEvent(TimerEvent::s_ptr event){
    event->setCanceled(true);

    ScopeMutex<Mutex> lock(mutex_);

    auto begin = events_.lower_bound(event->getArriveTime());
    auto end = events_.upper_bound(event->getArriveTime());

    auto it = begin;
    for(it = begin; it != end ; ++ it){
        if(it->second == event){
            break;
        }
    }
    if(it != end) { 
        events_.erase(it);
    }

    lock.unlock();
    DEBUGLOG("success delete TimerEvent at arrivetime %lld", event->getArriveTime());
}

void Timer::onTimer(){
    // 处理缓冲区
    char buf[8];
    while(1){
        if((read(getFd(), buf, 8) == -1) && errno == EAGAIN){
            break;
        }
    }

    // 执行定时任务
    int64_t now = get_now_ms();
    std::vector<TimerEvent::s_ptr> tmps;
    std::vector<std::pair<uint64_t, std::function<void()>>> tasks;

    ScopeMutex<Mutex> lock(mutex_);
    auto it = events_.begin();

    for(it = events_.begin(); it != events_.end(); ++ it){
        // 如果到期
        if((*it).second->getArriveTime() <= now ){
            if(!(*it).second->isCanceled()){
                tmps.emplace_back((*it).second);
                tasks.emplace_back(std::make_pair((*it).second->getArriveTime(), (*it).second->getCallBack()));
            }
        // 没有到期说明后面的也没到期
        } else {
            break;
        }
    }

    events_.erase(events_.begin(), it);
    lock.unlock();

    // 重新添加event
    for(auto i = tmps.begin(); i != tmps.end(); ++i){
        if((*i)->isRepeated()){
            (*i)->setArriveTime();
            addTimerEvent((*i));
        }
    }

    DEBUGLOG("Ontimer trigger event size %d:%d", tmps.size(), tasks.size());
    resetArriveTime();

    for(auto p = tasks.begin(); p != tasks.end(); ++p){
        if(p->second) p->second();
    }
}

void Timer::resetArriveTime(){
    ScopeMutex<Mutex> lock(mutex_);
    auto tmp = events_;
    lock.unlock();

    if(tmp.empty()){
        return;
    }

    int64_t now = get_now_ms();
    int64_t internal;
    auto it = tmp.begin();
    // 如果第一个任务还没执行, 设置间隔为
    internal = it->second->getArriveTime() - now;
    if(internal <= 0) internal = 10;

    DEBUGLOG("reset arrive time internal:%lld", internal);

    // s -> ms -> us -> ns
    timespec ts;
    bzero(&ts, sizeof(ts));
    ts.tv_sec = internal / 1000;
    ts.tv_nsec = (internal % 1000) * 1000000;

    itimerspec value;
    bzero(&value, sizeof(value));
    value.it_value = ts;

    // 告诉内核多久后唤醒我
    int rt = timerfd_settime(getFd(), 0, &value, NULL);
    if( rt != 0 ){
        ERRORLOG("timerfd_settime error, errno=%d, error=%s", errno, strerror(errno));
    }
}
} // namespace lrpc
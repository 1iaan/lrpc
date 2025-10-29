#pragma once

#include "lrpc/net/fd_event.h"
#include "lrpc/common/mutex.h"
#include "lrpc/net/timer.h"
#include "timer_event.h"
#include <queue>
#include <sched.h>
#include <set>
#include <functional>
namespace lrpc{

class EventLoop{
public:
    EventLoop();
    
    ~EventLoop();

public:
    void loop();
    
    void wakeup();

    void stop();

    // 如果是主线程执行就直接执行，不是主线程就丢进tasks_里等待主线程执行。
    void addEpollEvent(FdEvent* event);

    // 如果是主线程执行就直接执行，不是主线程就丢进tasks_里等待主线程执行。
    void delEpollEvent(FdEvent* event);

    bool isInLoopThread();

    void addTask(std::function<void()> callback, bool is_wake_up = false);

    void addTimerEvent(TimerEvent::s_ptr event);

private:
    pid_t tid_{0};

    int epoll_fd_{0};

    // 用来唤醒 epoll_wait
    int wakeup_fd_{0};

    WakeUpFdEvent *wakeup_fd_event_{NULL};

    // 定时器任务
    Timer *timer_{NULL};

    bool stop_{false};

    // 正在监听的事件
    std::set<int> listen_fds_;

    std::queue<std::function<void()>> tasks_;

    Mutex mutex_;

private:
    void dealWakeup();

    void initWakeUpFdEvent();

    void initTimer();
};

} // namespace lrpc
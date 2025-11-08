#include "lrpc/net/eventloop.h"
#include "lrpc/common/log.h"
#include "lrpc/common/util.h"
#include <cerrno>
#include <cstddef>
#include <string.h>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#define ADD_TO_EPOLL()  \
    auto it = listen_fds_.find(event->getFd()); \
    int op = EPOLL_CTL_ADD; \
    if(it != listen_fds_.end()){    \
        op = EPOLL_CTL_MOD; \
    }   \
    epoll_event tmp = event->getEpollEvent();   \
    int rt = epoll_ctl(epoll_fd_, op, event->getFd(), &tmp);    \
    if(rt == -1){   \
        ERRORLOG("faild epoll_ctl when add fd [%d:%s], errno=%d, errno=%s", event->getFd(), event->getFdName().c_str(), errno, strerror(errno)); \
    } else {  \
        listen_fds_.insert(event->getFd()); \
    } \
    DEBUGLOG("eventloop [%d] add event success, fd [%d:%s], event [%u]", tid_,  event->getFd(), event->getFdName().c_str(), tmp); \

#define DEL_TO_EPOLL() \
    auto it = listen_fds_.find(event->getFd()); \
    if(it == listen_fds_.end()){ \
        return; \
    } \
    int op = EPOLL_CTL_DEL; \
    epoll_event tmp = event->getEpollEvent(); \
    int rt = epoll_ctl(epoll_fd_, op, event->getFd(), nullptr); \
    if (rt == -1) { \
            ERRORLOG("faild epoll_ctl when add fd [%d:%s], errno=%d, errno=%s", event->getFd(), event->getFdName().c_str(), errno, strerror(errno)); \
    }else { \
        listen_fds_.erase(event->getFd()); \
    } \
    DEBUGLOG("eventloop [%d] delete event success, fd [%d:%s], event [%u]", tid_ , event->getFd(), event->getFdName().c_str(), tmp); \

namespace lrpc{

static thread_local EventLoop* t_cur_eventloop = NULL;
static int g_epoll_max_timeout = 10000;
static const int g_epoll_max_event = 10;

EventLoop::EventLoop(){
    INFOLOG("[Eventloop] Start create");
    if(t_cur_eventloop){
        ERRORLOG("faild to create event loop, already has a eventloop");
        exit(0);
    }
    tid_ = get_thread_id();
    
    epoll_fd_ = epoll_create(1);

    if(epoll_fd_ < 0) {
        ERRORLOG("[Eventloop] Failed to create event loop, epoll_create error, error info [%d]", errno);
        exit(0);
    }

    initWakeUpFdEvent(); 
    initTimer();

    INFOLOG("[Eventloop] Success create in thread [%d]", tid_);
    t_cur_eventloop = this;
}

EventLoop::~EventLoop(){
    close(epoll_fd_);
    if(wakeup_fd_event_){
        delete wakeup_fd_event_;
        wakeup_fd_event_ = NULL;
    }
}

void EventLoop::initWakeUpFdEvent(){
    wakeup_fd_ = eventfd(0, EFD_NONBLOCK);
    if(wakeup_fd_ <= 0){
        ERRORLOG("failed to create event loop, eventfd create error, error info [%d]", errno);
        exit(0);
    }

    wakeup_fd_event_ = new WakeUpFdEvent(wakeup_fd_);
    wakeup_fd_event_->listen(FdEvent::IN_EVENT, 
        [this]()->void{
            char buf[8];
            // -1&&EAGAIN 说明读完了
            while(read(wakeup_fd_, buf, 8) != -1 && errno != EAGAIN){

            }
            DEBUGLOG("read full bytes from wakeup fd[%d]", wakeup_fd_);
        }
    );
    addEpollEvent(wakeup_fd_event_);
}

void EventLoop::initTimer(){
    timer_ = new Timer();
    addEpollEvent(timer_);
}

void EventLoop::loop(){
    is_loop_ = true;
    while(!stop_){
        /* 执行 tasks_  */
        ScopeMutex<Mutex> lock(mutex_);
        std::queue<std::function<void()>> tmp_tasks;
        tasks_.swap(tmp_tasks);
        lock.unlock();

        while(!tmp_tasks.empty()){
            auto cb = tmp_tasks.front();
            tmp_tasks.pop();
            if(cb) cb();
        }

        /* 如果有定时任务需要执行，在这里执行 */
        /* 1. now() >= timerEvent.arrtive_time */
        /* 2. 如何让eventloop 监听 */

        int timeout = g_epoll_max_timeout;
        epoll_event result_event[g_epoll_max_event];
        int rt = epoll_wait(epoll_fd_, result_event, g_epoll_max_event, timeout);

        if(rt < 0){
            if(errno == EINTR) continue;
            ERRORLOG("epoll_wait error, errno=%d,error=%s", errno, strerror(errno));
            exit(0);
        }
        for(int i = 0;i < rt ;++ i){
            epoll_event trigger_event = result_event[i];
            FdEvent *fd_event = static_cast<FdEvent*>(trigger_event.data.ptr);
            if(fd_event == NULL) {
                continue;
            }

            if(trigger_event.events & EPOLLIN){
                DEBUGLOG("fd [%d:%s] trigger EPOLLIN event", fd_event->getFd(), fd_event->getFdName().c_str());
                addTask(fd_event->handler(FdEvent::IN_EVENT));
            }else if(trigger_event.events & EPOLLOUT){
                DEBUGLOG("fd [%d:%s] trigger EPOLLOUT event", fd_event->getFd(), fd_event->getFdName().c_str());
                addTask(fd_event->handler(FdEvent::OUT_EVENT));
            }

        }
    }
}

void EventLoop::wakeup(){
    wakeup_fd_event_->wakeup();
}

void EventLoop::stop(){
    stop_ = true;
}

void EventLoop::dealWakeup(){

}

void EventLoop::addEpollEvent(FdEvent* event){
    if(isInLoopThread()){
        ADD_TO_EPOLL();
    }else{
        auto callback = [this, event]()->void{
            ADD_TO_EPOLL();
        };
        addTask(callback,true);
    }
}

void EventLoop::delEpollEvent(FdEvent* event){
    if(isInLoopThread()){
        DEL_TO_EPOLL();
    }else{
        auto callback = [this, event]()->void{
            DEL_TO_EPOLL();
        };
        addTask(callback,true);
    }
}

bool EventLoop::isInLoopThread(){
    return get_thread_id() == tid_;
}

void EventLoop::addTask(std::function<void()> callback,  bool is_wake_up /*=false*/){
    ScopeMutex<Mutex> lock(mutex_);
    tasks_.push(callback);
    lock.unlock();

    if(is_wake_up){
        wakeup();
    }
}

void EventLoop::addTimerEvent(TimerEvent::s_ptr event){
    timer_->addTimerEvent(event);
}

EventLoop* EventLoop::GetCurEventLoop(){
    if(!t_cur_eventloop){
        t_cur_eventloop = new EventLoop();
    }   
    return t_cur_eventloop;
}

} // namespace lrpc
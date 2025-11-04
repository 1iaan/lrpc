#pragma once
#include <functional>
#include <string>
#include <sys/epoll.h>

namespace lrpc{



class FdEvent{
public:
    enum TriggerEvent {
        IN_EVENT = EPOLLIN,
        OUT_EVENT = EPOLLOUT,
    };

    FdEvent();
    FdEvent(int fd);
    FdEvent(int fd, std::string fd_name);
    
    ~FdEvent();
public:
    void setNonBlock();

    std::function<void()> handler(TriggerEvent ev_t);

    void listen(TriggerEvent ev_t, std::function<void()> callback);

    void cancel(TriggerEvent ev_t);

    int getFd() const { return fd_; }
    std::string getFdName() const { return fd_name_; }
    void setFdName(std::string name) { fd_name_ = name; }

    epoll_event getEpollEvent() { return listen_events_; }

private:
    int fd_{-1};
    std::string fd_name_;
    epoll_event listen_events_;
    std::function<void()> read_callback_;
    std::function<void()> write_callback_;

public:

};


class WakeUpFdEvent : public FdEvent{
public:
    WakeUpFdEvent(int fd);

    ~WakeUpFdEvent();

public:
    void wakeup();

private:

};


} // namespace lrpc
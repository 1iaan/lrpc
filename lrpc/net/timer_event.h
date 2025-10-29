#pragma once
#include <cstdint>
#include <functional>
#include <memory>


namespace lrpc{

class TimerEvent{
public:
    typedef std::shared_ptr<TimerEvent> s_ptr;

    TimerEvent(int internal, bool is_repeated, std::function<void()> callback);

public:
    void setArriveTime();

    int64_t getArriveTime() const { return arrive_time_; }

    bool isCanceled() const { return is_canceled_; }

    void setCanceled(bool c) { is_canceled_ = c; }

    bool isRepeated() const { return is_repeated_; }

    void setRepeated(bool r) { is_repeated_ = r; }

    std::function<void()> getCallBack() { return task_; }

private:
    int64_t arrive_time_;
    int64_t internal_;
    bool is_repeated_{false};
    bool is_canceled_{false};

    std::function<void()> task_;
};
    
} // namespace lrpc
#pragma once

#include "lrpc/net/fd_event.h"

namespace lrpc{

class WakeUpFdEvent : public FdEvent{
public:
    WakeUpFdEvent(int fd);

    ~WakeUpFdEvent();

public:
    void wakeup();

private:

};

}   // namespace lrpc
#pragma once 

#include "lrpc/common/mutex.h"
#include "lrpc/net/fd_event.h"
#include <cstddef>
#include <mutex>
#include <vector>

namespace lrpc{

class FdEventGroup{

public:
    FdEventGroup(size_t size);
    
    ~FdEventGroup();

public:
    FdEvent* getFdEvent(int fd);

    static FdEventGroup* GetFdEventGroup();

private:
    size_t size_{0};
    std::vector<FdEvent*> fd_group_;
    // Mutex mutex_;
    std::mutex mutex_;
};

} // namespace lrpc
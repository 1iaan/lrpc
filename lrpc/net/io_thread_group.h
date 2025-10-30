#pragma once
#include "lrpc/net/io_thread.h"
#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>


namespace lrpc {

class IOThreadGroup{
public:
    IOThreadGroup(int size);
    
    ~IOThreadGroup();

public: 
    void start();

    void join();

    IOThread* getIOThread();

public:

private:
    int size_;
    int index_;
    std::vector<IOThread*> io_thread_groups_;
};

} // namespace lrpc
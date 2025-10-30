#pragma once
#include "lrpc/net/eventloop.h"
#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>


namespace lrpc {

class IOThread{
public:
    IOThread();

    ~IOThread();

public:
    static void* Main(void *args);

    EventLoop* getEventloop() const { return eventloop_; }

    void start();

    void join();

private:
    pid_t tid_ {0};                // 线程号
    pthread_t thread_ {0};         // 线程句柄
    EventLoop* eventloop_ {NULL};   // io线程的loop

    sem_t init_semaphore_;
    sem_t start_semaphore_;
};

} // namespace lrpc
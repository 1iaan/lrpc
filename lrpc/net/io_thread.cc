#include "lrpc/net/io_thread.h"
#include "eventloop.h"
#include "lrpc/common/util.h"
#include "lrpc/common/log.h"
#include <cassert>
#include <cstddef>
#include <pthread.h>
#include <semaphore.h>

namespace lrpc {

IOThread::IOThread(){
    INFOLOG("[IOThread] Start create" );
    int rt = sem_init(&init_semaphore_, 0, 0);
    assert(rt == 0);
    rt = sem_init(&start_semaphore_, 0, 0);
    assert(rt == 0);
    pthread_create(&thread_, NULL, &IOThread::Main, this);

    // 需要等到Main函数的eventloop循环启动才返回
    sem_wait(&init_semaphore_);

    INFOLOG("[IOThread] Success create in thread [%d]", tid_);
}

IOThread::~IOThread(){
    ERRORLOG("~IOThread");
    if(eventloop_) eventloop_->stop();
    sem_destroy(&init_semaphore_);
    pthread_join(thread_, NULL);

    if(eventloop_){
        delete eventloop_;
        eventloop_ = NULL;
    }
}

void* IOThread::Main(void *args){
    IOThread* thread = static_cast<IOThread*>(args);

    thread->eventloop_ = new EventLoop();
    thread->tid_ = get_thread_id();

    sem_post(&thread->init_semaphore_);

    // DEBUGLOG("IOThread [%d] wait start", thread->tid_);
    sem_wait(&thread->start_semaphore_);

    DEBUGLOG("IOThread [%d] start eventloop", thread->tid_);
    thread->eventloop_->loop();
    DEBUGLOG("IOThread [%d] end eventloop", thread->tid_);
    return nullptr;
}

void IOThread::start(){
    sem_post(&start_semaphore_);
}

void IOThread::join(){
    pthread_join(thread_, NULL);
}
    
} // namespace lrpc
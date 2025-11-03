#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include <cstddef>
#include <pthread.h>
#include <string>

void *func(void*){
    int i = 100;
    while (i --){
        DEBUGLOG("this is thread in %s", "func");
        INFOLOG("this is thread in %s", "func");
        ERRORLOG("this is thread in %s", "func");
    }

    return NULL;
}

int main(){

    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    pthread_t thread;
    pthread_create(&thread, NULL, func, NULL);

    int i = 100;
    while (i --){ 
        DEBUGLOG("test debug log %s", std::to_string(i).c_str());
        INFOLOG("test info log %s", std::to_string(i).c_str());
        ERRORLOG("test error log %s", std::to_string(i).c_str());
    }
    pthread_join(thread, NULL);

    return 0;
}
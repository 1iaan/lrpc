#include "lrpc/common/util.h"
#include <unistd.h>
#include <sys/syscall.h>

namespace lrpc {
static int g_pid = 0;
static thread_local int g_thread_id = 0;

pid_t get_process_id(){
    if(g_pid != 0){
        return g_pid;
    }
    return (g_pid = getpid());
}

pid_t get_thread_id(){
    if(g_thread_id != 0){
        return g_thread_id;
    }
    return (g_thread_id = syscall(SYS_gettid));
}

} // namespace lrpc
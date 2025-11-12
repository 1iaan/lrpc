#include "lrpc/common/util.h"
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <netinet/in.h>
#include <sys/select.h>
#include <unistd.h>
 #include <sys/time.h>
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

int64_t get_now_ms(){
    timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec * 1000 + time.tv_usec / 1000;
}

std::string get_now_str(){
struct timeval now_time;
    gettimeofday(&now_time, nullptr);
    
    struct tm now_time_t;
    localtime_r(&(now_time.tv_sec), &now_time_t);

    char buf[128];
    strftime(&buf[0], 128, "%y-%m-%d %H:%M:%S", &now_time_t);
    
    char ms_buf[8];
    int ms = now_time.tv_usec / 1000;
    snprintf(ms_buf, sizeof(ms_buf), ".%03d", ms);
    
    return std::string(buf) + ms_buf;
}

// long 32 4B
int32_t getInt32FromNetByte(const char* buf){
    int32_t re;
    memcpy(&re, buf, sizeof(re));
    return ntohl(re);
}

} // namespace lrpc
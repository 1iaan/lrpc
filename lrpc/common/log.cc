#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/common/mutex.h"
#include "lrpc/common/util.h"
#include <cstdio>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <time.h>



namespace lrpc {

static Logger* g_logger = nullptr;

void Logger::SetGlobalLogger(){
    if(!g_logger){
        printf("Init Logger module\n");
        LogLevel level = StringToLogLevel( Config::GetGlobalConfig()->log_level_);
        g_logger = new Logger(level);
    }
}

Logger* Logger::GetGlobalLogger(){
    return g_logger;
}

std::string LogLevelToString(LogLevel level){
    switch (level) {
        case LogLevel::Debug: {
            return "DEBUG";
        }
        case LogLevel::Info: {
            return "INFO";
        }
        case LogLevel::Error: {
            return "ERROR";
        }
        default:
            return "UNKNOWN";
    }
}

LogLevel StringToLogLevel(const std::string& log_level) {
  if (log_level == "DEBUG") {
    return LogLevel::Debug;
  } else if (log_level == "INFO") {
    return LogLevel::Info;
  } else if (log_level == "ERROR") {
    return LogLevel::Error;
  } else {
    return LogLevel::Unknown;
  }
}

std::string LogEvent::toString(){
    struct timeval now_time;
    gettimeofday(&now_time, nullptr);
    
    struct tm now_time_t;
    localtime_r(&(now_time.tv_sec), &now_time_t);

    char buf[128];
    strftime(&buf[0], 128, "%y-%m-%d %H:%M:%S", &now_time_t);
    
    ctime_ = buf;
    int ms = now_time.tv_usec * 1000;
    ctime_ = ctime_ + "." + std::to_string(ms);

    pid_ = get_process_id();
    tid_ = get_thread_id();

    std::stringstream ss;
    ss  << "["  << LogLevelToString(log_level_) << "]\t"
        << "["  << ctime_ <<"]\t"   
        << "["  << pid_ << ":" << tid_ << "]\t";

    return ss.str();
}

void Logger::pushLog(const std::string &msg){
    ScopeMutex<Mutex> lock(mutex_);
    buffer_.push(msg);
}

void Logger::log(){
    ScopeMutex<Mutex> lock(mutex_);
    while (!buffer_.empty()) {
        std::string msg = buffer_.front();
        buffer_.pop();

        printf("%s", msg.c_str());
    }
}


} // namespace lrpc
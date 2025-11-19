#pragma once
#include "lrpc/common/mutex.h"
#include "lrpc/net/timer.h"
#include "lrpc/net/timer_event.h"
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <semaphore.h>
#include <string>
#include <thread>
#include <vector>

namespace lrpc {

// [%y-%m-%d %H:%M:%s.%ms]\t[pid:tid]\t[file_name:line][%msg]
inline std::string formatString(const char* fmt) {
    return std::string(fmt);
}

template<typename... Args>
std::string formatString(const char* str, Args&& ...args){
    int size = snprintf(nullptr, 0, str, args...); // 获取格式化长度
    std::string result(size, '\0');
    snprintf(&result[0], size + 1, str, args...);
    return result;
}

#define DEBUGLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Debug) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Debug)).toString() + _locBuf \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
    }\

#define INFOLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Info) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Info)).toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
    }\

#define ERRORLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Error) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Error)).toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
    }\


#define APPDEBUGLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Debug) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Debug)).toString() + _locBuf \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushAppLog(_msg_);\
    }\

#define APPINFOLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Info) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Info)).toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushAppLog(_msg_);\
    }\

#define APPERRORLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Error) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (lrpc::LogEvent(lrpc::LogLevel::Error)).toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushAppLog(_msg_);\
    }\

enum class LogLevel{
    Unknown = 0,
    Debug = 1, 
    Info = 2,
    Error = 3,
};

class AsyncLogger{
public:
    typedef std::shared_ptr<AsyncLogger> s_ptr;
    AsyncLogger(const std::string& file_name, const std::string& file_path, int max_size);

public:
    void Loop();

    void stop();

    void flush();

    void pushLogToBuffer(std::vector<std::string>& vec);

private:
    std::queue<std::vector<std::string>> buffer_;

    // file_path_/file_name_yyyymmdd.1
    std::string file_name_; // 文件名
    std::string file_path_; // 日志输出路径

    int max_file_size_{0};  // 单个文件最大大小

    sem_t semaphore_{0};
    std::thread thread_;

    std::condition_variable cv_;
    std::mutex mutex_;

    std::string date_;          // 当前打印日志的文件日期
    FILE* file_handler_{NULL};  // 当前打开的日志文件句柄

    bool reopen_flag_{false};

    int file_no_{0};            // 日志文件序号

    bool stop_{false};
};

std::string LogLevelToString(LogLevel level);
LogLevel StringToLogLevel(const std::string& log_level);

class Logger{
public:
    typedef std::shared_ptr<Logger> s_ptr;
    
    Logger(LogLevel level);

public: 
    static Logger* GetGlobalLogger();

    static void SetGlobalLogger();

public:
    void init();

    void pushLog(const std::string &msg);

    void pushAppLog(const std::string &msg);

    void log();

    LogLevel getLogLevel() { return log_level_; }

    void syncLoop();

private:
    LogLevel log_level_;
    // std::queue<std::string> buffer_;
    std::vector<std::string> buffer_;
    std::vector<std::string> app_buffer_;

    // Mutex mutex_;
    std::mutex mutex_;
    std::mutex app_mutex_;

    AsyncLogger::s_ptr async_logger_;

    AsyncLogger::s_ptr async_app_logger_;
    
    TimerEvent::s_ptr timer_event_;
};

class LogEvent {
public:
    LogEvent(LogLevel level): log_level_(level){}

public:
    std::string getFileName() const { return filename_; }

    LogLevel getLogLevel() const { return log_level_; }

    std::string toString();
    
private:
    std::string filename_;  // 文件名
    std::string fileline_;  // 行号
    int pid_;               // 进程号
    int tid_;               // 线程号
    std::string ctime_;     // 时间
    LogLevel log_level_;        // 日志级别

};

} // namespace lrpc
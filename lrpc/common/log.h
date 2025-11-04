#pragma once
#include "lrpc/common/mutex.h"
#include <cstdio>
#include <queue>
#include <string>

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
        std::string _msg_ = (new lrpc::LogEvent(lrpc::LogLevel::Debug))->toString() + _locBuf \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define INFOLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Info) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (new lrpc::LogEvent(lrpc::LogLevel::Info))->toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define ERRORLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Error) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string _msg_ = (new lrpc::LogEvent(lrpc::LogLevel::Error))->toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(_msg_);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

enum class LogLevel{
    Unknown = 0,
    Debug = 1, 
    Info = 2,
    Error = 3,
};

std::string LogLevelToString(LogLevel level);
LogLevel StringToLogLevel(const std::string& log_level);

class Logger{
public:
    // typedef std::shared_ptr<Logger> s_ptr;
    
    Logger(LogLevel level): log_level_(level){}

public: 
    static Logger* GetGlobalLogger();

    static void SetGlobalLogger();

public:
    void pushLog(const std::string &msg);

    void log();

    LogLevel getLogLevel() { return log_level_; }

private:
    LogLevel log_level_;
    std::queue<std::string> buffer_;
    Mutex mutex_;

private:


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
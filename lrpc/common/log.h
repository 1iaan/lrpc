#pragma once
#include "lrpc/common/mutex.h"
#include <cstdio>
#include <queue>
#include <string>

namespace lrpc {

// [%y-%m-%d %H:%M:%s.%ms]\t[pid:tid]\t[file_name:line][%msg]
template<typename... Args>
std::string formatString(const char* str, Args&& ...args){
    int size = snprintf(nullptr, 0, str, args...); // 获取格式化长度
    if(size <= 0) return {};
    
    std::string result;
    result.resize(size);
    snprintf(&result[0], size + 1, str, args...);
    return result;
}

#define DEBUGLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Debug) {\
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Debug))->toString() + "[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]\t" + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define INFOLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Info) {\
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Info))->toString() + "[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]\t" + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define ERRORLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Error) {\
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Error))->toString() + "[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]\t" + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
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
#pragma once
#include "mutex.h"
#include <cstdio>
#include <queue>
#include <string>

namespace lrpc {

// [%y-%m-%d %H:%M:%s.%ms]\t[pid:tid]\t[file_name:line][%msg]
template<typename... Args>
std::string formatString(const char* str, Args&& ...args){
    int size = snprintf(nullptr, 0, str, args...); // 获取格式化长度
    std::string result;
    if(size > 0){
        result.resize(size);
        snprintf(&result[0], size + 1, str, args...);
    }
    return result;
}

#define DEBUGLOG(str, ...)\
    std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Debug))->toString() + lrpc::formatString(str, ##__VA_ARGS__); \
    msg += "\n";\
    lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
    lrpc::Logger::  GetGlobalLogger()->log();\

enum class LogLevel{
    Unknown = 0,
    Debug = 1, 
    Info = 2,
    Error = 3,
};

std::string LogLevelToString(LogLevel level);


class Logger{
public:
    // typedef std::shared_ptr<Logger> s_ptr;

    void pushLog(const std::string &msg);
    void log();
public:
    static Logger* GetGlobalLogger();
private:
    LogLevel level_;
    std::queue<std::string> buffer_;
    Mutex mutex_;

private:


};

class LogEvent {
public:
    LogEvent(LogLevel level): level_(level){}

    std::string getFileName() const { return filename_; }

    LogLevel getLogLevel() const { return level_; }

    std::string toString();
private:
    std::string filename_;  // 文件名
    std::string fileline_;  // 行号
    int pid_;               // 进程号
    int tid_;               // 线程号
    std::string ctime_;     // 时间
    LogLevel level_;        // 日志级别

};

} // namespace lrpc
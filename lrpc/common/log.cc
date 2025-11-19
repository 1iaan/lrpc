#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/common/mutex.h"
#include "lrpc/common/run_time.h"
#include "lrpc/common/util.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/net/timer_event.h"
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <semaphore.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <thread>
#include <vector>

namespace lrpc {

static Logger* g_logger = nullptr;

void Logger::SetGlobalLogger(){
    if(!g_logger){
        LogLevel level = StringToLogLevel( Config::GetGlobalConfig()->log_level_);
        g_logger = new Logger(level);
        g_logger->init();
    }
}

Logger* Logger::GetGlobalLogger(){
    return g_logger;
}

Logger::Logger(LogLevel level): log_level_(level){

}

void Logger::init(){
    async_logger_ = std::make_shared<AsyncLogger>(
        Config::GetGlobalConfig()->log_file_name_ + "_rpc",
        Config::GetGlobalConfig()->log_file_path_, 
        Config::GetGlobalConfig()->log_max_file_size_
    );

    async_app_logger_ = std::make_shared<AsyncLogger>(
        Config::GetGlobalConfig()->log_file_name_ + "_app",
        Config::GetGlobalConfig()->log_file_path_, 
        Config::GetGlobalConfig()->log_max_file_size_
    );

    timer_event_ = std::make_shared<TimerEvent>(
        Config::GetGlobalConfig()->log_sync_internal_,
        true,
        std::bind(&Logger::syncLoop, this)
    );

    EventLoop::GetCurEventLoop()->addTimerEvent(timer_event_);
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
    // struct timeval now_time;
    // gettimeofday(&now_time, nullptr);
    
    // struct tm now_time_t;
    // localtime_r(&(now_time.tv_sec), &now_time_t);

    // char buf[128];
    // strftime(&buf[0], 128, "%y-%m-%d %H:%M:%S", &now_time_t);
    
    // ctime_ = buf;
    // int ms = now_time.tv_usec / 1000;
    // ctime_ = ctime_ + "." + std::to_string(ms);
    ctime_ = get_now_str();

    pid_ = get_process_id();
    tid_ = get_thread_id();

    std::stringstream ss;
    ss  << "["  << LogLevelToString(log_level_) << "]\t"
        << "["  << ctime_ <<"]\t"   
        << "["  << pid_ << ":" << tid_ << "]\t";



    // 获取当前线程请求的 msg_id_/method_name_
    std::string msg_id = RunTime::GetRunTime()->msg_id_;
    std::string method_name = RunTime::GetRunTime()->method_name_;
    if(!msg_id.empty()){
        ss << "[" << msg_id << "]\t";
    }
    if(!method_name.empty()){
        ss << "[" << method_name << "]\t";
    }
    return ss.str();
}

void Logger::pushLog(const std::string &msg){
    // ScopeMutex<Mutex> lock(mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    buffer_.push_back(msg);
    lock.unlock();
}

void Logger::pushAppLog(const std::string &msg){
    std::unique_lock<std::mutex> lock(app_mutex_);
    app_buffer_.push_back(msg);
    lock.unlock();
}


void Logger::log(){
    // ScopeMutex<Mutex> lock(mutex_);
    // std::unique_lock<std::mutex> lock(mutex_);
    // std::queue<std::string> tmp;
    // lock.unlock();

    // while (!buffer_.empty()) {
    //     std::string msg = tmp.front();
    //     tmp.pop();

    //     printf("%s", msg.c_str());
    // }

}

void Logger::syncLoop(){
    // 同步 buffer 到 async 的 buffer
    std::vector<std::string> tmp;
    
    std::unique_lock<std::mutex> lock(mutex_);
    tmp.swap(buffer_);
    lock.unlock();

    if(!tmp.empty()) {
        async_logger_->pushLogToBuffer(tmp);
    }

    // 同步 app_buffer 
    std::vector<std::string> tmp2;
    
    std::unique_lock<std::mutex> app_lock(app_mutex_);
    tmp2.swap(app_buffer_);
    app_lock.unlock();

    if(!tmp2.empty()) {
        async_app_logger_->pushLogToBuffer(tmp2);
    }
}


AsyncLogger::AsyncLogger(const std::string& file_name, const std::string& file_path, int max_size)
    :file_name_(file_name), file_path_(file_path), max_file_size_(max_size){
    sem_init(&semaphore_, 0, 0);

    thread_ = std::thread(&AsyncLogger::Loop, this);

    sem_wait(&semaphore_);
}

void AsyncLogger::Loop(){
    sem_post(&(this->semaphore_));

    while(!stop_){
        std::unique_lock<std::mutex> lock(this->mutex_);
        this->cv_.wait(lock, [this]()->bool{
            return !this->buffer_.empty();
        });

        std::vector<std::string> tmp;
        tmp.swap(this->buffer_.front());
        this->buffer_.pop();
        lock.unlock();

        timeval now;
        gettimeofday(&now, NULL);

        struct tm now_time;
        localtime_r(&(now.tv_sec), &now_time);

        const char* format = "%Y%m%d";
        char date[32];
        strftime(date, sizeof(date), format, &now_time);

        // 如果不是同一个日期
        if(std::string(date) != this->date_){
            this->file_no_ = 0;
            this->reopen_flag_ = true;
            this->date_ = std::string(date);
        }
        // 如果没有打开过文件
        if(this->file_handler_ == NULL){
            this->reopen_flag_ = true;
        }

        std::stringstream ss;
        ss << this->file_path_ << this->file_name_ << "_"
            << std::string(date) << "_log.";
        std::string log_file_name = ss.str() + std::to_string(this->file_no_);

        if(this->reopen_flag_){
            if(this->file_handler_){
                fclose(this->file_handler_);
            }
            this->file_handler_ = fopen(log_file_name.c_str(), "a");
            this->reopen_flag_ = false;
        }

        if(ftell(this->file_handler_) > this->max_file_size_){
            fclose(this->file_handler_);
            this->file_no_ += 1;

            log_file_name = ss.str() + std::to_string(this->file_no_);
            this->file_handler_ = fopen(log_file_name.c_str(), "a");
            this->reopen_flag_ = false;
        }

        for(auto &i : tmp){
            if(!i.empty()){
                fwrite(i.c_str(), 1, i.length(), this->file_handler_);
            }
        }

        fflush(this->file_handler_);
    }
}

void AsyncLogger::stop(){
    stop_ = true;
}

void AsyncLogger::flush(){
    if(file_handler_){
        fflush(file_handler_);
    }
}

void AsyncLogger::pushLogToBuffer(std::vector<std::string>& vec){
    std::unique_lock<std::mutex> lock(mutex_);
    buffer_.emplace(vec);
    lock.unlock();

    // 唤醒
    cv_.notify_one();
}

} // namespace lrpc
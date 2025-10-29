
学习用个人rpc框架实现

- [前置知识](#前置知识)
- [环境](#环境)
- [Config](#config)
  - [类设计](#类设计)
  - [xml读取宏](#xml读取宏)
  - [config.cc](#configcc)
- [Mutex](#mutex)
  - [mutex.h](#mutexh)
  - [参考](#参考)
- [Logger](#logger)
  - [类设计](#类设计-1)
  - [formatString](#formatstring)
  - [日志宏](#日志宏)
  - [log.cc](#logcc)
- [Reactor(EventLoop)](#reactoreventloop)
  - [Reactor 模型概念](#reactor-模型概念)
  - [FdEvent](#fdevent)
    - [fd\_event.h](#fd_eventh)
    - [fd\_event.cc](#fd_eventcc)
  - [WakeUpFdEvent](#wakeupfdevent)
  - [EventLoop](#eventloop)
    - [eventloop.h](#eventlooph)
    - [添加EPOLL事件宏](#添加epoll事件宏)
    - [删除EPOLL事件宏](#删除epoll事件宏)
    - [eventloop.cc](#eventloopcc)
      - [初始化与析构](#初始化与析构)
      - [主循环](#主循环)
      - [辅助函数](#辅助函数)
      - [添加/删除EPOLL操作](#添加删除epoll操作)
  - [TimerEvent](#timerevent)
    - [timer\_event.h](#timer_eventh)
    - [timer\_event.cc](#timer_eventcc)
  - [Timer](#timer)
    - [timer.h](#timerh)
    - [timer.cc](#timercc)
      - [addTimerEvent/deleteTimerEvent](#addtimereventdeletetimerevent)
      - [onTimer](#ontimer)
      - [resetArriveTime](#resetarrivetime)
- [TCP](#tcp)
- [RPC](#rpc)
  - [协议封装](#协议封装)
  - [模块封装](#模块封装)
- [项目完善](#项目完善)
- [测试](#测试)
  - [log](#log)
  - [eventloop](#eventloop-1)
- [结语](#结语)



# 前置知识
1. C++ 基础语法
2. Linux 环境，Linux 网络编程、Socket 编程
3. Reactor 架构
4. Git
5. 计算机网络知识 
6. ProtoBuf
# 环境
```
protobuf-cpp 
tinyxml 2.6.2
```

# Config
用xml读取配置文件。  
## 类设计
```c++
class Config{
public:
    Config(const char* xmlfile);

public:
    static void SetGlobalConfig(const char* xmlfile);
    static Config* GetGlobalConfig();

public:
    std::string log_level_;
};
```
## xml读取宏
##代表拼接成一个的变量声明  
#代表转字符串
```c++
TiXmlElement *root_node = xml_document->FirstChildElement("root");
if (!root_node) {
  printf("Start lrpc server error, failed to read node [%s]\n", "root");
  exit(0);
}

TiXmlElement *log_level_node = log_node->FirstChildElement("log_level");
if (!log_level_node || !log_level_node->GetText()) {
  printf("Start lrpc server error, failed to read node [%s]\n", "log_level");
}
std ::string log_level = std ::string(log_level_node->GetText());
```
```c++
#define READ_XML_NODE(name, parent)                                             \
    TiXmlElement* name#_node = parent->FirstChildElement(#name);               \
    if(!name#_node){                                                           \
        printf("Start lrpc server error, failed to read node [%s]\n", #name);   \
        exit(0);                                                                \
    }                                                                           \

#define READ_STR_FROM_XML_NODE(name, parent)                                    \
    TiXmlElement* name#_node = parent->FirstChildElement(#name);               \
    if(!name#_node || !name#_node->GetText()) {                               \
        printf("Start lrpc server error, failed to read node [%s]\n", #name);   \
    }                                                                           \
    std::string name = std::string(name#_node->GetText());                     \
`
```
## config.cc
```c++
static Config* g_config = NULL;
void Config::SetGlobalConfig(const char* xmlfile){
    if(g_config == NULL){
        g_config = new Config(xmlfile);
    }
}

Config* Config::GetGlobalConfig(){
    return g_config;
}

Config::Config(const char* xmlfile){
    TiXmlDocument* xml_document = new TiXmlDocument();
    bool rt = xml_document->LoadFile(xmlfile);

    if(!rt){
        printf("Start lrpc server error, failed to read config file %s, error info [%s]\n", xmlfile, xml_document->ErrorDesc());
        exit(0);
    }

    READ_XML_NODE(root, xml_document);
    READ_XML_NODE(log, root_node);

    READ_STR_FROM_XML_NODE(log_level, log_node);

    log_level_ = log_level;
    
    printf("Config: \n");
    printf("\tlog_level:\t[%s]\n", log_level_.c_str());
}
```

# Mutex
对 POSIX  API 做一下封装  
这里直接用C++ mutex 也可以，内部实现是差不多的  
## mutex.h
```c++
/**
 * @brief 
 * RAII锁
 * @tparam T 
 */
template<class T>
class ScopeMutex{
public:
    ScopeMutex(T& mutex): mutex_(mutex){
        mutex_.lock();
        is_lock_ = true;
    }
    ~ScopeMutex(){
        mutex_.unlock();
        is_lock_ = false;
    }

    void lock() {
        if(!is_lock_){
            mutex_.lock();
        }
    }

    void unlock(){
        if(is_lock_){
            mutex_.unlock();
        }
    }

private:
    T& mutex_;
    bool is_lock_ {false};
};

/**
 * @brief 封装pthread锁，类似std::mutex
 * 
 */
class Mutex{
public: 
    Mutex() {
        pthread_mutex_init(&mutex_, NULL);
    }

    ~Mutex(){
        pthread_mutex_destroy(&mutex_);
    }

    void lock(){
        pthread_mutex_lock(&mutex_);
    }

    void unlock(){
        pthread_mutex_unlock(&mutex_);
    }

private:
    pthread_mutex_t mutex_;
};

}
```

## 参考
```c++
// 构造函数被调用   lock_guard 会自动锁定
// 析构函数被调用   lock_guard 会自动解锁
template<typename _Mutex>  // 模板类型，希望传递进来的类型是有 lock和unlock方法的锁
class lock_guard
{
public:
    typedef _Mutex mutex_type;

    // 构造，加锁  explicit 禁止隐式推导
    explicit lock_guard(mutex_type& __m) : _M_device(__m)
    { _M_device.lock(); }

    // std::adopt lock
    lock_guard(mutex_type& __m, adopt_lock_t) noexcept : _M_device(__m)
    { } // calling thread owns mutex

    // 析构，解锁
    ~lock_guard()
    { _M_device.unlock(); }

    lock_guard(const lock_guard&) = delete;
    lock_guard& operator=(const lock_guard&) = delete;

private:
    mutex_type&  _M_device;
};
```

# Logger

日志模块：
1. 日志级别
2. 打印到文件，支持日期命名，日志滚动。
3. c 格式化风控
4. 线程安全  

日志格式
`[Level][%y-%m-%d %H:%M:%s.%ms]\t[pid:thread_id]\t[file_name:line][%msg]`   

## 类设计
```c++
enum class LogLevel{
    Unknown = 0,
    Debug = 1, 
    Info = 2,
    Error = 3,
};

std::string LogLevelToString(LogLevel level);
LogLevel StringToLogLevel(const std::string& log_level);

class LogEvent {
public:
    LogEvent(LogLevel level): log_level_(level){}

public:
    std::string getFileName() const { return filename_; }
    LogLevel getLogLevel() const { return log_level_; }
    std::string toString();

private:
    std::string filename_;      // 文件名
    std::string fileline_;      // 行号
    int pid_;                   // 进程号
    int tid_;                   // 线程号
    std::string ctime_;         // 时间
    LogLevel log_level_;        // 日志级别

};

class Logger{
public:
    // typedef std::shared_ptr<Logger> s_ptr;
    Logger(LogLevel level): log_level_(level){}

public:
    static Logger* GetGlobalLogger();
    static void SetGlobalLogger();
    void pushLog(const std::string &msg);
    void log();
    LogLevel getLogLevel() { return log_level_; }

private:
    LogLevel log_level_;
    std::queue<std::string> buffer_;
    Mutex mutex_;
};
```
## formatString
 实现 `“printf 风格字符串拼接到 std::string”` 的功能，如 `printf("%s, %d", "str", 11);` 等。
 
```c++
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
```
## 日志宏
解释：
如全局日志等级为 INFO, 那么打印 INFO, ERROR, 忽略 INFO > DEBUG  

msg = LogEvent([level][time][pid:tid]) + [文件名:行号] + formatString(msg);  

最后push到全局的logger的队列中  

__FILE__ const char *   

__LINE__ int   

... 是宏参数占位符    

__VA_ARGS__ 是预定义宏名，用于展开所有传入的可变参数    

 当宏参数为空时，##__VA_ARGS__ 会自动去掉前一个逗号  

```c++
// 如全局日志等级为 INFO, 那么打印 INFO, ERROR, 忽略 INFO > DEBUG
if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Debug) {
    // msg = LogEvent([level][time][pid:tid]) + [文件名:行号] + formatString(msg);
    std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Debug))->toString() 
        + "[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]\t" 
        + lrpc::formatString(str, #__VA_ARGS__) + "\n"; 
    lrpc::Logger::GetGlobalLogger()->pushLog(msg);
    lrpc::Logger::  GetGlobalLogger()->log();
}
```
```c++

#define DEBUGLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Debug) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Debug))->toString() + _locBuf \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define INFOLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Info) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Info))->toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\

#define ERRORLOG(str, ...)\
    if(lrpc::Logger::GetGlobalLogger()->getLogLevel() <= lrpc::LogLevel::Error) {\
        char _locBuf[64]; snprintf(_locBuf, sizeof(_locBuf), "%-40s", ("[" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "]").c_str()); \
        std::string msg = (new lrpc::LogEvent(lrpc::LogLevel::Error))->toString() + _locBuf  \
        + lrpc::formatString(str, ##__VA_ARGS__) + "\n"; \
        lrpc::Logger::GetGlobalLogger()->pushLog(msg);\
        lrpc::Logger::  GetGlobalLogger()->log();\
    }\
```
## log.cc
```c++
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
```

# Reactor(EventLoop)
EventLoop 即 Reactor架构的核心逻辑。  

实现定时器。  

IO线程封装。  
## Reactor 模型概念
Reactor 模式是一种事件驱动的设计模式，  
让一个或多个线程负责：  
1. 等待事件（I/O 就绪）  
2. 分发事件（调用相应的事件处理器）  
让一个或多个线程负责：
1. 执行任务

目前的实现是负责Reactor的线程也负责执行，也就是单个线程同时负责等待、分发、执行。

## FdEvent
作用：把callback、fd与epoll事件绑定起来。

1. listen: 给fd绑定回调。
2. handler： 返回fd对应的回调。
3. getEpollEvent: 返回fd上绑定的事件。



### fd_event.h
```c++
enum TriggerEvent {
    IN_EVENT = EPOLLIN,
    OUT_EVENT = EPOLLOUT,
};

class FdEvent{
public:

    FdEvent(int fd);
    FdEvent(int fd, std::string fd_name);
    
    ~FdEvent();

    std::function<void()> handler(TriggerEvent ev_t);

    void listen(TriggerEvent ev_t, std::function<void()> callback);

    int getFd() const { return fd_; }
    std::string getFdName() { return fd_name_; }

    epoll_event getEpollEvent() { return listen_events_; }

private:
    int fd_{-1};
    std::string fd_name_;
    epoll_event listen_events_;
    std::function<void()> read_callback_;
    std::function<void()> write_callback_;
};

```

### fd_event.cc
```c++
FdEvent::FdEvent(int fd): fd_(fd){
    fd_name_ = "fd_" + std::to_string(fd);
    memset(&listen_events_, 0, sizeof(listen_events_));
    listen_events_.data.ptr = this;
}


FdEvent::FdEvent(int fd, std::string fd_name): fd_(fd), fd_name_(fd_name){
    memset(&listen_events_, 0, sizeof(listen_events_));
    listen_events_.data.ptr = this;
}

FdEvent::~FdEvent(){

}

std::function<void()> FdEvent::handler(TriggerEvent ev_t){
    switch (ev_t) {
        case TriggerEvent::IN_EVENT:{
            return read_callback_;
        }
        case TriggerEvent::OUT_EVENT:{
            return write_callback_;            
        }
        default:
        break;
    }
}

void FdEvent::listen(TriggerEvent ev_t, std::function<void()> callback){
    switch (ev_t) {
        case TriggerEvent::IN_EVENT:{
            listen_events_.events |= EPOLLIN;
            read_callback_ = callback;
            break;
        }
        case TriggerEvent::OUT_EVENT:{
            listen_events_.events |= EPOLLOUT;
            write_callback_ = callback;       
            break; 
        }
        default:
        break;
    }
}


```

## WakeUpFdEvent
作用：将主线程从EPOLL_WAIT中唤醒。
1. 其他线程添加EpollEvent。
2. 定时器到时间。

EventLoop初始化的时候定义了该对象, 并且监听EPOLLIN, 定义回调函数为消费这些数据。

其wakeup方法会向wakeup_fd_event_写入数据, 从而唤醒epoll_wait。  
可以是其他线程AddTask调用，也可以是定时器调用。

感觉没有必要抽象出一个类呢。但为了单独说明其重要性还是抽象出来。
```c++
class WakeUpFdEvent : public FdEvent{
public:
    WakeUpFdEvent(int fd);

    ~WakeUpFdEvent();

public:
    void wakeup();
};

WakeUpFdEvent::WakeUpFdEvent(int fd) : FdEvent(fd, "WAKEUP"){

}

// 作用：在其他线程添加EpollEvent的时候，将主线程从EPOLL_WAIT中唤醒。
void WakeUpFdEvent::wakeup(){
    uint64_t one = 1;
    int rt = write(fd_, &one, sizeof(one));
    if(rt != 8){
        ERRORLOG("write to wakeup fd less than 8 bytes, fd[%d]", fd_);
    }
}
```

## EventLoop
eventloop
```c++
while (!stop_) {
    执行 tasks_ 里的任务（来自其他线程）
    epoll_wait() 等待事件
    针对对应的事件，调用 addTask 把其对应的回调加入到 tasks_ 中 
}
```
addEpollEvent 注册事件
```c++
if(是本线程){
    添加EpollEvent, 也就通过epoll_ctl注册
}else{
    把添加EpollEvent打包成任务, 立刻通过WakeUpFdEvent唤醒Reactor线程执行添加操作
}
```
delEpollEvent 删除事件
```c++
if(是本线程){
    删除EpollEvent, 也就通过epoll_ctl删除
}else{
    把删除EpollEvent打包成任务, 立刻通过WakeUpFdEvent唤醒Reactor线程执行删除操作
}
```

### eventloop.h
```c++
class EventLoop{
public:
    EventLoop();
    
    ~EventLoop();

public:
    void loop();
    
    void wakeup();

    void stop();

    // 如果是主线程执行就直接执行，不是主线程就丢进tasks_里等待主线程执行。
    void addEpollEvent(FdEvent* event);

    // 如果是主线程执行就直接执行，不是主线程就丢进tasks_里等待主线程执行。
    void delEpollEvent(FdEvent* event);

    bool isInLoopThread();

    void addTask(std::function<void()> callback, bool is_wake_up = false);

private:
    pid_t tid_{0};

    int epoll_fd_{0};

    // 用来唤醒 epoll_wait
    int wakeup_fd_{0};

    WakeUpFdEvent *wakeup_fd_event_{NULL};

    bool stop_{false};

    std::set<int> listen_fds_;

    std::queue<std::function<void()>> tasks_;

    Mutex mutex_;

private:
    void dealWakeup();

    void initWakeUpFdEvent();
};
```

### 添加EPOLL事件宏
如果存在于正在监听的fd中，op为MOD，否则为ADD  
添加的是封装的FdEvent的fd与event, 尽管event中有fd和ptr但这里还是不省那点内存  
将event.fd添加到正在监听的fd中
```c++                                                      
    auto it = listen_fds_.find(event->getFd());
    int op = EPOLL_CTL_ADD;
    if (it != listen_fds_.end()) {
    op = EPOLL_CTL_MOD;
    }
    epoll_event tmp = event->getEpollEvent();
    int rt = epoll_ctl(epoll_fd_, op, event->getFd(), &tmp); 
    if (rt == -1) {
    ERRORLOG("faild epoll_ctl when add fd [%d:%s], errno=%d, errno=%s",
                event->getFd(), event->getFdName().c_str(), errno,
                strerror(errno));
    } else {
    listen_fds_.insert(event->getFd()); 
    }
    DEBUGLOG("add event success, fd [%d:%s], event [%d]", event->getFd(),
            event->getFdName().c_str(), tmp);
```
```c++
#define ADD_TO_EPOLL()  \
    auto it = listen_fds_.find(event->getFd()); \
    int op = EPOLL_CTL_ADD; \
    if(it != listen_fds_.end()){    \
        op = EPOLL_CTL_MOD; \
    }   \
    epoll_event tmp = event->getEpollEvent();   \
    int rt = epoll_ctl(epoll_fd_, op, event->getFd(), &tmp);    \
    if(rt == -1){   \
        ERRORLOG("faild epoll_ctl when add fd [%d:%s], errno=%d, errno=%s", event->getFd(), event->getFdName().c_str(), errno, strerror(errno)); \
    } else {  \
        listen_fds_.insert(event->getFd()); \
    } \
    DEBUGLOG("add event success, fd [%d:%s], event [%d]", event->getFd(), event->getFdName().c_str(), tmp); \
```
### 删除EPOLL事件宏
与上面类似
```c++
#define DEL_TO_EPOLL() \
    auto it = listen_fds_.find(event->getFd()); \
    if(it == listen_fds_.end()){ \
        return; \
    } \
    int op = EPOLL_CTL_DEL; \
    epoll_event tmp = event->getEpollEvent(); \
    INFOLOG("delete epoll event = %d", tmp); \
    int rt = epoll_ctl(epoll_fd_, op, event->getFd(), &tmp); \
    if (rt == -1) { \
            ERRORLOG("faild epoll_ctl when add fd [%d:%s], errno=%d, errno=%s", event->getFd(), event->getFdName().c_str(), errno, strerror(errno)); \
    }else { \
        listen_fds_.erase(event->getFd()); \
    } \
    DEBUGLOG("delete event success, fd [%d:%s], event [%d]", event->getFd(), event->getFdName().c_str(), tmp); \

```

### eventloop.cc
#### 初始化与析构
```c++
static thread_local EventLoop* t_cur_eventloop = NULL;
static int g_epoll_max_timeout = 10000;
static const int g_epoll_max_event = 10;

EventLoop::EventLoop(){
    if(t_cur_eventloop){
        ERRORLOG("faild to create event loop, already has a eventloop");
        exit(0);
    }
    tid_ = get_thread_id();
    
    epoll_fd_ = epoll_create(1);

    if(epoll_fd_ < 0) {
        ERRORLOG("failed to create event loop, epoll_create error, error info [%d]", errno);
        exit(0);
    }

    initWakeUpFdEvent();

    INFOLOG("success create event loop in thread [%d]", tid_);
    t_cur_eventloop = this;
}

EventLoop::~EventLoop(){
    close(epoll_fd_);
    if(wakeup_fd_event_){
        delete wakeup_fd_event_;
        wakeup_fd_event_ = NULL;
    }
}

void EventLoop::initWakeUpFdEvent(){
    wakeup_fd_ = eventfd(0, EFD_NONBLOCK);
    if(wakeup_fd_ <= 0){
        ERRORLOG("failed to create event loop, eventfd create error, error info [%d]", errno);
        exit(0);
    }

    wakeup_fd_event_ = new WakeUpFdEvent(wakeup_fd_);
    wakeup_fd_event_->listen(TriggerEvent::IN_EVENT, 
        [this]()->void{
            char buf[8];
            // -1&&EAGAIN 说明读完了
            while(read(wakeup_fd_, buf, 8) != -1 && errno != EAGAIN){

            }
            DEBUGLOG("read full bytes from wakeup fd[%d]", wakeup_fd_);
        }
    );    {}

    addEpollEvent(wakeup_fd_event_);
}
```

#### 主循环
```c++
void EventLoop::loop(){
    while(!stop_){
        ScopeMutex<Mutex> lock(mutex_);
        std::queue<std::function<void()>> tmp_tasks;
        tasks_.swap(tmp_tasks);
        lock.unlock();

        while(!tmp_tasks.empty()){
            auto cb = tmp_tasks.front();
            tmp_tasks.pop();
            if(cb) cb();
        }

        int timeout = g_epoll_max_timeout;
        epoll_event result_event[g_epoll_max_event];
        int rt = epoll_wait(epoll_fd_, result_event, g_epoll_max_event, timeout);

        if(rt < 0){
            ERRORLOG("epoll_wait error, errno=%d", errno);
            exit(0);
        }
        for(int i = 0;i < rt ;++ i){
            epoll_event trigger_event = result_event[i];
            FdEvent *fd_event = static_cast<FdEvent*>(trigger_event.data.ptr);
            if(fd_event == NULL) {
                continue;
            }

            if(trigger_event.events & EPOLLIN){
                DEBUGLOG("fd [%d:%s] trigger EPOLLIN event", fd_event->getFd(), fd_event->getFdName().c_str());
                addTask(fd_event->handler(TriggerEvent::IN_EVENT));
            }else if(trigger_event.events & EPOLLOUT){
                DEBUGLOG("fd [%d:%s] trigger EPOLLOUT event", fd_event->getFd(), fd_event->getFdName().c_str());
                addTask(fd_event->handler(TriggerEvent::OUT_EVENT));
            }

        }
    }
}
```
#### 辅助函数
``` c++
void EventLoop::wakeup(){
    wakeup_fd_event_->wakeup();
}

void EventLoop::stop(){
    stop_ = true;
}

void EventLoop::dealWakeup(){

}
```
#### 添加/删除EPOLL操作

```c++
void EventLoop::addEpollEvent(FdEvent* event){
    if(isInLoopThread()){
        ADD_TO_EPOLL();
    }else{
        auto callback = [this, event]()->void{
            ADD_TO_EPOLL();
        };    {}
        addTask(callback,true);
    }
}

void EventLoop::delEpollEvent(FdEvent* event){
    if(isInLoopThread()){
        DEL_TO_EPOLL();
    }else{
        auto callback = [this, event]()->void{
            DEL_TO_EPOLL();
        };    {}
        addTask(callback,true);
    }
}

bool EventLoop::isInLoopThread(){
    return get_thread_id() == tid_;
}

// 添加到队列中
void EventLoop::addTask(std::function<void()> callback,  bool is_wake_up /*=false*/){
    ScopeMutex<Mutex> lock(mutex_);
    tasks_.push(callback);
    lock.unlock();

    if(is_wake_up){
        wakeup();
    }
}
```
## TimerEvent
一个定时器事件的封装, 包含触发时间, 间隔, 是否取消, 是否重复。
### timer_event.h
```c++
class TimerEvent{
public:
    typedef std::shared_ptr<TimerEvent> s_ptr;

    TimerEvent(int internal, bool is_repeated, std::function<void()> callback);

public:
    void setArriveTime();

    int64_t getArriveTime() const { return arrive_time_; }

    bool isCanceled() const { return is_canceled_; }

    void setCanceled(bool c) { is_canceled_ = c; }

    bool isRepeated() const { return is_repeated_; }

    void setRepeated(bool r) { is_repeated_ = r; }

    std::function<void()> getCallBack() { return task_; }

private:
    int64_t arrive_time_;
    int64_t internal_;
    bool is_repeated_{false};
    bool is_canceled_{false};

    std::function<void()> task_;
};
```
### timer_event.cc
```c++
TimerEvent::TimerEvent(int internal, bool is_repeated, std::function<void()> callback)
        :internal_(internal), is_repeated_(is_repeated), task_(callback){
    // 
    arrive_time_ = get_now_ms() + internal_;
    DEBUGLOG("success create timer event, will execute at [%lld]", arrive_time_);
}

void TimerEvent::setArriveTime(){
    arrive_time_ = arrive_time_ + internal_;
    // int64_t now = get_now_ms();
    // if (arrive_time_ < now) {
    //     arrive_time_ = now + (internal_ - (now - arrive_time_) % internal_);
    // }
    DEBUGLOG("reset timer event, will execute at [%lld]", arrive_time_);
}
```
## Timer
定时器事件封装, 除了下面的实现我觉得也可以控制往WakeUpFd写入数据来触发epoll_wait, 性能未知。

最重要的是 `timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC)` 创建一个fd  
和 `timerfd_settime(getFd(), 0, &value, NULL)` 让内核在timespec规定的事件触发一次EPOLLIN  
所以需要listen(IN_EVENT, bind(ontimer))  
在ontimer中会顺序执行定时器记录的所有任务  
在eventloop中添加定时器后, 定时器会周期性的timerfd_settime让内核通知epoll有写入事件, 随后会调用这里的ontimer回调执行定时任务, ontimer中会再次设置timerfd_settime。
### timer.h
```c++
class Timer:public FdEvent{
public:
    Timer();
    
    ~Timer();

public:
    void addTimerEvent(TimerEvent::s_ptr event);

    void deleteTimerEvent(TimerEvent::s_ptr event);

    void onTimer(); // 发生IO事件后, eventloop会执行这个函数

private:
    std::multimap<int64_t, TimerEvent::s_ptr> events_;
    Mutex mutex_;

private:
    void resetArriveTime();
};
```
### timer.cc
```c++
Timer::Timer(): FdEvent(timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC), "TIMER"){
    
    INFOLOG("timer init,\t fd=%d", getFd());

    // 把fd的可读事件放到event上监听
    listen(FdEvent::IN_EVENT, std::bind(&Timer::onTimer, this));
}

Timer::~Timer(){

}
```
#### addTimerEvent/deleteTimerEvent
访问mutimap前加锁  

添加  
如果被添加的event的触发事件＞队首的event的触发时间, 需要让内核在新的时间通知自己。

删除  
遍历events找到要删除的事件删除
```c++
void Timer::addTimerEvent(TimerEvent::s_ptr event){
    bool is_reset_timerfd = false;

    ScopeMutex<Mutex> lock(mutex_);
    if(events_.empty()){
        is_reset_timerfd = true;
    }else{
        auto it = events_.begin();
        if((*it).second->getArriveTime() > event->getArriveTime()){
            is_reset_timerfd = true;
        }
    }
    events_.emplace(event->getArriveTime(), event);
    lock.unlock();

    if(is_reset_timerfd){
        resetArriveTime();
    }
    DEBUGLOG("success add TimerEvent at arrivetime %lld", event->getArriveTime());
}

void Timer::deleteTimerEvent(TimerEvent::s_ptr event){
    event->setCanceled(true);

    ScopeMutex<Mutex> lock(mutex_);

    auto begin = events_.lower_bound(event->getArriveTime());
    auto end = events_.upper_bound(event->getArriveTime());

    auto it = begin;
    for(it = begin; it != end ; ++ it){
        if(it->second == event){
            break;
        }
    }
    if(it != end) { 
        events_.erase(it);
    }

    lock.unlock();
    DEBUGLOG("success delete TimerEvent at arrivetime %lld", event->getArriveTime());
}
```
#### onTimer
先消费timerfd的内容。  
随后遍历events, 备份到期的事件并删除, 同时保存其回调  
遍历保存的到期任务, 如果是重复任务就重置下一次执行的事件, 再次添加到events中
遍历回调并执行  
```c++
void Timer::onTimer(){
    // 处理缓冲区
    char buf[8];
    while(1){
        if((read(getFd(), buf, 8) == -1) && errno == EAGAIN){
            break;
        }
    }

    // 执行定时任务
    int64_t now = get_now_ms();
    DEBUGLOG("ontime at %lld", now);
    std::vector<TimerEvent::s_ptr> tmps;
    std::vector<std::pair<uint64_t, std::function<void()>>> tasks;

    ScopeMutex<Mutex> lock(mutex_);
    auto it = events_.begin();

    for(it = events_.begin(); it != events_.end(); ++ it){
        // 如果到期
        if((*it).second->getArriveTime() < now ){
            if(!(*it).second->isCanceled()){
                tmps.emplace_back((*it).second);
                tasks.emplace_back(std::make_pair((*it).second->getArriveTime(), (*it).second->getCallBack()));
            }
        // 没有到期说明后面的也没到期
        } else {
            break;
        }
    }

    events_.erase(events_.begin(), it);
    lock.unlock();

    // 重新添加event
    for(auto i = tmps.begin(); i != tmps.end(); ++i){
        if((*i)->isRepeated()){
            (*i)->setArriveTime();
            addTimerEvent((*i));
        }
    }

    resetArriveTime();

    for(auto p = tasks.begin(); p != tasks.end(); ++p){
        if(p->second) p->second();
    }
}
```
#### resetArriveTime
重要

大体意思是, 如果最近的任务已经过时了, 让内核尽快通知自己去执行。
如果最近的任务还没执行, 让内核在对应的事件通知自己。
```c++
void Timer::resetArriveTime(){
    ScopeMutex<Mutex> lock(mutex_);
    auto tmp = events_;
    lock.unlock();

    if(tmp.empty()){
        return;
    }

    int64_t now = get_now_ms();
    int64_t internal;
    auto it = tmp.begin();
    // 如果第一个任务还没执行, 设置间隔为
    if(it->second->getArriveTime() > now){
        internal = it->second->getArriveTime() - now;
    } else {
        internal = 100;
    }
    DEBUGLOG("reset arrive time internal:%lld", internal);

    // s -> ms -> us -> ns
    timespec ts;
    bzero(&ts, sizeof(ts));
    ts.tv_sec = internal / 1000;
    ts.tv_nsec = (internal % 1000) * 1000000;

    itimerspec value;
    bzero(&value, sizeof(value));
    value.it_value = ts;

    // 告诉内核多久后唤醒我
    int rt = timerfd_settime(getFd(), 0, &value, NULL);
    if( rt != 0 ){
        ERRORLOG("timerfd_settime error, errno=%d, error=%s", errno, strerror(errno));
    }
}
```
# TCP  
参考muduo  
TcpBuffer  
TcpClient  
TcpServer  
TcpConnection  
# RPC  
## 协议封装  
基于Protobuf的RPC协议编码  
基于Protobuf的RPC协议解码  
## 模块封装
RpcController  
RpcClosesure  
RpcDisPatcher  
RpcChannel和RpcAsyncChannel  
# 项目完善
日志完善和优化  
代码生成工具脚手架封装  
项目构建与测试  

# 测试
## log
``` c++
#include "config.h"
#include "log.h"
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
```
## eventloop
``` c++
#include "config.h"
#include "fd_event.h"
#include "lrpc/net/eventloop.h"
#include "lrpc/common/log.h"
#include <strings.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    lrpc::EventLoop* eventloop = new lrpc::EventLoop();

    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if(listenfd == -1){
        ERRORLOG("listenfd = -1");
        exit(0);
    }

    sockaddr_in addr;
    bzero(&addr, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = INADDR_ANY;

    int rt = bind(listenfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if(rt == -1){
        ERRORLOG("bind error");
        exit(1);
    }
    
    rt = listen(listenfd, 100);
    if(rt == -1){
        ERRORLOG("listen error");
        exit(1);
    }

    lrpc::FdEvent event(listenfd, "(port:12345)");
    event.listen(lrpc::IN_EVENT, [listenfd]()->void{
        sockaddr_in peer_addr;
        socklen_t addr_len = sizeof(peer_addr);

        bzero(&peer_addr, sizeof(peer_addr));
        int clientfd = accept(listenfd, reinterpret_cast<sockaddr*>(&peer_addr), &addr_len);
        clientfd += 0;

        DEBUGLOG("success get client [%s:%d]", inet_ntoa(peer_addr.sin_addr), ntohs(peer_addr.sin_port));

    }); {}

    eventloop->addEpollEvent(&event);

    eventloop->loop();

    return 0;
}
```
# 结语
实现了一个轻量级C++ RPC框架，基于 Reactor 架构，单机可达100KQPS。项目参考了muduo 网络框架，包含代码生成工具、异步日志。通过本项目我熟悉了RPC通信原理，Reactor 架构，Linux 下后台开发知识。

学习用个人rpc框架实现

- [前置知识](#前置知识)
- [环境](#环境)
- [Config](#config)
  - [config.h](#configh)
  - [xml读取宏](#xml读取宏)
  - [config.cc](#configcc)
- [Mutex](#mutex)
  - [mutex.h](#mutexh)
  - [参考](#参考)
- [Logger](#logger)
  - [log.cc](#logcc)
  - [formatString](#formatstring)
  - [日志宏](#日志宏)
  - [log.cc](#logcc-1)
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
  - [IO 线程](#io-线程)
    - [io\_thread.h](#io_threadh)
    - [io\_thread.cpp](#io_threadcpp)
    - [io\_thread\_group.h](#io_thread_grouph)
    - [io\_thread\_group.cc](#io_thread_groupcc)
- [TCP](#tcp)
  - [TcpBuffer](#tcpbuffer)
    - [tcp\_buffer.h](#tcp_bufferh)
    - [tcp\_buffer.cc](#tcp_buffercc)
  - [TcpAcceptor](#tcpacceptor)
    - [net\_addr.h](#net_addrh)
    - [net\_addr.cc](#net_addrcc)
    - [tcp\_acceptor.h](#tcp_acceptorh)
    - [tcp\_acceptor.cc](#tcp_acceptorcc)
  - [TcpServer](#tcpserver)
    - [tcp\_server.h](#tcp_serverh)
    - [tcp\_server.cc](#tcp_servercc)
  - [TcpConnection](#tcpconnection)
    - [tcp\_connection.h](#tcp_connectionh)
    - [tcp\_connection.cc](#tcp_connectioncc)
  - [TcpClient](#tcpclient)
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
## config.h
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

## log.cc
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

## IO 线程
创建一个IO线程，他会帮我们执行：
1. 创建一个新线程
2. 在新线程里创建一个eventloop完成初始化
3. 开启loop循环

信号量 semaphore  
init  
1. sem_t *sem: 指向要初始化的信号量对象的指针
2. int pshared: 控制信号量的共享范围
   1. 0: 信号量将在当前进程的线程之间共享
   2. !0: 信号量可以在多个进程之间共享
3. unsigned int value: 信号量的初始值
   1. 1: 二进制信号量, 0/1, 互斥锁
   2. 0: 用于线程执行顺序的同步
   3. N: 计数信号量, 每当一个线程获取一个连接时，信号量减1；当连接被归还时，信号量加1。
post: +1  
wait: -1  
destory  
### io_thread.h
封装EventLoop, 初始化一个线程去执行,   
注意这里必须在start前完成AddEpollEvent, 否则只能由其他线程来AddEpollEvent（本线程会被阻塞在loop中）。
```c++
class IOThread{
public:
    IOThread();

    ~IOThread();

public:
    static void* Main(void *args);

    EventLoop* getEventloop() const { return eventloop_; }

    void start();

    void join();

private:
    pid_t tid_ {0};                // 线程号
    pthread_t thread_ {0};         // 线程句柄
    EventLoop* eventloop_ {NULL};   // io线程的loop

    sem_t init_semaphore_;
    sem_t start_semaphore_;
};
```
### io_thread.cpp
```c++
IOThread::IOThread(){
    int rt = sem_init(&init_semaphore_, 0, 0);
    assert(rt == 0);
    rt = sem_init(&start_semaphore_, 0, 0);
    assert(rt == 0);
    pthread_create(&thread_, NULL, &IOThread::Main, this);

    // 需要等到Main函数的eventloop循环启动才返回
    sem_wait(&init_semaphore_);

    DEBUGLOG("IOThread [%d] create success", tid_);
}

IOThread::~IOThread(){
    if(eventloop_) eventloop_->stop();
    sem_destroy(&init_semaphore_);
    pthread_join(thread_, NULL);

    if(eventloop_){
        delete eventloop_;
        eventloop_ = NULL;
    }
}

void* IOThread::Main(void *args){
    IOThread* thread = static_cast<IOThread*>(args);

    thread->eventloop_ = new EventLoop();
    thread->tid_ = get_thread_id();

    sem_post(&thread->init_semaphore_);

    DEBUGLOG("IOThread [%d] wait start", thread->tid_);
    sem_wait(&thread->start_semaphore_);

    DEBUGLOG("IOThread [%d] start eventloop", thread->tid_);
    thread->eventloop_->loop();
    DEBUGLOG("IOThread [%d] end eventloop", thread->tid_);
    return nullptr;
}

void IOThread::start(){
    sem_post(&start_semaphore_);
}

void IOThread::join(){
    pthread_join(thread_, NULL);
}
```
### io_thread_group.h
一个池
```c++
class IOThreadGroup{
public:
    IOThreadGroup(int size);
    
    ~IOThreadGroup();

public: 
    void start();

    void join();

    IOThread* getIOThread();

public:

private:
    int size_;
    int index_;
    std::vector<IOThread*> io_thread_groups_;
};
```

### io_thread_group.cc
```c++
IOThreadGroup::IOThreadGroup(int size): size_(size){
    io_thread_groups_.resize(size);
    for(int i =0;i < size; ++i){
        io_thread_groups_[i] = new IOThread();
    }
}
    
IOThreadGroup::~IOThreadGroup(){

}

void IOThreadGroup::start(){
    for(int i = 0;i < size_; ++ i){
        io_thread_groups_[i]->start();
    }
}

void IOThreadGroup::join(){
    for(int i = 0;i < size_; ++ i){
        io_thread_groups_[i]->join();
    }
}

IOThread* IOThreadGroup::getIOThread(){
    if(index_ >= io_thread_groups_.size() || index_ == -1){
        index_ = 0;
    }
    return io_thread_groups_[index_++];
}
```

信号量 semaphore, sem_init, sem_wait, 等待信号量+1返回。

# TCP  
参考muduo  
## TcpBuffer  
负责把发送的数据写入Buffer然后从Buffer写入到发送缓冲区,  
或者把接收缓冲区里的数据读到Buffer中。

为什么需要应用层Buffer
- 方便数据处理, 堆应用层的包进行组装、拆解。Tcp粘包：对Tcp没有包的概念, 传输的是二进制字节流。
包是应用层的, 可能Tcp接收到的数据并不是完整的应用层的包, 而是一半。这个时候就要先写入Buffer。
- 方便异步发送, 发送数据到缓冲区里, eventloop自己从缓冲区里拿数据发送。不需要同步等待。
- 提高一个发送效率，可以多包合并发送。

循环可扩容数组
### tcp_buffer.h
```c++
class TcpBuffer{
public:
    TcpBuffer(size_t size);

    ~TcpBuffer();

public:
    size_t readable();
    
    size_t writeable();

    size_t getReadIdx() { return read_idx_; }

    size_t getWriteIdx() { return write_idx_; }

    void resizeBuffer(size_t new_size);

    void writeToBuf(const char* data, size_t len);

    void readFromBuf(std::vector<char>& re, size_t size);

    void moveReadIndex(size_t len);

    void moveWriteIndex(size_t len);
private:
    size_t read_idx_{0};
    size_t write_idx_{0};
    size_t size_{0};
    bool full_{false};

    std::vector<char> buffer_;
};
```
### tcp_buffer.cc
readindex/writeindex  
size = (writeindex-readindex+buffersize)%buffersize

```c++
TcpBuffer::TcpBuffer(size_t size):size_(size){
    buffer_.resize(size_);
}

TcpBuffer::~TcpBuffer(){

}

size_t TcpBuffer::readable(){
if (full_) return size_;
    if (write_idx_ >= read_idx_) {
        return write_idx_ - read_idx_;
    }
    return size_ - read_idx_ + write_idx_;
}

size_t TcpBuffer::writeable(){
    return size_ - readable();
}

void TcpBuffer::resizeBuffer(size_t new_size){
    std::vector<char> new_buf(new_size);
    size_t r = readable();
    
    // 手动复制数据，不调用 readFromBuf
    if (r > 0) {
        size_t end_data = size_ - read_idx_;
        if (r <= end_data) {
            memcpy(&new_buf[0], &buffer_[read_idx_], r);
        } else {
            memcpy(&new_buf[0], &buffer_[read_idx_], end_data);
            memcpy(&new_buf[end_data], &buffer_[0], r - end_data);
        }
    }
    
    buffer_.swap(new_buf);
    size_ = new_size;
    read_idx_ = 0;
    write_idx_ = r;
    full_ = (r == size_);
}

void TcpBuffer::writeToBuf(const char* data, size_t len){
    if(len > writeable()){
        // 扩容
        resizeBuffer(len + size_);
    }
    size_t end_space = size_ - write_idx_;
    if(len <= end_space){
        memcpy(&buffer_[write_idx_], data, len);
        write_idx_ = (write_idx_ + len) % size_;
    } else {
        memcpy(&buffer_[write_idx_], data, end_space);
        memcpy(&buffer_[0], data + end_space, len - end_space);
        write_idx_ = len - end_space;
    }

    if (write_idx_ == read_idx_) full_ = true;
}

void TcpBuffer::readFromBuf(std::vector<char>& re, size_t len){
    size_t avail = readable();
    if (avail == 0) return;
    len = std::min(len, avail);

    std::vector<char> tmp(len);
    size_t end_data = size_ - read_idx_;
    if (len <= end_data) {
        memcpy(&tmp[0], &buffer_[read_idx_], len);
        read_idx_ = (read_idx_ + len) % size_;
    } else {
        memcpy(&tmp[0], &buffer_[read_idx_], end_data);
        memcpy(&tmp[end_data], &buffer_[0], len - end_data);
        read_idx_ = len - end_data;
    }

    if (full_) full_ = false;
    re.swap(tmp);
}


void TcpBuffer::moveWriteIndex(size_t len){
    if (len == 0) return;
    
    size_t avail = writeable();
    if (len > avail) {
        len = avail;  // 限制移动长度不超过可写空间
    }
    
    write_idx_ = (write_idx_ + len) % size_;
    
    // 更新 full_ 标志
    if (write_idx_ == read_idx_) {
        full_ = true;
    }
}

void TcpBuffer::moveReadIndex(size_t len){
    if (len == 0) return;
    
    size_t avail = readable();
    if (len > avail) {
        len = avail;  // 限制移动长度不超过可读数据
    }
    
    read_idx_ = (read_idx_ + len) % size_;
    
    // 更新 full_ 标志
    if (read_idx_ != write_idx_) {
        full_ = false;
    }
}   
```
## TcpAcceptor
socket-bind-listen-accept   
SO_REUSEADDR 监听一个套接字，然后服务器关闭，重启这个服务，如果是同一个端口可能会报bind错误，addr已经被绑定。  
因为tcp在主动关闭连接的一方，套接字会变成timewait状态，处于timewait状态会持续占用端口，如果新程序在这个端口启动，bind就会出错。  
该选项可以重新绑定这个端口。  

tcpacceptor负责 1. listen 2. acceptor 3. addepollevent到eventloop
具体执行由eventloop负责
### net_addr.h
对 sockaddr 的封装
```c++
class NetAddr{

public:
    typedef std::shared_ptr<NetAddr> s_ptr;

    virtual sockaddr* getSockAddr() = 0;

    virtual socklen_t getSockLen() = 0;

    virtual int getFamily() = 0;

    virtual std::string toString() = 0;

    virtual bool checkValid() = 0;
};

class IPNetAddr: public NetAddr{

public:
    IPNetAddr(const std::string& ip, uint16_t port);

    IPNetAddr(const std::string& addr);

    IPNetAddr(sockaddr_in addr);

    sockaddr* getSockAddr();

    socklen_t getSockLen();

    int getFamily();

    std::string toString();

    bool checkValid();

private:
    std::string  ip_;
    uint16_t port_;
    sockaddr_in addr_;
};
```
### net_addr.cc
```c++
IPNetAddr::IPNetAddr(const std::string& ip, uint16_t port):ip_(ip), port_(port){
    memset(&addr_, 0, sizeof(addr_));

    addr_.sin_family = AF_INET;
    // inet_addr() 点分十进制IP地址 转换为网络字节序
    addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
    // 主机序转换为网络字节序
    addr_.sin_port = htons(port_);
}

IPNetAddr::IPNetAddr(const std::string& addr){
    size_t i = addr.find_first_of(":");
    if(i == addr.size()){
        ERRORLOG("invalid ipv4 addr %s", addr.c_str());
        return ;
    }
    ip_ = addr.substr(0, i);
    port_ = std::atoi(addr.substr(i+1, addr.size()-i-1).c_str());

    
    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
    addr_.sin_port = htons(port_);
}

IPNetAddr::IPNetAddr(sockaddr_in addr):addr_(addr){
    ip_ = inet_ntoa(addr_.sin_addr);
    port_ = ntohs(addr.sin_port);
}

sockaddr* IPNetAddr::getSockAddr(){
    return reinterpret_cast<sockaddr*>(&addr_);
}

socklen_t IPNetAddr::getSockLen(){
    return sizeof(addr_);
}

int IPNetAddr::getFamily(){
    return AF_INET;
}

std::string IPNetAddr::toString(){
    return ip_ + ":" + std::to_string(port_);
}

bool IPNetAddr::checkValid(){
    if(ip_.empty()) {
        return false;
    }
    if(port_ <= 0 || port_ > 65535){
        return false;
    }
    if(inet_addr(ip_.c_str()) == INADDR_NONE) {
        return false;
    }
    return true;
}
```
### tcp_acceptor.h
```c++
class TcpAcceptor {

public:
    TcpAcceptor(NetAddr::s_ptr local_addr);

    ~TcpAcceptor();

public:
    int accept();

private:
    NetAddr::s_ptr local_addr_; // 服务端监听的地址 addr -> ip:port
    int family_{-1};            // 地址协议族
    int listenfd_{-1};           // listenfd
    
};
```
### tcp_acceptor.cc
```c++
TcpAcceptor::TcpAcceptor(NetAddr::s_ptr local_addr):local_addr_(local_addr){
    if(!local_addr_->checkValid()){
        ERRORLOG("invalid local addr %s", local_addr_->toString().c_str());
        exit(0);
    }
    family_ = local_addr_->getFamily();
    listenfd_ = socket(family_, SOCK_STREAM, 0);
    if(listenfd_ == 0){
        ERRORLOG("invalid listenfd %d", listenfd_);
        exit(0);
    }

    int val = 1;
    // SO_REUSEADDR 监听一个套接字，然后服务器关闭，重启这个服务，如果是同一个端口可能会报bind错误，addr已经被绑定。
    // 因为tcp在主动关闭连接的一方，套接字会变成timewait状态，处于timewait状态会持续占用端口，如果新程序在这个端口启动，bind就会出错。
    // 该选项可以重新绑定这个端口。
    if(setsockopt(listenfd_, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0){
        ERRORLOG("setsockopt REUSEADDR error, errno=%d, error=%s", errno, strerror(errno));
    }

    socklen_t len = local_addr_->getSockLen();
    int rt = bind(listenfd_, local_addr->getSockAddr(), len);
    if(rt != 0){
        ERRORLOG("bind error, errno=%d, error=%s", errno, strerror(errno));
        exit(0);
    }

    rt = listen(listenfd_, 1000);
    if(rt != 0){
        ERRORLOG("listen error, errno=%d, error=%s", errno, strerror(errno));
        exit(0);
    }
}

TcpAcceptor::~TcpAcceptor(){

}

int TcpAcceptor::accept(){
    if(family_ == AF_INET){
        sockaddr_in client_addr;
        bzero(&client_addr,sizeof(client_addr));
        socklen_t client_addr_len = sizeof(client_addr);

        int client_fd = ::accept(listenfd_, reinterpret_cast<sockaddr*>(&client_addr), &client_addr_len);
        if(client_fd < 0){
            ERRORLOG("accept error, errno=%d, error=%s", errno, strerror(errno));
        }

        IPNetAddr addr(client_addr);
        INFOLOG("A client have accept succ, peer addr [%s]", addr.toString().c_str());
        return client_fd;
        
    }
    return -1;
}
```

## TcpServer  
![主从Reactor](img/主从.png)

工作流程： A(主循环) B(工作线程)
1. [线程A]创建TcpServer
   1. 创建A
      1. 向线程自己添加wakeup、timer
   2. 创建线程组
      1. 创建B
         1. 向线程自己添加wakeup、timer
2. [线程A]监听listenfd的INEVENT, 回调onAcceptor:
      1. 创建一个connection:
      2. 初始化buffer
      3. 创建fdevent
      4. 添加到[线程B]的监听中, 由于是异线程, 会添加到任务队列中让线程自己添加
3. [线程A]启动
4. [线程A]触发新连接, 调用2中的步骤
   1. 把监听任务添加到[线程B]的任务队列并且唤醒
5. [线程B]被唤醒
   1. 执行监听任务监听连接的INEVENT
6. [线程B]触发INEVENT
   1. read
   2. execute
   3. write
7. [线程B]对端关闭 clear
8. [线程B]主动关闭 shutdown

主线程通过一个EventLoop循环的从Client接收连接请求并且注册给IO线程  
只注册Acceptor，绑定的处理方法是将连接对应的处理函数构造成EPOLL事件在IO线程注册。

IO线程则监听这些事件，等待EPOLL触发。

### tcp_server.h
```c++
class TcpServer{

public:
    TcpServer(NetAddr::s_ptr local_addr);

    ~TcpServer();

public:
    void start();

private:
    void init();
    
    // 有新客户端连接需要执行
    void onAccept();

private:
    TcpAcceptor::s_ptr acceptor_;
    NetAddr::s_ptr local_addr_;             // 本地监听的地址
    FdEvent *listen_fd_event_{0};
    
    EventLoop* main_event_loop_{NULL};      // main reactor
    IOThreadGroup* io_thread_group_{NULL};  // subReactor 组
    
    int client_counts_{0};
};
```
### tcp_server.cc
```c++
TcpServer::TcpServer(NetAddr::s_ptr local_addr):local_addr_(local_addr){
    INFOLOG("[TcpServer] Start create");
    init();
    INFOLOG("[TcpServer] Success create on [%s]", local_addr->toString().c_str());
}

TcpServer::~TcpServer(){
    if(main_event_loop_){
        delete main_event_loop_;
        main_event_loop_ = NULL;
    }
    if(io_thread_group_){
        delete io_thread_group_;
        io_thread_group_ = NULL;
    }
}

void TcpServer::start(){
    DEBUGLOG("tcpServer start - io_thread_group");
    io_thread_group_->start();
    main_event_loop_->loop();
}

void TcpServer::init(){
    acceptor_ = std::make_shared<TcpAcceptor>(local_addr_);

    DEBUGLOG("---------- tcpServer mian_event_loop create ----------");
    main_event_loop_ = EventLoop::GetCurEventLoop();

    DEBUGLOG("---------- tcpServer io_thread_group create ----------");
    io_thread_group_ = new IOThreadGroup(2);

    listen_fd_event_ = new FdEvent(acceptor_->getListenFd(), "监听主循环事件");
    listen_fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpServer::onAccept, this));

    main_event_loop_->addEpollEvent(listen_fd_event_);
    DEBUGLOG("----------        tcpServer init over        ----------");
}

// 有一个新连接来了， maineventloop处理，添加到 iothreadgroup中
void TcpServer::onAccept(){
    int client_fd = acceptor_->accept();
    ++client_counts_;

    IOThread* io_thread = io_thread_group_->getIOThread();
    
    TcpConnection::s_ptr connection = std::make_shared<TcpConnection>(io_thread, client_fd, 128, peer_addr);
    connection->setState(TcpConnection::Connected);
    client_.insert(connection);
    

    INFOLOG("[TcpServer] Success get client fd=%d", client_fd);
}
```
## TcpConnection  
![](img/TcpConnection.png)
对客户端和服务端有不同的逻辑

对客户端，对端为服务端。  
- 初始化时
    1. 必须等待connect回包(fd 可写)连接才算成功，不需要设置listenRead(onRead)
- onRead 时
    1. 从 socket 缓冲区调用 read 读取字节流到 inbuffer 里
    2. 从 inbuffer 中 decode 得到 message
    3. message 的 req_id 符合，则执行其注册时候的回调
- onWrite时
    1. 把 client push 的 messages encode到 outbuffer
    2. 从 outbuffer 调用 write 写字节流到 socket 缓冲区
    3. 如果全部发送完成，取消listenRead
    4. 执行回调
- tcp_client调用writeMessage时
    1. connection.pushSendMessage
    2. connection.listenWrite
- tcp_server调用readReadMessage时
    1. connection.pushMessage
    2. connection.listenRead


对服务端，对端为客户端
- 初始化时
    1. accpet成功则连接建立，需要立刻listenRead(onRead)
- onRead时
    1. 从 socket 缓冲区调用 read 读取字节流到 inbuffer 里
    2. execute 协议解析
    3. 结果写入outbuffer 并且 listenWrite
- onWrite时
    1. 从 outbuffer 调用 write 写字节流到 socket 缓冲区
    2. 如果全部发送完成，取消listenRead

全过程
1. client：调用writeMessage，listenWrite 并记录消息(req_id+msg)、回调
2. client：发现可写，触发onWrite， encode message 到 outbuffer 然后发送到 socke，发送完毕取消listenWrite, 执行回调
3. server：发现可读，触发OnRead，从socket读取到inbuffer，协议解析执行，写回 outbuffer，listenWrite
4. server：发现可写，触发OnWrite, 将 outbuffer 发送到socket
5. client：调用readMessage，listenRead 并且记录req_id、回调
6. client：发现可读，除法OnRead，从socket读取到inbuffer，decode 得到 message，执行回调（操作message）
    

### tcp_connection.h
```c++
class TcpConnection{
public:
    enum TcpState{
        NotConnected = 1,
        Connected = 2,
        HalfConnected = 3,
        Closed = 4,
    };

    enum TcpConnectionType{
        ClientConnectionByServer = 1,   // 服务端使用，代表对端为客户端连接
        ServerConnectionByClient = 2,   // 客户端使用，代表对端为服务端连接
    };

public:
    typedef std::shared_ptr<TcpConnection> s_ptr;

    // TcpConnection(IOThread* io_thread, int fd, int buffer_size, NetAddr::s_ptr peer_addr);
    TcpConnection(EventLoop* event_loop, int fd, int buffer_size, NetAddr::s_ptr peer_addr, TcpConnectionType type);

    ~TcpConnection();


public:
    void onRead();

    void execute();
    
    void onWrite();

    void setState(const TcpState s) { state_ = s; }

    TcpState getState() { return state_ ; }

    void clear();

    void shutdown();

    void setConnectionType(const TcpConnectionType type) { connection_type_ = type; }

    void listenRead();

    void listenWrite();

    void pushSendMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback);

    void pushReadMessage(std::string req_id, std::function<void(AbstractProtocol::s_ptr)> callback);

private:
    // 通信的两个对端
    NetAddr::s_ptr local_addr_;
    NetAddr::s_ptr peer_addr_;

    // 输入输出缓冲区
    TcpBuffer::s_ptr in_buffer;
    TcpBuffer::s_ptr out_buffer;

    // 持有该连接的IO线程IO， 也就是本对象属于哪个IO线程
    // IOThread* io_thread_{NULL};
    EventLoop* event_loop_{NULL};
    
    // 一个连接只用一个 FdEvent监听
    FdEvent* fd_event_{NULL};
    int fd_{-1};

    TcpState state_;

    TcpConnectionType connection_type_{TcpConnectionType::ServerConnectionByClient};

    // key = message->req_id_ value = callback
    std::vector<
        std::pair<AbstractProtocol::s_ptr, std::function<void(AbstractProtocol::s_ptr)>>
    > write_callbacks_;

    std::map<
        std::string, std::function<void(AbstractProtocol::s_ptr)>
    > read_callbacks_;

    AbstractCoder* coder_{NULL};
};
```

### tcp_connection.cc
```c++
TcpConnection::TcpConnection(EventLoop* event_loop, int fd, int buffer_size, NetAddr::s_ptr peer_addr, TcpConnectionType type)
    : peer_addr_(peer_addr), event_loop_(event_loop), fd_(fd), state_(NotConnected), connection_type_(type) {
    in_buffer = std::make_shared<TcpBuffer>(buffer_size);
    out_buffer = std::make_shared<TcpBuffer>(buffer_size);

    fd_event_ = FdEventGroup::GetFdEventGroup()->getFdEvent(fd);
    fd_event_->setNonBlock();

    // 服务端，对端为客户端，需要在初始化监听可读事件
    // server中onAccept会创建Connection，连接完成
    if(connection_type_ == ClientConnectionByServer){
        listenRead();
    }
    // 客户端，对端为服务端，不需要一直监听可读事件
    // 他发送connect后必须等待回包才说明连接成功
    // 可读的OnRead可以执行的前提是连接已经建立

    coder_ = new StringCoder();
}

TcpConnection::~TcpConnection(){

}


void TcpConnection::onRead(){
    // 1. 从 socket 缓冲区调用 read 读取字节流到 inbuffer 里

    if(state_ != Connected){
        ERRORLOG("onRead error client has already disconnected, addr[%s], clientfd[%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    bool is_read_all = false;
    bool is_closed = false;
    while(!is_read_all){
        
        if(in_buffer->writeable() == 0){
            in_buffer->resizeBuffer(2 * in_buffer->buffer_.size());
        }

        int read_size = in_buffer->writeable();
        int write_index = in_buffer->getWriteIdx();

        int rt = read(fd_, &in_buffer->buffer_[write_index], read_size);
        DEBUGLOG("success read %d byte from addr[%s], clientfd[%d]", rt, peer_addr_->toString().c_str(), fd_);
        if(rt > 0){
            in_buffer->moveWriteIndex(rt);
            // 如果本次读取数据和writeable相同， 可能还有没读的， 继续读
            if(rt == read_size){
                continue;
            } else {
                is_read_all = true;
                break;
            }
        }else if(rt == 0){
            is_closed = true; 
            break;
        // 非阻塞读到没有可读会报错 EAGAIN
        }else if(rt == -1 && errno == EAGAIN){
            is_read_all = true;
            break;
        }
    }

    if(is_closed){
        // TODO: 处理关闭连接
        clear();
        DEBUGLOG("peer closed, peer addr [%d], clientfd [%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    if(!is_read_all){
        ERRORLOG("not read all data");
    }

    // TODO: RPC解析协议
    
    // 2. 协议解析
    execute();
}

void TcpConnection::execute(){
    // 服务端逻辑，对端是客户端
    if(connection_type_ == ClientConnectionByServer){
        DEBUGLOG("server execute");
        // 将RPC请求执行业务逻辑，获取RPC响应，发送回去
        std::vector<char> tmp;
        int size = in_buffer->readable();
        tmp.resize(size);

        in_buffer->readFromBuf(tmp, size);

        std::string msg;
        for(int i = 0;i < tmp.size(); ++ i){
            msg += tmp[i];
        }
        INFOLOG("success get request[%s] fomr client[%s]", tmp.data(), peer_addr_->toString().c_str());

        // echo
        out_buffer->writeToBuf(tmp.data(), tmp.size());

        listenWrite();
    }
    // 客户端逻辑，对端是服务端
    else {
        DEBUGLOG("client execute");
        // 从 buffer 里decode 得到 message 对象，执行其回调
        std::vector<AbstractProtocol::s_ptr> out_messages;
        coder_->decode(out_messages, in_buffer);

        for(size_t i = 0; i < out_messages.size(); ++ i){
            std::string req_id = out_messages[i]->req_id_;
            auto it = read_callbacks_.find(req_id);
            if( it != read_callbacks_.end()){
                it->second(out_messages[i]->shared_from_this());
            }
        }
    }
}

void TcpConnection::onWrite(){
    if(state_ != Connected){
        ERRORLOG("onWrite error client has already disconnected, addr[%s], clientfd[%d]", peer_addr_->toString().c_str(), fd_);
        return ;
    }

    // 对端是服务端
    // 1. 编码message到outbuffer
    if (connection_type_ == ServerConnectionByClient){
        // 将数据写入到 buffer 
        // 1. 将 message encoder
        // 2. 将 字节流写入到 buffer，全部发送

        std::vector<AbstractProtocol::s_ptr> messages;
        for(size_t i = 0;i < write_callbacks_.size(); ++ i){
            messages.push_back(write_callbacks_[i].first);
        }
        coder_->encode(messages, out_buffer);
    }

    // 2. 从 outbuffer 调用 write 写字节流到 socket 缓冲区
    bool is_write_all = false;
    while(true){
        if(out_buffer->readable() == 0){
            DEBUGLOG("no data need to send to client [%s]", peer_addr_->toString().c_str());
            is_write_all = true;
            break;
        }
        int write_size = out_buffer->readable();
        int read_index = out_buffer->getReadIdx();
        int rt = write(fd_, &out_buffer->buffer_[read_index], write_size);

        if(rt >= write_size){
            DEBUGLOG("send all data to client [%s]", peer_addr_->toString().c_str());
            is_write_all = true;
            break;
        }
        if(rt == -1 && errno == EAGAIN){
            // 缓冲区满了不能发送
            // 等待下次 fd 可写在发送
            DEBUGLOG("write data error, errno=EAGAIN, rt = -1");
            break;
        }
    }
    // 3. 如果全部发送完成，取消可写事件监听
    if(is_write_all){
        fd_event_->cancel(FdEvent::OUT_EVENT);
        event_loop_->addEpollEvent(fd_event_);
    }

    // 4.如果是对端是服务端，执行回调函数
    if(connection_type_ == ServerConnectionByClient){
        for(size_t i = 0;i < write_callbacks_.size(); ++ i){
            write_callbacks_[i].second(write_callbacks_[i].first);
        }
    }
    write_callbacks_.clear(); 
}

void TcpConnection::clear(){
    // 处理关闭连接后的清理动作
    if(state_ == Closed){
        return;
    }


    event_loop_->delEpollEvent(fd_event_);
    fd_event_->cancel(FdEvent::IN_EVENT);
    fd_event_->cancel(FdEvent::OUT_EVENT);

    state_ = Closed;

}

void TcpConnection::shutdown(){
    if(state_ == Closed || state_ == NotConnected){
        return;
    }

    // 处于半关闭
    state_ = HalfConnected;

    // 关闭读写， 服务器不会再对这个fd进行读写操作了
    // 发送FIN报文，触发四次挥手第一阶段
    // 当FD发生可读事件但是可读数据为0，即对端也发送了FIN报文
    // 服务器就会进入timewait
    ::shutdown(fd_, SHUT_RDWR);
}

void TcpConnection::listenRead(){
    fd_event_->listen(FdEvent::IN_EVENT, std::bind(&TcpConnection::onRead, this));
    event_loop_->addEpollEvent(fd_event_);
}

void TcpConnection::listenWrite(){
    fd_event_->listen(FdEvent::OUT_EVENT, std::bind(&TcpConnection::onWrite, this));
    event_loop_->addEpollEvent(fd_event_);
}

void TcpConnection::pushSendMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback){
    write_callbacks_.push_back(std::make_pair(message, callback));
}

void TcpConnection::pushReadMessage(std::string req_id, std::function<void(AbstractProtocol::s_ptr)> callback){
    read_callbacks_.insert(std::make_pair(req_id, callback));
}
```

## TcpClient  
- Connect: 连接对端
- Write:    将RPC请求encode然后发送给服务端
- Read:     读取服务端发来的数据decode成RPC请求
非阻塞Connect:
- 0 成功
- -1 errno=EINPROGRESS 表示连接正在建立，此时可以添加到epoll去监听其可写事件。等待可写就绪，调用getsockopt获取fd上的错误，0代表连接成功。
- 其他errno

当ReadMessage的时候，会监听可读事件，可读发生才会执行回调，WriteMessage会监听可写事件。

### tcp_client.cc
```c++
TcpClient::TcpClient(NetAddr::s_ptr peer_addr):peer_addr_(peer_addr){
    event_loop_ = EventLoop::GetCurEventLoop();
    fd_ = socket(peer_addr->getFamily(), SOCK_STREAM, 0);

    if(fd_ < 0){
        ERRORLOG("create socket error");
        return ;
    }

    fd_event_ = FdEventGroup::GetFdEventGroup()->getFdEvent(fd_);
    fd_event_->setNonBlock();

    connection_ = std::make_shared<TcpConnection>(event_loop_, fd_, 128, peer_addr_, TcpConnection::ServerConnectionByClient);

}

TcpClient::~TcpClient(){
    if(fd_ > 0){
        close(fd_);
    }
}

void TcpClient::connect(std::function<void()> callback){
    int rt = ::connect(fd_, peer_addr_->getSockAddr(), peer_addr_->getSockLen());
    if(rt == 0){
        DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
    }else if(rt == -1){
        if(errno == EINPROGRESS){
            // epoll 监听可写
            DEBUGLOG("connect in progress");

            fd_event_->listen(FdEvent::OUT_EVENT, [this, callback]()->void{
                {}
                int err = 0;
                socklen_t len = sizeof(err);
                getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len);
                bool is_connected = false;
                if(err == 0){
                    DEBUGLOG("connect [%s] success", peer_addr_->toString().c_str());
                    connection_->setState(TcpConnection::Connected);
                    is_connected = true; 
                } else {
                    ERRORLOG("connect error, errno=%d, error=%s", err, strerror(errno));
                }
                // 连接完成去掉可写事件监听，否则会一直除法
                DEBUGLOG("去掉可写事件监听");
                fd_event_->cancel(FdEvent::OUT_EVENT);
                event_loop_->addEpollEvent(fd_event_);

                // 连接成功才执行回调
                if(is_connected && callback) callback();
            });
            
            event_loop_->addEpollEvent(fd_event_);
            if(!event_loop_->isLooping()) {
                event_loop_->loop();
            }

        } else {
            ERRORLOG("connect error, errno=%d, error=%s", errno, strerror(errno));
            return ;
        }
    }
}

void TcpClient::writeMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback){
    //1. 把message编码写入到connection的buffer中, done 也要写入。
    //2. 启动connection可写事件。
    connection_->pushSendMessage(message, callback);
    connection_->listenWrite();
}

void TcpClient::readMessage(const std::string &req_id, std::function<void(AbstractProtocol::s_ptr)> callback){
    //1. 启动connection可读事件
    //2. 从buffer中解码读出message, 判断 req_id 是否相等，相等则读成功，执行其回调。
    connection_->pushReadMessage(req_id, callback);
    connection_->listenRead();
}
```

### tcp_client.h
```c++
class TcpClient {

public: 
    TcpClient(NetAddr::s_ptr peer_addr);

    ~TcpClient();

public:
    // 异步connect
    // 如果调用connect成功，callback会执行
    void connect(std::function<void()> callback);

    // 异步发送 message
    // 发送成功会调用callback，callback入参是message对象
    void writeMessage(AbstractProtocol::s_ptr message, std::function<void(AbstractProtocol::s_ptr)> callback);

    // 异步接收 message
    // 接收成功会调用callback， callback入参是message对象
    void readMessage(const std::string &req_id, std::function<void(AbstractProtocol::s_ptr)> callback);


private:
    NetAddr::s_ptr peer_addr_;
    EventLoop* event_loop_{NULL};

    int fd_{-1};
    FdEvent* fd_event_{NULL};
    TcpConnection::s_ptr connection_{NULL};
};
```

# RPC  
为什么要定义一个RPC协议，既然做了ProtoBuf序列化，为什么不把序列化结果直接发送？
- 为了方便分割两个请求，protobuf序列化结果是一串无意义的字节流，无法区分哪里是开始哪里是结束。
- 为了定位：加上MsgId等信息，帮助匹配一次的RPC请求和响应，不会串包
- 错误提升：加上错误信息，定位RPC失败的原因
## 协议封装  
### Tiny ProtoBuf Protocol
大端存储/小端存储  
由于不同电脑对int类型解读的方式不同，所以默认网络上的是大端序，在本地再转换成本地字节序(htonl/htons)  
大端是高位存储在低地址
```scss
[Start 0x02]
[包长度]                            // 包含开始和结束
[MsgID长度][MsgId]                  // 标识唯一RPC请求
[Func长度][Func]                    // RPC方法完整名
[ErrMsgCode][ErrMsgLength][ErrMsg]  // 错误信息
[ProtoBufData]                      // 序列化数据
[CheckSum]                          // 校验和
[End 0x03]
```

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
## tcp_server
```c++
#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_server.h"
#include <memory>
#include <sys/socket.h>

void test_tcpServer(){
    lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    
    lrpc::TcpServer tcp_server(addr);
    tcp_server.start();
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    test_tcpServer();
    return 0;
}
```

## tcp_client
```c++
#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/coder/string_coder.h"
#include "lrpc/net/tcp/tcp_client.h"
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

void test_connect(){
    // 调用connect连接server
    // write 一个字符串
    // 等待 read 返回结果

    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if(fd < 0){
        ERRORLOG("invalid fd %d", fd);
        exit(0);
    }

    sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(12345);
    inet_aton("127.0.0.1", &server_addr.sin_addr); 

    int rt = connect(fd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr));

    std::string msg = "hello";

    rt = write(fd, msg.c_str(), msg.length());

    DEBUGLOG("success write %d bytes, [%s]",rt, msg.c_str());


    char buf[100];
    rt = read(fd, buf, 100);
    DEBUGLOG("success read %d bytes, [%s]", rt , std::string(buf).c_str());
}

void test_client(){
    lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    lrpc::TcpClient client(addr);
    

    
    client.connect([addr, &client]()->void{
        {}
        DEBUGLOG("{tcp_client connect 回调} connect to [%s] success", addr->toString().c_str());
        
        std::shared_ptr<lrpc::StringProtocol> message = std::make_shared<lrpc::StringProtocol>("12345", "hello lrpc");
        
        client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            DEBUGLOG("client write message success, req_id=[%s]", msg_ptr->req_id_.c_str());
        });

        client.readMessage("12345", [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            std::shared_ptr<lrpc::StringProtocol> message = std::dynamic_pointer_cast<lrpc::StringProtocol>(msg_ptr);
            DEBUGLOG("client read message success, req_id=[%s], msg=[%s]", 
                message->req_id_.c_str(), message->msg_.c_str()
            );
        });

        client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            DEBUGLOG("client write message success, req_id=[%s]", msg_ptr->req_id_.c_str());
        });
    });
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    // test_connect();
    test_client();

    return 0;
}
```

# 结语
实现了一个轻量级C++ RPC框架，基于 Reactor 架构，单机可达100KQPS。项目参考了muduo 网络框架，包含代码生成工具、异步日志。通过本项目我熟悉了RPC通信原理，Reactor 架构，Linux 下后台开发知识。
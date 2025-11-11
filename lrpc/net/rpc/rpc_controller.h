#pragma once

#include "lrpc/net/tcp/net_addr.h"
#include <cstdint>
#include <google/protobuf/service.h>
#include <google/protobuf/stubs/callback.h>
namespace lrpc{

class RpcController : public google::protobuf::RpcController{
public:
    RpcController(){}
    ~RpcController(){}

    // 客户端调用 清除RPC初始状态
    void Reset();

    // 当前RPC调用是否成功
    bool Failed() const;

    std::string ErrorText() const;

    void StartCancel();

    // 设置失败的原因
    void SetFailed(const std::string& reason);

    bool IsCanceled() const;

    void NotifyOnCancel(google::protobuf::Closure* callback);

    
    
    void setError(int32_t err_code, const std::string err_info);

    int32_t getErrorCode();

    std::string getErrorInfo();

    void setMsgId(const std::string& msg_id);

    std::string getMsgId();

    void setLocalAddr(NetAddr::s_ptr addr);

    void setPeerAddr(NetAddr::s_ptr addr);

    NetAddr::s_ptr getLocalAddr();

    NetAddr::s_ptr getPeerAddr();

    void setTimeOut(int timeout);

    int getTimeOut();

private:
    int error_code_{0};
    std::string error_info_;
    std::string msg_id_;

    bool is_failed_{false};
    bool is_canceled_{false};

    NetAddr::s_ptr local_addr_;
    NetAddr::s_ptr peer_addr_;

    int timeout_{1000};         //ms
};

}   // namespace lrpc
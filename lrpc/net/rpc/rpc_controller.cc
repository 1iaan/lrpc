#include "lrpc/net/rpc/rpc_controller.h"

namespace lrpc{

// 客户端调用 清除RPC初始状态
void RpcController::Reset(){
    error_code_ = 0;
    error_info_ = "";
    req_id_ = "";

    is_failed_ = false;
    is_canceled_ = false;

    local_addr_ = nullptr;
    peer_addr_ = nullptr;

    timeout_ = 1000;
}

// 当前RPC调用是否成功
bool RpcController::Failed() const{
    return is_failed_;
}

std::string RpcController::ErrorText() const{
    return error_info_;
}

void RpcController::StartCancel(){
    is_canceled_ = true;
}

void RpcController::SetFailed(const std::string& reason){
    error_info_ = reason;
    is_failed_ = true;
}

bool RpcController::IsCanceled() const{
    return is_canceled_;
}

void RpcController::NotifyOnCancel(google::protobuf::Closure* callback){

}

void RpcController::setError(int32_t err_code, const std::string err_info){
    error_code_ = err_code;
    error_info_ = err_info;
}

int32_t RpcController::getErrorCode(){
    return error_code_;
}

std::string RpcController::getErrorInfo(){
    return error_info_;
}

void RpcController::setReqId(const std::string& req_id){
    req_id_ = req_id;
}

std::string RpcController::getReqId(){
    return req_id_;
}

void RpcController::setLocalAddr(NetAddr::s_ptr addr){
    local_addr_ = addr;
}

void RpcController::setPeerAddr(NetAddr::s_ptr addr){
    peer_addr_ = addr;
}

NetAddr::s_ptr RpcController::getLocalAddr(){
    return local_addr_;
}

NetAddr::s_ptr RpcController::getPeerAddr(){
    return peer_addr_;
}

void RpcController::setTimeOut(int timeout){
    timeout_ = timeout;
}

int RpcController::getTimeOut(){
    return timeout_;
}

}   // namespace lrpc
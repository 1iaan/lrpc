#pragma once

#include <memory>
#include <string>

namespace lrpc{

class AbstractProtocol :public std::enable_shared_from_this<AbstractProtocol> {
public:
    typedef std::shared_ptr<AbstractProtocol> s_ptr;

    virtual ~AbstractProtocol() = default;

public:
    std::string getReqId() const { return req_id_; }
    
    void setReqId(const std::string& req_id) { req_id_ = req_id; }

protected:
    std::string req_id_;    // 请求号，唯一标识一个请求或响应


private:

};

} // namespace lrpc
#pragma once

#include <memory>
#include <string>

namespace lrpc{

struct AbstractProtocol :public std::enable_shared_from_this<AbstractProtocol> {
public:
    typedef std::shared_ptr<AbstractProtocol> s_ptr;

    virtual ~AbstractProtocol() = default;

public:
    std::string req_id_;    // 请求号，唯一标识一个请求或响应


private:

};

} // namespace lrpc
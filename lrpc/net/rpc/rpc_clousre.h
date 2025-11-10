#pragma once

#include <functional>
#include <google/protobuf/stubs/callback.h>
namespace lrpc{

class RpcClousre: public google::protobuf::Closure{
public:
    RpcClousre(){};
    ~RpcClousre(){};
    void Run() override;

private:
    std::function<void()> callback_{nullptr};
};

}   // namespace lrpc
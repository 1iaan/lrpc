#include "ping.pb.h"
#include "lrpc/net/tcp/tcp_server.h"
#include "lrpc/net/rpc/rpc_dispatcher.h"
#include "lrpc/common/log.h"
#include "lrpc/common/config.h"

class PingServiceImpl : public PingService {
public:
    void ping(google::protobuf::RpcController* controller,
              const ::PingRequest* request,
              ::PingResponse* response,
              ::google::protobuf::Closure* done) override {
        // 不做复杂逻辑，直接回包
        response->set_reply("pong");
        if (done) done->Run();
    }
};

int main() {
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    std::shared_ptr<PingServiceImpl> service = std::make_shared<PingServiceImpl>();
    lrpc::RpcDispatcher::GetRpcDispatcher()->registerService(service);

    auto addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    lrpc::TcpServer server(addr);
    server.start();

    return 0;
}

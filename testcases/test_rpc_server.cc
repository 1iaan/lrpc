#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/rpc/rpc_dispatcher.h"
#include "lrpc/net/tcp/net_addr.h"
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

#include "lrpc/net/tcp/tcp_server.h"
#include "order.pb.h"

class OrderImpl : public OrderService{
    void make_order(google::protobuf::RpcController* controller,
                       const ::makeOrderRequest* request,
                       ::makeOrderResponse* response,
                       ::google::protobuf::Closure* done){
        if(request->price() < 10){
            response->set_ret_code(-1);
            response->set_res_info("short balance");
            return;
        }
        response->set_order_id("251110");
    }

    void query_order(google::protobuf::RpcController* controller,
                       const ::queryOrderRequest* request,
                       ::queryOrderResponse* response,
                       ::google::protobuf::Closure* done){
        if(request->order_id() != "1999"){
            response->set_order_id("1999");
            response->set_price(1000);
            response->set_goods("apple");
        }else {
            response->set_res_info("no goods");
        }
    }
};

void test_tcpServer(){
    lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    
    lrpc::TcpServer tcp_server(addr);
    tcp_server.start();
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    std::shared_ptr<OrderImpl> order = std::make_shared<OrderImpl>();
    lrpc::RpcDispatcher::GetRpcDispatcher()->registerService(order);

    test_tcpServer();
    return 0;
}
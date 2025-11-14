#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/rpc/rpc_clousre.h"
#include "lrpc/net/rpc/rpc_controller.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/coder/string_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/net/rpc/rpc_channel.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "testcases/order.pb.h"
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <semaphore.h>
#include <string>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

void test_client(){
    lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    lrpc::TcpClient client(addr);
    
    client.connect([addr, &client]()->void{
        {}
        DEBUGLOG("{tcp_client connect 回调} connect to [%s] success", addr->toString().c_str());
        
        // std::shared_ptr<lrpc::StringProtocol> message = std::make_shared<lrpc::StringProtocol>("12345", "hello lrpc");
        std::shared_ptr<lrpc::TinyPBProtocol> message = std::make_shared<lrpc::TinyPBProtocol>();
        message->msg_id_ = "1688";

        makeOrderRequest request;
        request.set_price(100);
        request.set_goods("computer");
        if(!request.SerializeToString(&message->pb_data_)){
            ERRORLOG("serialize error");
            return;
        }

        message->method_name_ = "OrderService.make_order";

        client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            DEBUGLOG("client write message success, msg_id=[%s]", msg_ptr->msg_id_.c_str());
        });

        client.readMessage("1688", [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            std::shared_ptr<lrpc::TinyPBProtocol> message = std::dynamic_pointer_cast<lrpc::TinyPBProtocol>(msg_ptr);
            DEBUGLOG("client read message success, msg_id=[%s], get response=[%s,%s]", 
                message->msg_id_.c_str(), message->method_name_.c_str(), message->pb_data_.c_str());

            makeOrderResponse response;

            if(!response.ParseFromString(message->pb_data_)){
                ERRORLOG("deserialize error");
                return;
            }
            
            DEBUGLOG("get response success, response[%s]", response.ShortDebugString().c_str());
        });
    });
}

void makeOrder(lrpc::RpcChannel::s_ptr channel){
    for(int i = 0;i < 1000; ++i) {
        NEWMESSAGE(makeOrderRequest, request);
        NEWMESSAGE(makeOrderResponse, response);
        request->set_price(100*i);
        request->set_goods("computer");

        NEWRPCCONTROLLER(controller);
        controller->setMsgId(std::to_string(i));
        controller->setTimeOut(1000000);

        std::shared_ptr<lrpc::RpcClousre> closure = std::make_shared<lrpc::RpcClousre>([request, response, channel, controller]() mutable ->void{
            if(controller->getErrorCode() != 0){
                ERRORLOG("[%s] | call rpc failed, req[%s], error code[%d], error info[%s]",
                    controller->getMsgId().c_str(),
                    request->ShortDebugString().c_str(), 
                    controller->getErrorCode(), 
                    controller->getErrorInfo().c_str()
                );
                channel->getTcpClient()->stop();
                channel.reset();
            }else{
                INFOLOG("[%s] | call rpc success, req[%s], res[%s]", 
                    controller->getMsgId().c_str(),
                    request->ShortDebugString().c_str(), response->ShortDebugString().c_str());
                // 业务逻辑
            }
        });

        CALLRPC(OrderService_Stub, channel, make_order, controller, request, response, closure);
        // sleep(5);
    }
}

void queryOrder(lrpc::RpcChannel::s_ptr channel){
    NEWMESSAGE(queryOrderRequest, request);
    NEWMESSAGE(queryOrderResponse, response);
    request->set_order_id("1999");

    NEWRPCCONTROLLER(controller);
    controller->setMsgId("2");
    controller->setTimeOut(10000);

    std::shared_ptr<lrpc::RpcClousre> closure = std::make_shared<lrpc::RpcClousre>([request, response, channel, controller]() mutable ->void{
        if(controller->getErrorCode() != 0){
            ERRORLOG("[%s] | call rpc failed, req[%s], error code[%d], error info[%s]",
                controller->getMsgId().c_str(),
                request->ShortDebugString().c_str(), 
                controller->getErrorCode(), 
                controller->getErrorInfo().c_str()
            );
            channel->getTcpClient()->stop();
            channel.reset();
        }else{
            INFOLOG("[%s] | call rpc success, req[%s], res[%s]", 
                controller->getMsgId().c_str(),
                request->ShortDebugString().c_str(), response->ShortDebugString().c_str());
            // 业务逻辑
        }

    });

    CALLRPC(OrderService_Stub, channel, query_order, controller, request, response, closure);
}

void test_channel(){
    // lrpc::IPNetAddr::s_ptr addr = std::make_shared<lrpc::IPNetAddr>("127.0.0.1", 12345);
    // std::shared_ptr<lrpc::RpcChannel> channel = std::make_shared<lrpc::RpcChannel>(addr);
    NEWRPCCHANNEL("127.0.0.1:12345", channel);

    // std::shared_ptr<makeOrderRequest> request = std::make_shared<makeOrderRequest>();
    // std::shared_ptr<makeOrderResponse> response = std::make_shared<makeOrderResponse>();

    // std::shared_ptr<lrpc::RpcController> controller = std::make_shared<lrpc::RpcController>();

    // channel->init(controller, request, response, closure);
    // OrderService_Stub stub(channel.get());
    // stub.make_order(controller.get(), request.get(), response.get(), closure.get());


    makeOrder(channel);

    // queryOrder(channel);
    
    channel->getTcpClient()->join();
    channel->getTcpClient()->stop();
    channel.reset();
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    // test_client();
    test_channel();

    return 0;
}
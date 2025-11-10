#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/coder/string_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "testcases/order.pb.h"
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
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
        message->req_id_ = "1688";

        makeOrderRequest request;
        request.set_price(100);
        request.set_goods("computer");
        if(!request.SerializeToString(&message->pb_data_)){
            ERRORLOG("serialize error");
            return;
        }

        message->method_name_ = "OrderService.make_order";

        client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            DEBUGLOG("client write message success, req_id=[%s]", msg_ptr->req_id_.c_str());
        });

        client.readMessage("1688", [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            std::shared_ptr<lrpc::TinyPBProtocol> message = std::dynamic_pointer_cast<lrpc::TinyPBProtocol>(msg_ptr);
            DEBUGLOG("client read message success, req_id=[%s], get response=[%s,%s]", 
                message->req_id_.c_str(), message->method_name_.c_str(), message->pb_data_.c_str());

            makeOrderResponse response;

            if(!response.ParseFromString(message->pb_data_)){
                ERRORLOG("deserialize error");
                return;
            }
            
            DEBUGLOG("get response success, response[%s]", response.ShortDebugString().c_str());
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
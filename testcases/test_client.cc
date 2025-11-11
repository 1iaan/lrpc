#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include "lrpc/net/coder/string_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"
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
        
        // std::shared_ptr<lrpc::StringProtocol> message = std::make_shared<lrpc::StringProtocol>("12345", "hello lrpc");
        std::shared_ptr<lrpc::TinyPBProtocol> message = std::make_shared<lrpc::TinyPBProtocol>();
        message->msg_id_ = "12345";
        message->pb_data_ = "test pb data";
        message->method_name_ = "test method name";

        client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            DEBUGLOG("client write message success, msg_id=[%s]", msg_ptr->msg_id_.c_str());
        });

        // client.readMessage("12345", [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
        //     std::shared_ptr<lrpc::StringProtocol> message = std::dynamic_pointer_cast<lrpc::StringProtocol>(msg_ptr);
        //     DEBUGLOG("client read message success, msg_id=[%s], msg=[%s]", 
        //         message->msg_id_.c_str(), message->msg_.c_str()
        //     );
        // });
        client.readMessage("12345", [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
            std::shared_ptr<lrpc::TinyPBProtocol> message = std::dynamic_pointer_cast<lrpc::TinyPBProtocol>(msg_ptr);
            DEBUGLOG("client read message success, msg_id=[%s], get response=[%s,%s]", 
                message->msg_id_.c_str(), message->method_name_.c_str(), message->pb_data_.c_str()
            );
        });

        // client.writeMessage(message, [](lrpc::AbstractProtocol::s_ptr msg_ptr)->void{
        //     DEBUGLOG("client write message success, msg_id=[%s]", msg_ptr->msg_id_.c_str());
        // });
    });
}

int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    // test_connect();
    test_client();

    return 0;
}
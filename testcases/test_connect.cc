#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

void test_connection(){
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


int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    test_connection();

    return 0;
}
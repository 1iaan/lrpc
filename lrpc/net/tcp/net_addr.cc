#include "lrpc/common/log.h"
#include "lrpc/net/tcp/net_addr.h"
#include <arpa/inet.h>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>

namespace lrpc{

IPNetAddr::IPNetAddr(const std::string& ip, uint16_t port):ip_(ip), port_(port){
    memset(&addr_, 0, sizeof(addr_));

    addr_.sin_family = AF_INET;
    // inet_addr() 点分十进制IP地址 转换为网络字节序
    addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
    // 主机序转换为网络字节序
    addr_.sin_port = htons(port_);
}

IPNetAddr::IPNetAddr(const std::string& addr){
    size_t i = addr.find_first_of(":");
    if(i == addr.size()){
        ERRORLOG("invalid ipv4 addr %s", addr.c_str());
        return ;
    }
    ip_ = addr.substr(0, i);
    port_ = std::atoi(addr.substr(i+1, addr.size()-i-1).c_str());

    
    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
    addr_.sin_port = htons(port_);
}

IPNetAddr::IPNetAddr(sockaddr_in addr):addr_(addr){
    ip_ = inet_ntoa(addr_.sin_addr);
    port_ = ntohs(addr.sin_port);
}

sockaddr* IPNetAddr::getSockAddr(){
    return reinterpret_cast<sockaddr*>(&addr_);
}

socklen_t IPNetAddr::getSockLen(){
    return sizeof(addr_);
}

int IPNetAddr::getFamily(){
    return AF_INET;
}

std::string IPNetAddr::toString(){
    return ip_ + ":" + std::to_string(port_);
}

bool IPNetAddr::checkValid(){
    if(ip_.empty()) {
        return false;
    }
    if(port_ <= 0 || port_ > 65535){
        return false;
    }
    if(inet_addr(ip_.c_str()) == INADDR_NONE) {
        return false;
    }
    return true;
}

} // namespace lrpc
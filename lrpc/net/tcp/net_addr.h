#pragma once

#include <cstdint>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
namespace lrpc {

class NetAddr{

public:
    typedef std::shared_ptr<NetAddr> s_ptr;

    virtual sockaddr* getSockAddr() = 0;

    virtual socklen_t getSockLen() = 0;

    virtual int getFamily() = 0;

    virtual std::string toString() = 0;

    virtual bool checkValid() = 0;
};

class IPNetAddr final : public NetAddr{

public:
    IPNetAddr(const std::string& ip, uint16_t port);

    IPNetAddr(const std::string& addr);

    IPNetAddr(sockaddr_in addr);

    sockaddr* getSockAddr();

    socklen_t getSockLen();

    int getFamily();

    std::string toString();

    bool checkValid();

private:
    std::string  ip_;
    uint16_t port_;
    sockaddr_in addr_;
};

}   // namespace lrpc
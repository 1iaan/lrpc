#pragma once

#include "lrpc/net/coder/abs_coder.h"
#include "lrpc/net/coder/abs_protocol.h"
#include <cstddef>
#include <memory>

namespace lrpc{

struct StringProtocol : public AbstractProtocol {
public:
    StringProtocol() = default;
    StringProtocol(const std::string &req_id, const std::string& s): msg_(s) {
        req_id_ = req_id;
    }

public:

    std::string getBytesStream() { return req_id_ +  "/" + msg_; }

public:
    std::string msg_;
}; 

class StringCoder : public AbstractCoder {
public:
    // 将 message 编码成字节流  ， 写入 buffer
    void encode(std::vector<AbstractProtocol::s_ptr>& messages, TcpBuffer::s_ptr out_buffer){
        for(size_t i = 0;i < messages.size(); ++ i){
            std::shared_ptr<StringProtocol> msg = std::dynamic_pointer_cast<StringProtocol>(messages[i]);
            out_buffer->writeToBuf(msg->getBytesStream().c_str(), msg->getBytesStream().length());
        }
    }
    
    // 从 buffer 读取字节流     ， 解码成 message
    void decode(std::vector<AbstractProtocol::s_ptr>& out_messages, TcpBuffer::s_ptr buffer){
        std::vector<char> re;
        buffer->readFromBuf(re, buffer->readable());


        
        std::string data(re.begin(), re.end());
        size_t pos = data.find('/');
        if (pos == std::string::npos) {
            return ;
        }

        // 分割成功，第一部分是 req，第二部分是 msg
        std::string req_id = data.substr(0, pos);
        std::string msg = data.substr(pos + 1);

        std::shared_ptr<StringProtocol> message = std::make_shared<StringProtocol>(req_id, msg); 

        
        out_messages.push_back(message);
    }
private:
};

} // namespace lrpc
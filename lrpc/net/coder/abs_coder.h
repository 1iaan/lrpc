#pragma once

#include "lrpc/net/tcp/tcp_buffer.h"
#include "lrpc/net/coder/abs_protocol.h"

namespace lrpc{

class AbstractCoder {
public:
    virtual ~AbstractCoder() = default;

public:
    // 将 message 编码成字节流  ， 写入 buffer
    virtual void encode(std::vector<AbstractProtocol::s_ptr>& messages, TcpBuffer::s_ptr out_buffer) = 0;
    
    // 从 buffer 读取字节流     ， 解码成 message
    virtual void decode(std::vector<AbstractProtocol::s_ptr>& out_messages, TcpBuffer::s_ptr buffer) = 0;

private:
};


} // namespace lrpc
#pragma once

#include "lrpc/net/coder/abs_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"

namespace lrpc {

class TinyPBCoder : public AbstractCoder {
public:
    TinyPBCoder() = default;
    ~TinyPBCoder() = default;
public:
    // 将 message 编码成字节流  ， 写入 buffer
    void encode(std::vector<AbstractProtocol::s_ptr>& messages, TcpBuffer::s_ptr out_buffer);
    
    // 从 buffer 读取字节流     ， 解码成 message
    void decode(std::vector<AbstractProtocol::s_ptr>& out_messages, TcpBuffer::s_ptr buffer);

private:
    const char* encodeTinyPB(std::shared_ptr<TinyPBProtocol> msg, int &len);

};

} // namespace lrpc
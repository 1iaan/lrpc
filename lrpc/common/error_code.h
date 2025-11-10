#pragma once

namespace lrpc{

#ifndef SYS_ERROR_PREFIX
#define SYS_ERROR_PREFIX(xx) 0266##xx

#endif

const int ERROR_PEER_CLOSE = SYS_ERROR_PREFIX(0000);            // 对端关闭
const int ERROR_FAILED_CONNECT = SYS_ERROR_PREFIX(0001);        // 连接失败
const int ERROR_FAILED_GET_REPLAY = SYS_ERROR_PREFIX(0002);     // 返回值
const int ERROR_FAILED_DESERIALIZE = SYS_ERROR_PREFIX(0003);    // 反序列化
const int ERROR_FAILED_SERIALIZE = SYS_ERROR_PREFIX(0004);      // 序列化

const int ERROR_PARSE_SERVICE_NAME = SYS_ERROR_PREFIX(0005);
const int ERROR_SERVICE_NOT_FOUND = SYS_ERROR_PREFIX(0006);
const int ERROR_METHOD_NOT_FOUND = SYS_ERROR_PREFIX(0007);

} // namespace lrpc
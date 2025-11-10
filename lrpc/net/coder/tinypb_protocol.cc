#include "lrpc/net/coder/tinypb_protocol.h"

namespace lrpc {


char TinyPBProtocol::PB_START = 0x02;
char TinyPBProtocol::PB_END = 0x03;

}   // namespace lrpc
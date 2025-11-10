#include "lrpc/net/rpc/rpc_clousre.h"

namespace lrpc{
    
void RpcClousre::Run() {
    if(callback_) {
        callback_();
    }
}

}   // namespace lrpc
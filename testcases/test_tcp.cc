#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/tcp/net_addr.h"


int main(){
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    lrpc::IPNetAddr addr("127.0.0.1", 12345);
    DEBUGLOG("IPNetAddr toString %s", addr.toString().c_str());
    return 0;
}
#include "log.h"

int main(){

    // DEBUGLOG("test log %s", 11);
    std ::string msg = (new lrpc ::LogEvent(lrpc ::LogLevel ::Debug))->toString() +
                   lrpc ::formatString("test log %s", "11");
    msg += "\n";
    lrpc ::Logger ::GetGlobalLogger()->pushLog(msg);
    lrpc ::Logger ::GetGlobalLogger()->log();

    return 0;
}
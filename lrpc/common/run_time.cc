#include "lrpc/common/run_time.h"
#include <cstddef>

namespace lrpc{

thread_local RunTime* t_run_time = NULL ;

RunTime* RunTime::GetRunTime(){
    if(!t_run_time){
        t_run_time = new RunTime;
    }
    return t_run_time;
}

}
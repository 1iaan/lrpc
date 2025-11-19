#pragma once

#include <string>
namespace lrpc{

class RunTime{
public:

public:
    static RunTime* GetRunTime();

// private:
    std::string msg_id_;
    std::string method_name_;
};

}
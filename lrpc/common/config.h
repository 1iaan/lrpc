#pragma once

#include <map>
#include <string>

namespace lrpc{

class Config{
public:
    Config(const char* xmlfile);

public:
    static void SetGlobalConfig(const char* xmlfile);
    
    static Config* GetGlobalConfig();

public:
    std::string log_level_;
};

} // namespace lrpc
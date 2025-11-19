#pragma once

#include <map>
#include <string>
#include <tinyxml/tinyxml.h>

namespace lrpc{

class Config{
public:
    Config(const char* xmlfile);

public:
    static void SetGlobalConfig(const char* xmlfile);
    
    static Config* GetGlobalConfig();

public:
    std::string xmlfile_;

    std::string log_level_;
    std::string log_file_name_;
    std::string log_file_path_;
    int log_max_file_size_;
    int log_sync_internal_;

    int io_thread_num_;
    int worker_thread_num_;

private:
    void getLogConfig(TiXmlElement* root_node);

    void getThreadConfig(TiXmlElement* root_node);
};

} // namespace lrpc
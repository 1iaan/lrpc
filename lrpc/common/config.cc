#include "lrpc/common/config.h"
#include <cstddef>
#include <tinyxml/tinyxml.h>

namespace lrpc{
#define READ_XML_NODE(name, parent)                                             \
    TiXmlElement* name##_node = parent->FirstChildElement(#name);               \
    if(!name##_node){                                                           \
        printf("Start lrpc server error, failed to read node [%s]\n", #name);   \
        exit(0);                                                                \
    }                                                                           \

#define READ_STR_FROM_XML_NODE(name, parent)                                    \
    TiXmlElement* name##_node = parent->FirstChildElement(#name);               \
    if(!name##_node || !name##_node->GetText()) {                               \
        printf("Start lrpc server error, failed to read node [%s]\n", #name);   \
    }                                                                           \
    std::string name = std::string(name##_node->GetText());                     \


static Config* g_config = NULL;
void Config::SetGlobalConfig(const char* xmlfile){
    if(g_config == NULL){
        g_config = new Config(xmlfile);
    }
}

Config* Config::GetGlobalConfig(){
    return g_config;
}

Config::Config(const char* xmlfile){
    TiXmlDocument* xml_document = new TiXmlDocument();
    bool rt = xml_document->LoadFile(xmlfile);

    if(!rt){
        printf("Start lrpc server error, failed to read config file %s, error info [%s]\n", xmlfile, xml_document->ErrorDesc());
        exit(0);
    }

    READ_XML_NODE(root, xml_document);
    READ_XML_NODE(log, root_node);

    READ_STR_FROM_XML_NODE(log_level, log_node);

    log_level_ = log_level;
    
    printf("Config: \n");
    printf("\tlog_level:\t[%s]\n", log_level_.c_str());
}

} // namespace lrpc
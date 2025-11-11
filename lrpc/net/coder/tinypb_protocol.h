#pragma once

#include "lrpc/net/coder/abs_protocol.h"
#include <cstdint>

namespace lrpc {

struct TinyPBProtocol : public AbstractProtocol {
public:
    typedef std::shared_ptr<TinyPBProtocol> s_ptr ;

    static char PB_START;
    static char PB_END;

public:
    
    int32_t package_len_{0};

    int32_t msg_id_len_{0};
    // msg_id 继承父类
    
    int32_t method_name_len_{0};
    std::string method_name_;

    int32_t err_code_{0};
    int32_t err_info_len_{0};
    std::string err_info_;

    std::string pb_data_;

    int32_t check_sum_{0};

    bool parse_success_{false};
};


} // namespace lrpc
#include "lrpc/common/log.h"
#include "lrpc/common/util.h"
#include "lrpc/net/coder/tinypb_coder.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <netinet/in.h>

namespace lrpc {

// 将 message 编码成字节流  ， 写入 buffer
void TinyPBCoder::encode(std::vector<AbstractProtocol::s_ptr>& messages, TcpBuffer::s_ptr out_buffer){
    for(auto &message: messages){
        TinyPBProtocol::s_ptr msg = std::dynamic_pointer_cast<TinyPBProtocol>(message);
        int len = 0;
        const char* buf = encodeTinyPB(msg, len);

        if(buf != NULL && len != 0){
            out_buffer->writeToBuf(buf, len);
        }

        if(buf){
            free((void*)buf);
            buf = NULL;
        }
    }
}

// 从 buffer 读取字节流     ， 解码成 message
void TinyPBCoder::decode(std::vector<AbstractProtocol::s_ptr>& out_messages, TcpBuffer::s_ptr buffer){
    // 遍历Buffer，找到PB_START，找到后解析出长度，然后得到结束符的位置判断是否为PG_END
    while(true){
        std::vector<char> &tmp = buffer->buffer_;
        int start_index = buffer->getReadIdx();
        int end_index = -1;

        int package_len = 0;

        bool parse_success = false;
        int i = 0;
        for(i = start_index;i < buffer->getWriteIdx(); ++ i){
            if(tmp[i] == TinyPBProtocol::PB_START){
                // 取四个字节，由于是网络字节序需要转换
                if(i + 1  < buffer->getWriteIdx()){
                    package_len = getInt32FromNetByte(&tmp[i+1]);
                    DEBUGLOG("GET packpage length = %d", package_len);

                    // 结束符索引
                    int j = i + package_len - 1;
                    // 结束符不存在， 说明只接受了半个包，跳过 i 会递增到末尾 然后退出循环
                    // 感觉可以直接退出? 
                    if(j >= buffer->getWriteIdx()){
                        continue;
                    }

                    if(tmp[j] == TinyPBProtocol::PB_END){
                        start_index = i;
                        end_index = j;
                        parse_success = true;
                        break;
                    }
                }
            }
        }

        if(i >= buffer->getWriteIdx()){
            DEBUGLOG("decode end, read all buffer data");
            return;
        }

        if(parse_success){
            // INFOLOG("start_idx[%d], end_idx[%d]", start_index, end_index);

            TinyPBProtocol::s_ptr message = std::make_shared<TinyPBProtocol>();
            message->package_len_ = package_len;

            // msg_id
            int msg_id_len_index = start_index + sizeof(char) + sizeof(message->package_len_);
            if(msg_id_len_index >= end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, req_len[%d] > end_index[%d]", msg_id_len_index, end_index);
                continue;
            }
            message->msg_id_len_ = getInt32FromNetByte(&tmp[msg_id_len_index]);
            DEBUGLOG("parse msg_id_len=%d", message->msg_id_len_);

            int msg_id_index = msg_id_len_index + sizeof(message->msg_id_len_);
            char msg_id[100] = {0};
            if(msg_id_index + message->msg_id_len_ > end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, msg_id data out of range");
                continue;
            }
            memcpy(&msg_id[0], &tmp[msg_id_index], message->msg_id_len_);
            message->msg_id_ = std::string(msg_id);
            DEBUGLOG("parse msg_id=%s", message->msg_id_.c_str());

            // method
            int method_name_len_index = msg_id_index + message->msg_id_len_;
            if(method_name_len_index >= end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, method_len_index[%d] > end_index[%d]", method_name_len_index, end_index);
                continue;
            }
            message->method_name_len_ = getInt32FromNetByte(&tmp[method_name_len_index]);
            DEBUGLOG("parse method_len=%d", message->method_name_len_);

            int method_name_index = method_name_len_index + sizeof(message->method_name_len_);
            char method_name[512] = {0};
            if(method_name_index + message->method_name_len_ > end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, method_name data out of range");
                continue;
            }
            memcpy(&method_name[0], &tmp[method_name_index], message->method_name_len_);
            message->method_name_ = std::string(method_name);
            DEBUGLOG("parse method_name=%s", message->method_name_.c_str());

            // errcode
            int err_code_index = method_name_index + message->method_name_len_;
            if(err_code_index >= end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, err_code_index[%d] > end_index[%d]", err_code_index, end_index);
                continue;
            }
                
            message->err_code_ = getInt32FromNetByte(&tmp[err_code_index]);
            DEBUGLOG("parse err_code=%d", message->err_code_);
        
            int err_info_len_index = err_code_index + sizeof(message->err_code_);
            if(err_info_len_index >= end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, err_info_len_index[%d] > end_index[%d]", err_info_len_index, end_index);
            }
            message->err_info_len_ = getInt32FromNetByte(&tmp[err_info_len_index]);
            DEBUGLOG("parse err_info_len=%d", message->err_info_len_);

            int err_info_index = err_info_len_index + sizeof(message->err_info_len_);
            char err_info[512] = {0};
            if(err_info_index + message->err_info_len_ > end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, err_info data out of range");
                continue;
            }
            memcpy(&err_info[0], &tmp[err_info_index], message->err_info_len_);
            message->err_info_ = std::string(err_info);
            DEBUGLOG("parse err_info=%s", message->err_info_.c_str());

            // pbdata
            int pb_data_len = message->package_len_ 
                            - message->method_name_len_ 
                            - message->msg_id_len_ 
                            - message->err_info_len_ 
                            - 2 - 24;

            int pb_data_index = err_info_index + message->err_info_len_;
            if(pb_data_len < 0 || pb_data_index + pb_data_len > end_index){
                message->parse_success_ = false;
                ERRORLOG("parse error, pb_data length invalid: %d", pb_data_len);
                continue;
            }

            message->pb_data_ = std::string(&tmp[pb_data_index], pb_data_len);

            // TODO: checksum

            message->parse_success_ = true;
            buffer->moveReadIndex(end_index - start_index + 1);

            out_messages.push_back(message);
            DEBUGLOG("decode message[%s] success", message->msg_id_.c_str());
        }
    }
}

const char* TinyPBCoder::encodeTinyPB(TinyPBProtocol::s_ptr msg, int &len){
    if(msg->msg_id_.empty()) {
        msg->msg_id_ = "randomly";
    }
    DEBUGLOG("get msg_id=%s", msg->msg_id_.c_str());

    // package

    int package_len = 2 + 24 + msg->msg_id_.length() + msg->method_name_.length() + msg->err_info_.length() + msg->pb_data_.length();
    DEBUGLOG("get package_len=%d", package_len);

    char* buf = reinterpret_cast<char *>(malloc(package_len));
    char* tmp = buf;

    *tmp = TinyPBProtocol::PB_START;
    tmp++;

    int package_len_net = htonl(package_len);
    memcpy(tmp, &package_len_net, sizeof(package_len_net));
    tmp += sizeof(package_len_net);

    // msg_id
    int msg_id_len = msg->msg_id_.length();
    int msg_id_len_net = htonl(msg_id_len);
    memcpy(tmp, &msg_id_len_net, sizeof(msg_id_len_net));
    tmp += sizeof(msg_id_len_net);

    if(!msg->msg_id_.empty()){
        memcpy(tmp, &msg->msg_id_[0], msg_id_len);
        // INFOLOG("%s",tmp);
        tmp += msg_id_len;
    }

    // method
    int method_name_len = msg->method_name_.length();
    int method_name_len_net = htonl(method_name_len);
    memcpy(tmp, &method_name_len_net, sizeof(method_name_len_net));
    tmp += sizeof(method_name_len_net);

    if(!msg->method_name_.empty()){
        memcpy(tmp, &msg->method_name_[0], method_name_len);
        tmp += method_name_len;
    }

    // errcode
    int32_t err_code_net = htonl(msg->err_code_);
    memcpy(tmp, &err_code_net, sizeof(err_code_net));
    tmp += sizeof(err_code_net);

    int err_info_len = msg->err_info_.length();
    int err_info_len_net = htonl(err_info_len);
    memcpy(tmp, &err_info_len_net, sizeof(err_info_len_net));
    tmp += sizeof(err_info_len_net);

    if(!msg->err_info_.empty()){
        memcpy(tmp, &msg->err_info_[0], err_info_len);
        tmp += err_info_len;
    }

    // pb_data
    if(!msg->pb_data_.empty()){
        memcpy(tmp, &msg->pb_data_[0], msg->pb_data_.length());
        tmp += msg->pb_data_.length();
    }

    // checksum 
    int32_t checksum_net = htonl(1);
    memcpy(tmp, &checksum_net, sizeof(checksum_net));
    tmp += sizeof(checksum_net);

    //
    *tmp = TinyPBProtocol::PB_END;

    msg->package_len_ = package_len;
    msg->msg_id_len_ = msg_id_len;
    msg->method_name_len_ = method_name_len;
    msg->err_info_len_ = err_info_len;
    msg->parse_success_ = true;

    len = package_len;

    DEBUGLOG("encode message[%s] success", msg->msg_id_.c_str());
    return buf;

}


}  // namespace lrpc
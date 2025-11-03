#include "lrpc/common/log.h"
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <vector>
#include "lrpc/net/tcp/tcp_buffer.h"

namespace lrpc{
TcpBuffer::TcpBuffer(size_t size):size_(size){
    buffer_.resize(size_);
}

TcpBuffer::~TcpBuffer(){

}

size_t TcpBuffer::readable(){
if (full_) return size_;
    if (write_idx_ >= read_idx_) {
        return write_idx_ - read_idx_;
    }
    return size_ - read_idx_ + write_idx_;
}

size_t TcpBuffer::writeable(){
    return size_ - readable();
}

void TcpBuffer::resizeBuffer(size_t new_size){
    std::vector<char> new_buf(new_size);
    size_t r = readable();
    
    // 手动复制数据，不调用 readFromBuf
    if (r > 0) {
        size_t end_data = size_ - read_idx_;
        if (r <= end_data) {
            memcpy(&new_buf[0], &buffer_[read_idx_], r);
        } else {
            memcpy(&new_buf[0], &buffer_[read_idx_], end_data);
            memcpy(&new_buf[end_data], &buffer_[0], r - end_data);
        }
    }
    
    buffer_.swap(new_buf);
    size_ = new_size;
    read_idx_ = 0;
    write_idx_ = r;
    full_ = (r == size_);
}

void TcpBuffer::writeToBuf(const char* data, size_t len){
    if(len > writeable()){
        // 扩容
        resizeBuffer(len + size_);
    }
    size_t end_space = size_ - write_idx_;
    if(len <= end_space){
        memcpy(&buffer_[write_idx_], data, len);
        write_idx_ = (write_idx_ + len) % size_;
    } else {
        memcpy(&buffer_[write_idx_], data, end_space);
        memcpy(&buffer_[0], data + end_space, len - end_space);
        write_idx_ = len - end_space;
    }

    if (write_idx_ == read_idx_) full_ = true;
}

void TcpBuffer::readFromBuf(std::vector<char>& re, size_t len){
    size_t avail = readable();
    if (avail == 0) return;
    len = std::min(len, avail);

    std::vector<char> tmp(len);
    size_t end_data = size_ - read_idx_;
    if (len <= end_data) {
        memcpy(&tmp[0], &buffer_[read_idx_], len);
        read_idx_ = (read_idx_ + len) % size_;
    } else {
        memcpy(&tmp[0], &buffer_[read_idx_], end_data);
        memcpy(&tmp[end_data], &buffer_[0], len - end_data);
        read_idx_ = len - end_data;
    }

    if (full_) full_ = false;
    re.swap(tmp);
}


void TcpBuffer::moveWriteIndex(size_t len){
    if (len == 0) return;
    
    size_t avail = writeable();
    if (len > avail) {
        len = avail;  // 限制移动长度不超过可写空间
    }
    
    write_idx_ = (write_idx_ + len) % size_;
    
    // 更新 full_ 标志
    if (write_idx_ == read_idx_) {
        full_ = true;
    }
}

void TcpBuffer::moveReadIndex(size_t len){
    if (len == 0) return;
    
    size_t avail = readable();
    if (len > avail) {
        len = avail;  // 限制移动长度不超过可读数据
    }
    
    read_idx_ = (read_idx_ + len) % size_;
    
    // 更新 full_ 标志
    if (read_idx_ != write_idx_) {
        full_ = false;
    }
}   


} // namespace lrpc
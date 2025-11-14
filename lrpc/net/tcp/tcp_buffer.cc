#include "lrpc/common/log.h"
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <vector>
#include "lrpc/net/tcp/tcp_buffer.h"

namespace lrpc{
TcpBuffer::TcpBuffer(size_t size):size_(size){
    buffer_.resize(size_);
}

TcpBuffer::~TcpBuffer(){

}

size_t TcpBuffer::readable(){
    return write_idx_ - read_idx_;
}

size_t TcpBuffer::writeable(){
    return size_ - write_idx_;
}

void TcpBuffer::resizeBuffer(size_t new_size){
    std::vector<char> new_buf(new_size);

    size_t r = readable();
    if (r > 0) {
        memcpy(&new_buf[0], &buffer_[read_idx_], r);
    }

    buffer_.swap(new_buf);
    size_ = new_size;
    read_idx_ = 0;
    write_idx_ = r;
}

void TcpBuffer::writeToBuf(const char* data, size_t len){
    // 若空间不够，扩容
    if (len > writeable()) {
        size_t new_size = std::max(size_ * 2, write_idx_ + len);
        resizeBuffer(new_size);
    }

    memcpy(&buffer_[write_idx_], data, len);
    write_idx_ += len;
}

void TcpBuffer::readFromBuf(std::vector<char>& re, size_t len){
    size_t avail = readable();
    if (avail == 0) return;

    len = std::min(len, avail);

    std::vector<char> tmp(len);
    memcpy(&tmp[0], &buffer_[read_idx_], len);

    read_idx_ += len;
    re.swap(tmp);

    // 若 buffer 前端空闲太多，把未读数据搬到前面（避免无限增长）
    if (read_idx_ > size_ / 2) {
        size_t r = readable();
        memmove(&buffer_[0], &buffer_[read_idx_], r);
        read_idx_ = 0;
        write_idx_ = r;
    }
}


void TcpBuffer::moveWriteIndex(size_t len){
    size_t avail = writeable();
    if (len > avail) len = avail;

    write_idx_ += len;
}

void TcpBuffer::moveReadIndex(size_t len){
    size_t avail = readable();
    if (len > avail) len = avail;

    read_idx_ += len;

    // 同样进行整理
    if (read_idx_ > size_ / 2) {
        size_t r = readable();
        memmove(&buffer_[0], &buffer_[read_idx_], r);
        read_idx_ = 0;
        write_idx_ = r;
    }
}   

} // namespace lrpc
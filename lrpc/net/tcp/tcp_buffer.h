#pragma once

#include <cstddef>
#include <vector>
namespace lrpc{

class TcpBuffer{
public:
    TcpBuffer(size_t size);

    ~TcpBuffer();

public:
    size_t readable();
    
    size_t writeable();

    size_t getReadIdx() { return read_idx_; }

    size_t getWriteIdx() { return write_idx_; }

    void resizeBuffer(size_t new_size);

    void writeToBuf(const char* data, size_t len);

    void readFromBuf(std::vector<char>& re, size_t size);

    void moveReadIndex(size_t len);

    void moveWriteIndex(size_t len);
private:
    size_t read_idx_{0};
    size_t write_idx_{0};
    size_t size_{0};
    bool full_{false};

    std::vector<char> buffer_;
};

} // namespace lrpc
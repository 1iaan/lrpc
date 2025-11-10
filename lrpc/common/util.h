#pragma once

#include <cstdint>
#include <sys/types.h>
namespace lrpc {

pid_t get_process_id();
pid_t get_thread_id();

int64_t get_now_ms();

int32_t getInt32FromNetByte(const char* buf);

} // namespace lrpc
#pragma once

#include <sys/types.h>
namespace lrpc {

pid_t get_process_id();
pid_t get_thread_id();

} // namespace lrpc
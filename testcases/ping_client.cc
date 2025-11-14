#include "lrpc/net/rpc/rpc_clousre.h"
#include "lrpc/net/rpc/rpc_channel.h"
#include "lrpc/common/log.h"
#include "lrpc/common/config.h"
#include "lrpc/net/rpc/rpc_controller.h"
#include "testcases/ping.pb.h"
#include <atomic>
#include <semaphore.h>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <vector>

using namespace std::chrono;

std::atomic<int> success_count{0};
std::atomic<int> fail_count{0};

void client_worker(int thread_id, int requests_per_thread) {
    NEWRPCCHANNEL("127.0.0.1:12345", channel);

    sem_t loop_ok{0};
    sem_init(&loop_ok, 0, 0);

    for (int i = 0; i < requests_per_thread; ++i) {
        NEWMESSAGE(PingRequest, req);
        NEWMESSAGE(PingResponse, rsp);
        req->set_msg("hello");

        NEWRPCCONTROLLER(controller);
        controller->setMsgId(std::to_string(thread_id * 100000 + i));
        controller->setTimeOut(5000);

        std::shared_ptr<lrpc::RpcClousre> closure = std::make_shared<lrpc::RpcClousre>(
            [req, rsp, controller, i, requests_per_thread, &loop_ok]() mutable {
                if (controller->getErrorCode() == 0) {
                    success_count++;
                } else {
                    fail_count++;
                }
                if(i == requests_per_thread -1 ){
                    // INFOLOG("sempost");
                    sem_post(&loop_ok);
                }
            });

        CALLRPC(PingService_Stub, channel, ping, controller, req, rsp, closure);
    }

    sem_wait(&loop_ok);
    channel->stop();
    channel.reset();
}

int main() {
    lrpc::Config::SetGlobalConfig("../conf/lrpc.xml");
    lrpc::Logger::SetGlobalLogger();

    int thread_num = 1;
    int requests_per_thread = 1000;

    std::vector<std::thread> threads;
    auto start = high_resolution_clock::now();

    for (int i = 0; i < thread_num; ++i) {
        threads.emplace_back(client_worker, i, requests_per_thread);
    }

    for (auto& t : threads) t.join();

    auto end = high_resolution_clock::now();
    double duration = duration_cast<milliseconds>(end - start).count() / 1000.0;

    int total = thread_num * requests_per_thread;
    double qps = total / duration;

    printf("======== RPC QPS TEST ========\n");
    printf("Total Requests: %d\n", total);
    printf("Success: %d\n", success_count.load());
    printf("Fail: %d\n", fail_count.load());
    printf("Total Time: %.3f s\n", duration);
    printf("QPS: %.2f req/s\n", qps);
    printf("==============================\n");

    return 0;
}

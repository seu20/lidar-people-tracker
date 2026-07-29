#pragma once
#include "Protocol.h"
#include "SharedState.h"
#include <pthread.h>
#include <atomic>

class UDPReceiver {
private:
    int sockfd;
    SharedState *state_;
    std::atomic<bool> running_;
    pthread_t thread_id_;

    static void* threadfunc(void *arg);
    void run();
    void handlePacket(const uint8_t *data, size_t len);

public:
    UDPReceiver(int port, SharedState *state);
    ~UDPReceiver();

    bool start();
    void stop();
};
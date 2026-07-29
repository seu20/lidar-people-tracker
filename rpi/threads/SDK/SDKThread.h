#pragma once
#include "SensorWorker.h"
#include <pthread.h>
#include <atomic>
#include <string>

class SDKThread {
private:
    Lidar *sensor_;
    std::string port_;
    int baudrate_;
    std::atomic<bool> running_;
    pthread_t thread_id_;


    static void* threadfunc(void* arg);
    void run();
public:
    SDKThread(const std::string &port, int baudrate, Lidar* sensor);
    ~SDKThread();

    bool start();
    void stop();
};
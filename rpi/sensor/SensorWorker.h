#pragma once
#include <iostream>
#include <vector>
#include <mutex>
#include <pthread.h>
#include <Utility.h>
#include "CYdLidar.h"
#include "Protocol.h"



// 라이다 센서 - Sensor 상속
class Lidar{
private:
    CYdLidar lidar;
    std::vector<SensorPoint> points_;
    static constexpr uint64_t TIMEOUT_MS = 500;
    bool data_ready_ = false;
    mutable pthread_mutex_t mtx_;
    pthread_cond_t cv_;
public:
    Lidar();
    bool init(const std::string &port, int baudrate);
    void setScan(std::vector<SensorPoint> points); // SDK 폴링을 통해 값 읽기
    void getScan();
    std::vector<SensorPoint> getData();        // fusion에 넘겨줄 SensorPoint

    void waitForData();
    void notifyDataReady();
};

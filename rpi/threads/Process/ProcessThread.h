#pragma once
#include "SensorWorker.h"
#include "Background.h"
#include "Grid.h"
#include "Tracker.h"
#include "UDPSender.h"
#include "Protocol.h"
#include "Utility.h"

#include <pthread.h>
#include <atomic>
#include <vector>

class ProcessThread {
private:
    Lidar *sensor_;                 // SDKThread가 채워주는 raw 데이터 소스
    BackgroundModel *background_;   // 이미 calibrate() 끝난 상태로 넘겨받음

    Grid grid_;
    Tracker tracker_;
    // 소유권째로 받음 (std::move로 받음)
    UDPSender udp_sender_;         

    std::atomic<bool> running_;
    pthread_t thread_id_;
    uint32_t frame_id_ = 0;

    static void* threadfunc(void* arg);
    void run();

    std::vector<SensorPoint> filterForeground(const std::vector<SensorPoint> &raw) const;
    void sendPointFrame(const std::vector<SensorPoint> &foreground, uint32_t frame_id);
    void sendObjectFrame(const std::vector<Track> &tracks, uint32_t frame_id);

public:
    ProcessThread(Lidar *sensor,
                  BackgroundModel *background,
                  int grid_max_range,
                  int grid_cell_num,
                  UDPSender udp_sender);
    ~ProcessThread();

    bool start();
    void stop();

    // 캘리브레이션 완료 직후 UI에 배경 한 번만 알려줄 때 사용
    void sendBackgroundFrame();
};
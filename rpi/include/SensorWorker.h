#ifndef SENSOR_WORKER_H
#define SENSOR_WORKER_H

#include <pthread.h>
#include <atomic>

// TODO: 실제 프로젝트의 KalmanFilter.h, Protocol.h 경로에 맞게 수정
#include "KalmanFilter.h"

/**
 * SensorWorker
 * -------------
 * 센서 1개(초음파 또는 LiDAR 채널 1개)를 담당하는 pthread를 RAII로 감싼 클래스.
 *
 * 설계 원칙:
 *  - 생성자에서는 아무것도 시작하지 않음 (start()를 명시적으로 호출해야 스레드 생성)
 *  - 소멸자에서 반드시 join() 호출 -> 좀비 스레드/리소스 누수 방지
 *  - 각 SensorWorker는 자신만의 입력/출력 데이터를 가짐 (mutex 불필요, task-parallel)
 */
class SensorWorker {
public:
    SensorWorker(int sensorId, double dt);
    ~SensorWorker();

    // 복사 금지 (pthread_t, 리소스를 가진 클래스는 복사 의미가 불명확함)
    SensorWorker(const SensorWorker&) = delete;
    SensorWorker& operator=(const SensorWorker&) = delete;

    // 이동은 허용 (필요 시 구현, 지금은 unique_ptr로 관리할 것이므로 생략 가능)

    void start();                  // pthread_create 호출
    void join();                   // pthread_join 호출 (중복 호출 안전하게 처리)

    void setRawInput(float distance);   // 최신 raw 측정값 갱신 (메인 스레드 -> 워커)
    float getFilteredOutput() const;    // 필터링된 결과 조회 (워커 -> 메인 스레드)

    void requestStop();            // 루프 종료 요청 (join 전에 호출)

private:
    static void* threadEntry(void* arg);  // pthread_create가 요구하는 C 스타일 함수 포인터
    void run();                            // 실제 워커 스레드 로직

    int sensorId_;
    pthread_t thread_;
    bool threadStarted_;

    KalmanFilter filter_;

    // 스레드 간 공유되는 최소한의 데이터
    // (지금은 std::atomic으로 단순화. 구조체 단위 공유가 필요해지면 mutex 고려)
    std::atomic<float> rawInput_;
    std::atomic<float> filteredOutput_;
    std::atomic<bool> stopRequested_;
};

#endif // SENSOR_WORKER_H

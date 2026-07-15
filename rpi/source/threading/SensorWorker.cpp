#include "SensorWorker.h"
#include <chrono>
#include <thread>

SensorWorker::SensorWorker(int sensorId, double dt)
    : sensorId_(sensorId),
      thread_(),
      threadStarted_(false),
      filter_(dt),
      rawInput_(0.0f),
      filteredOutput_(0.0f),
      stopRequested_(false)
{
}

SensorWorker::~SensorWorker() {
    // 소멸자에서 안전하게 정리: stop 요청 -> join
    // (start()가 호출된 적 없으면 threadStarted_가 false라 join 생략)
    requestStop();
    join();
}

void SensorWorker::start() {
    if (threadStarted_) {
        return;  // 이미 시작된 스레드를 중복 생성하지 않음
    }
    stopRequested_ = false;
    pthread_create(&thread_, nullptr, threadEntry, this);
    threadStarted_ = true;
}

void SensorWorker::join() {
    if (threadStarted_) {
        pthread_join(thread_, nullptr);
        threadStarted_ = false;
    }
}

void SensorWorker::requestStop() {
    stopRequested_ = true;
}

void SensorWorker::setRawInput(float distance) {
    rawInput_.store(distance, std::memory_order_relaxed);
}

float SensorWorker::getFilteredOutput() const {
    return filteredOutput_.load(std::memory_order_relaxed);
}

void* SensorWorker::threadEntry(void* arg) {
    static_cast<SensorWorker*>(arg)->run();
    return nullptr;
}

void SensorWorker::run() {
    // 실제 프로젝트에서는 이 주기를 메인 루프/센서 샘플링 주기에 맞춰 조정
    constexpr auto kLoopInterval = std::chrono::milliseconds(20); // 예: 50Hz

    while (!stopRequested_.load(std::memory_order_relaxed)) {
        float raw = rawInput_.load(std::memory_order_relaxed);

        filter_.predict();
        filter_.update(raw);

        float filtered = static_cast<float>(filter_.getState()(0)); // 상태벡터[0] = 거리
        filteredOutput_.store(filtered, std::memory_order_relaxed);

        std::this_thread::sleep_for(kLoopInterval);
    }
}

#pragma once
#include <vector>
#include "SensorWorker.h"
#include "Protocol.h"
#include "Utility.h"

class BackgroundModel {
private:
    static constexpr float MAX_RANGE_FALLBACK = 20;
    static constexpr float DEFAULT_STD_FALLBACK = 0.3f;

    // 반사로 인해 LIDAR가 거리값으로 0을 내뱉을때를 대비한 최소 유효 측정 거리
    static constexpr float MIN_VALID_RANGE = 0.05f;   // 0.05m
    static constexpr float MAX_VALID_RANGE = 12.0f;   // 12m (라이다가 최대 측정거리)
    
    // 몇 개의 각도 구간으로 나눌지 
    int num_bins_;                              
    
    // calibration 측정 시간 중 받는 단위 bin별의 거리 벡터
    std::vector<std::vector<float>> samples_;    
    // 각 bin의 거리 중앙값
    std::vector<float> background_;    
    // 각 bin의 표준편차 (foreground인지 판단용)          
    std::vector<float> std_;                    
    
    // foreground 판정 민감도 (2~3)
    float k_;                                    
    std::vector<uint8_t> valid_;   

    int AngleToBin(float angle_rad) const;
    float median(std::vector<float> values) const;
    float stddev(const std::vector<float>& values) const; 
public:
    BackgroundModel(int num_bins, float k);
    void calibrate(Lidar &lidar, uint64_t duration_ms);
    void computeBackground();
    bool isForeground(float angle, float range) const;
    const std::vector<float>& getBackground() const { return background_; }
    int getNumBins() const { return num_bins_; }
};
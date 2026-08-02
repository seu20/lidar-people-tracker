#pragma once
#include <vector>
#include "SensorWorker.h"
#include "Protocol.h"
#include "Utility.h"

class BackgroundModel {
private:
    static constexpr float MAX_RANGE_FALLBACK = 20;
    static constexpr float DEFAULT_STD_FALLBACK = 0.3f;

    // 유효 측정 범위. 라이다는 반사가 없으면 range = 0 을 돌려주는데,
    // 이걸 배경 통계에 넣으면 stddev가 폭발해서 그 방향이 검출 불능이 된다.
    static constexpr float MIN_VALID_RANGE = 0.05f;   // m
    static constexpr float MAX_VALID_RANGE = 12.0f;   // m (X4 계열 최대 측정 거리)
    

    int num_bins_;                              // 몇 개의 각도 구간으로 나눌지 (예: 720개, 0.5도 단위)
    
    std::vector<std::vector<float>> samples_;    // 캘리브레이션 중 bin별로 모은 range 샘플들 (계산 끝나면 비움)
    std::vector<float> background_;              // bin별 배경 거리 (median 값), 완성된 배경맵
    std::vector<float> std_;                     // bin별 표준편차 (노이즈 정도)
    
    float k_;                                    // foreground 판정 민감도 (표준편차 배수, 보통 2~3)
    std::vector<uint8_t> valid_;   // private에 추가

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
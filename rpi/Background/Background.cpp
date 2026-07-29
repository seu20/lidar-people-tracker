#include "Background.h"
#include <cmath>
#include <algorithm>

BackgroundModel::BackgroundModel(int num_bins, float k)
    : num_bins_(num_bins),
      k_(k),
      samples_(num_bins)
{
}

int BackgroundModel::AngleToBin(float angle_rad) const {
    int bin = static_cast<int>(rad_to_deg(angle_rad) * num_bins_ / 360.0f);
    if (bin < 0) bin = 0;
    if (bin >= num_bins_) bin = num_bins_ - 1;
    return bin;
}

float BackgroundModel::median(std::vector<float> values) const {  // 값 복사로 받음 (정렬해도 원본 안 바뀌게)
    if (values.empty()) return -1.0f;  // invalid 처리
    
    std::sort(values.begin(), values.end());
    
    size_t n = values.size();
    if (n % 2 == 1) {
        // 홀수개: 정확히 가운데 값
        return values[n / 2];
    } else {
        // 짝수개: 가운데 두 값의 평균
        return (values[n/2 - 1] + values[n/2]) / 2.0f;
    }
}

float BackgroundModel::stddev(const std::vector<float>& values) const {
    if (values.size() < 2) return 0.0f;
    
    float mean = 0.0f;
    for (float v : values) mean += v;
    mean /= values.size();
    
    float variance = 0.0f;
    for (float v : values) variance += (v - mean) * (v - mean);
    variance /= values.size();
    
    return std::sqrt(variance);
}

void BackgroundModel::calibrate(Lidar &lidar, uint64_t duration_ms)
{
    samples_.assign(static_cast<size_t>(num_bins_), {}); // 매번 초기화
    uint64_t start_time = now_ms();
    while (now_ms() - start_time <= duration_ms)
    {
        
        lidar.waitForData(); 

        std::vector<SensorPoint> sensorpoint = lidar.getData();  // 실제 데이터 받아옴 (기존에 빠져있었음)
        
        for (auto &[rad, dist] : sensorpoint)
        {
            if (dist <= MIN_VALID_RANGE || dist > MAX_VALID_RANGE) continue;
            int bin = AngleToBin(rad);
            samples_[bin].push_back(dist);   // ★ bin 위치에 "누적"만 함, samples_ 자체는 크기 안 바뀜
        }
    }
    computeBackground();
}


void BackgroundModel::computeBackground()
{
    static constexpr float MIN_STD = 0.02f;   // 라이다 실측 노이즈 하한 (2cm)
    
    background_.assign(num_bins_, MAX_RANGE_FALLBACK);  // fallback 기본값 먼저 채움
    std_.assign(num_bins_, DEFAULT_STD_FALLBACK);
    
    for (int i = 0; i < num_bins_; i++)
    {
        // if (samples_[i].empty()) continue;   // 데이터 없는 bin은 fallback 값 유지
        if (samples_[i].empty()) continue;
        background_[i] = median(samples_[i]);
        std_[i] = std::max(stddev(samples_[i]), MIN_STD);   // ★
    }
    
    samples_.assign(static_cast<size_t>(num_bins_), {}); // samples 초기화
}

bool BackgroundModel::isForeground(float angle, float range) const
{
    if (range <= MIN_VALID_RANGE || range > MAX_VALID_RANGE) return false;   // ★ 무효/범위 밖 컷
    int bin = AngleToBin(angle);
    float diff = background_[bin] - range;
    return (diff > k_ * std_[bin]);
}

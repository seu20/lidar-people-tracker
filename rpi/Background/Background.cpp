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

float BackgroundModel::median(std::vector<float> values) const {  
    if (values.empty()) return -1.0f;  // invalid 처리
    
    std::sort(values.begin(), values.end());
    
    size_t n = values.size();
    if (n % 2 == 1) {
        // 홀수개: 가운데 값
        return values[n / 2];
    } else {
        // 짝수개: 가운데 두 값의 평균
        return (values[n/2 - 1] + values[n/2]) / 2.0f;
    }
}

float BackgroundModel::stddev(const std::vector<float>& values) const {
    // 최소 값 개수 검사
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

        //데이터 받아옴 
        std::vector<SensorPoint> sensorpoint = lidar.getData(); 
        
        for (auto &[rad, dist] : sensorpoint)
        {
            if (dist <= MIN_VALID_RANGE || dist > MAX_VALID_RANGE) continue;
            int bin = AngleToBin(rad);
            // bin 위치에 누적함
            samples_[bin].push_back(dist);   
        }
    }
    computeBackground();
}


void BackgroundModel::computeBackground()
{
    // 라이다의 최소 오차 표준편차 (5cm)
    static constexpr float MIN_STD = 0.05f;   
    
    // 값이 안들어왔을 때를 대비해서 기본값으로 저장
    background_.assign(num_bins_, MAX_RANGE_FALLBACK); 
    std_.assign(num_bins_, DEFAULT_STD_FALLBACK);
    valid_.assign(num_bins_, 0);  
    
    for (int i = 0; i < num_bins_; i++)
    {
        // 샘플이 5개밖에 없는 bin은 불확실한 bin이기 때문에 건너뛰기
        if (samples_[i].size() < 5) continue;
        background_[i] = median(samples_[i]);
        std_[i] = std::max(stddev(samples_[i]), MIN_STD);   
        valid_[i] = 1;                              
    }
    // 메모리 절약을 위해 samples 초기화
    samples_.assign(static_cast<size_t>(num_bins_), {}); 
}

bool BackgroundModel::isForeground(float angle, float range) const
{
    // 무효/범위 밖 컷
    if (range <= MIN_VALID_RANGE || range > MAX_VALID_RANGE) return false;   
    int bin = AngleToBin(angle);
    if (!valid_[bin]) return false; 
    float diff = background_[bin] - range;
    return (diff > k_ * std_[bin]);
}

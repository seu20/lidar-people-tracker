#pragma once
#include <cstdint>
#pragma pack(push, 1)
// RPI -> PC 로 보내는 UDP 헤더 (초음파 센서 + 라이다 정보)
struct ScanFrame {
    uint32_t magic;
    uint32_t timestamp;
    uint16_t seq_num;
    uint8_t  ultrasonic_count;      // 초음파는 5개 고정
    float    ultrasonic_dist[5];     // 초음파 5개 거리값
    uint16_t lidar_point_count;      // 실제 채워진 LiDAR 포인트 개수 (가변)
    float    lidar_points[150][2];   // 최대 150개, 안 쓰는 부분은 그냥 남겨둠
};

// PC -> RPI 로 보내는 TCP ( 제어 명령: 시작 + 정지 )
struct ControlFrame {
    uint32_t magic;
    uint32_t timestamp;
    uint16_t seq_num;
};

// STM32 -> RPI 로 보내는 CAN ( 초음파 센서 5개의 거리값 )

#pragma pack(pop)
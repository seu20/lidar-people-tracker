#pragma once
#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

constexpr size_t MAX_POINTS = 1400;
constexpr size_t MAX_BINS = 720;
constexpr size_t MAX_OBJECTS = 16;

enum class TCPCmdType : uint8_t { START = 0x01, STOP  = 0x02 };
enum class MsgType : uint8_t { POINTS = 1, BACKGROUND = 2, OBJECTS = 3 };


// 센서 데이터 구조체
struct SensorPoint{
    float angle;
    float dist;
};

struct Point2D {
    float x;
    float y;
};

/***********  Point Header, 객체 헤더 ***********/
struct PointFrameHeader {
    uint32_t frameId;
    uint16_t pointCount;
};

struct ObjectFrameHeader {
    uint32_t frameId;
    uint16_t ObjectCount;
};
/******************************************** */

/*****************  Frame  ********************/
// 1. 원본/foreground 포인트 - 매 프레임, "지금 뭐가 보이는지" 시각화용
struct PointFrame {
    MsgType type;
    PointFrameHeader header;      // frameId, pointCount
    Point2D points[MAX_POINTS];
};

// 2. 배경 - 캘리브레이션 끝나고 딱 한 번 (또는 재보정할 때만)
struct BackgroundFrame {
    MsgType type;
    uint16_t numBins;
    float background[MAX_BINS]; // bin별 배경 거리
};

// 3. 추적된 객체 - 매 프레임, id/속도 포함
struct TrackedObject {
    int32_t id;
    float x, y;
    float vx, vy;
};

struct ObjectFrame {
    MsgType type;
    ObjectFrameHeader header;   // frameId, ObjectCount
    TrackedObject objects[MAX_OBJECTS];
};

/******************************************** */

// PC -> RPI 로 보내는 TCP ( 제어 명령: 시작 + 정지 )
struct TCPFrame {
    TCPCmdType cmd;
};


#pragma pack(pop)
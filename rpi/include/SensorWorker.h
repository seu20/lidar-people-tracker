#include <iostream>
// #include <thread>
// #include <atomic>
// #include <mutex>
// #include <condition_variable>

#pragma pack(1)
struct USData
{
    int US_id;
    float dist;
};

struct LidarData
{
    float dist;
    float angle;
};
struct pointxy {
    float x;
    float y;
};
#pragma pop


class Point {
private:
    float x;
    float y;
public:
    Point US_to_Point(const USData& us)
    {
        
    }
};

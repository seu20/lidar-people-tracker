#pragma once
#include "Protocol.h"
#include <pthread.h>
#include <vector>

struct ViewState {
    std::vector<Point2D> points;
    std::vector<TrackedObject> objects;
    std::vector<float> background;
    uint32_t lastPointFrameId = 0;
    uint32_t lastObjectFrameId = 0;
};

class SharedState {
private:
    mutable pthread_mutex_t mtx_;
    ViewState state_;

public:
    SharedState();
    ~SharedState();

    void updatePoints(std::vector<Point2D> points, uint32_t frameId);
    void updateObjects(std::vector<TrackedObject> objects, uint32_t frameId);
    void updateBackground(std::vector<float> background);

    ViewState getSnapshot() const;
};
#include "SharedState.h"

SharedState::SharedState()
{
    pthread_mutex_init(&mtx_, nullptr);
}

SharedState::~SharedState()
{
    pthread_mutex_destroy(&mtx_);
}

void SharedState::updatePoints(std::vector<Point2D> points, uint32_t frameId)
{
    pthread_mutex_lock(&mtx_);
    state_.points = std::move(points);
    state_.lastPointFrameId = frameId;
    pthread_mutex_unlock(&mtx_);
}

void SharedState::updateObjects(std::vector<TrackedObject> objects, uint32_t frameId)
{
    pthread_mutex_lock(&mtx_);
    state_.objects = std::move(objects);
    state_.lastObjectFrameId = frameId;
    pthread_mutex_unlock(&mtx_);
}

void SharedState::updateBackground(std::vector<float> background)
{
    pthread_mutex_lock(&mtx_);
    state_.background = std::move(background);
    pthread_mutex_unlock(&mtx_);
}

ViewState SharedState::getSnapshot() const
{
    pthread_mutex_lock(&mtx_);
    ViewState copy = state_;
    pthread_mutex_unlock(&mtx_);
    return copy;
}
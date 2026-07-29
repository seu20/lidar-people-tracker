#include "ProcessThread.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cstddef>

ProcessThread::ProcessThread(Lidar *sensor,
                              BackgroundModel *background,
                              int grid_max_range,
                              int grid_cell_num,
                              UDPSender udp_sender)
    : sensor_(sensor)
    , background_(background)
    , grid_(grid_max_range, grid_cell_num)
    , tracker_()
    , udp_sender_(std::move(udp_sender))
    , running_(false)
{
}

ProcessThread::~ProcessThread()
{
    if (running_) stop();
}

void* ProcessThread::threadfunc(void *arg)
{
    static_cast<ProcessThread*>(arg)->run();
    return nullptr;
}

bool ProcessThread::start()
{
    running_ = true;
    if (pthread_create(&thread_id_, nullptr, &ProcessThread::threadfunc, this) != 0)
    {
        std::cerr << "process thread not created!" << std::endl;
        running_ = false;
        return false;
    }
    return true;
}

void ProcessThread::stop()
{
    if (!running_) return;
    running_ = false;
    pthread_join(thread_id_, nullptr);
}

std::vector<SensorPoint> ProcessThread::filterForeground(const std::vector<SensorPoint> &raw) const
{
    std::vector<SensorPoint> foreground;
    foreground.reserve(raw.size());

    for (const auto &p : raw)
    {
        if (background_->isForeground(p.angle, p.dist))
        {
            foreground.push_back(p);
        }
    }
    return foreground;
}

void ProcessThread::run()
{
    while (running_)
    {
        sensor_->waitForData();
        if (!running_) break;

        std::vector<SensorPoint> raw = sensor_->getData();
        std::vector<SensorPoint> foreground = filterForeground(raw);

        grid_.Clear();
        grid_.InsertPoints(foreground);
        std::vector<Point2D> centroids = grid_.Cluster();

        uint64_t t = now_ms();
        tracker_.update(centroids, t);

        uint32_t frame_id = frame_id_++;
        sendPointFrame(foreground, frame_id);
        sendObjectFrame(tracker_.getTracks(), frame_id);
    }
}

void ProcessThread::sendPointFrame(const std::vector<SensorPoint> &foreground, uint32_t frame_id)
{
    PointFrame frame;
    frame.type = MsgType::POINTS;
    frame.header.frameId = frame_id;

    size_t count = std::min(foreground.size(), MAX_POINTS);
    for (size_t i = 0; i < count; ++i)
    {
        frame.points[i].x = foreground[i].dist * std::cos(foreground[i].angle);
        frame.points[i].y = foreground[i].dist * std::sin(foreground[i].angle);
    }
    frame.header.pointCount = static_cast<uint16_t>(count);

    // points[0] 시작 오프셋까지 + 실제 채운 개수만큼만 전송
    size_t send_size = offsetof(PointFrame, points) + count * sizeof(Point2D);
    udp_sender_.send(&frame, send_size);
}

void ProcessThread::sendObjectFrame(const std::vector<Track> &tracks, uint32_t frame_id)
{
    ObjectFrame frame;
    frame.type = MsgType::OBJECTS;
    frame.header.frameId = frame_id;

    size_t count = std::min(tracks.size(), MAX_OBJECTS);
    for (size_t i = 0; i < count; ++i)
    {
        Track t = tracks[i];   // getState()가 const가 아니라 복사해서 사용 (아래 참고)
        auto state = t.kf.getState();

        frame.objects[i].id = t.id;
        frame.objects[i].x  = static_cast<float>(state[0]);
        frame.objects[i].y  = static_cast<float>(state[1]);
        frame.objects[i].vx = static_cast<float>(state[2]);
        frame.objects[i].vy = static_cast<float>(state[3]);
    }
    frame.header.ObjectCount = static_cast<uint16_t>(count);

    size_t send_size = offsetof(ObjectFrame, objects) + count * sizeof(TrackedObject);
    udp_sender_.send(&frame, send_size);
}

void ProcessThread::sendBackgroundFrame()
{
    BackgroundFrame frame;
    frame.type = MsgType::BACKGROUND;

    const auto &background = background_->getBackground();   // getter 추가 필요 (저번 답변 참고)
    size_t count = std::min(background.size(), MAX_BINS);

    for (size_t i = 0; i < count; ++i)
    {
        frame.background[i] = background[i];
    }
    frame.numBins = static_cast<uint16_t>(count);

    size_t send_size = offsetof(BackgroundFrame, background) + count * sizeof(float);
    udp_sender_.send(&frame, send_size);
}
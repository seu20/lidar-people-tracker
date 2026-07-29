#pragma once
#include <vector>
#include "KalmanFilter.h"
#include "Utility.h"
#include "Protocol.h"

struct Track {
    int id;
    KalmanFilter kf;          // 이 track 전용 Kalman filter
    int missed_frames = 0;    // 몇 프레임 연속으로 매칭 안 됐는지 (삭제 판단용)
};

class Tracker {
private:
    std::vector<Track> tracks;
    int next_id = 0;
    float match_threshold;   // 이 거리 이내면 "같은 물체"로 판단
    int max_missed_frames;   // 이 프레임 수 넘게 안 보이면 track 삭제

public:
    Tracker();

    // 매 프레임 호출 ( Predict -> Associate -> Update )
    void update(const std::vector<Point2D>& centroids, uint64_t time_ms);

    // 전체 track-centroid 페어를 거리순으로 매칭
    void associate(const std::vector<Point2D>& centroids,
                    std::vector<std::pair<int,int>>& matched,
                    std::vector<int>& unmatched_centroids,
                    std::vector<int>& unmatched_tracks);

    // 새 track 생성
    void CreateNewTrack(const Point2D& centroid, uint64_t time_ms);

    std::vector<Track> getTracks() const;
    void setTracks(std::vector<Track>& tracks_);
};
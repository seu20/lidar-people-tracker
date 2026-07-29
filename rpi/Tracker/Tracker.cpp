#include "Tracker.h"
#include <algorithm>
#include <limits>

Tracker::Tracker()
    : next_id(0)
    , match_threshold(0.5f)      // 예시값, 네 센서 스케일에 맞게 조정 필요
    , max_missed_frames(5)        // 예시값, 5프레임 연속 안 보이면 삭제
{
}

void Tracker::update(const std::vector<Point2D> &centroids, uint64_t time_ms)
{
    std::vector<std::pair<int, int>> matched;   // { [track_idx, centroid_idx], ... }
    std::vector<int> unmatched_centroids;
    std::vector<int> unmatched_tracks;

    // 1. predict
    for (auto &track : tracks)
    {
        track.kf.predict(time_ms);
    }

    // 2. associate : 전역 거리순 greedy 매칭
    associate(centroids, matched, unmatched_centroids, unmatched_tracks);

    // 3. update: 매칭된 track은 측정값으로 갱신, missed_frames 리셋
    for (auto &[track_idx, centroid_idx] : matched)
    {
        float meas_x = centroids[centroid_idx].x;
        float meas_y = centroids[centroid_idx].y;
        tracks[track_idx].kf.update(meas_x, meas_y);
        tracks[track_idx].missed_frames = 0;
    }

    // 4. unmatched track: missed_frames 증가
    for (int idx : unmatched_tracks)
    {
        tracks[idx].missed_frames++;
    }

    // 5. 너무 오래 안 보인 track 삭제
    tracks.erase(
        std::remove_if(tracks.begin(), tracks.end(),
            [this](const Track &t) { return t.missed_frames > max_missed_frames; }),
        tracks.end()
    );

    // 6. unmatched centroid: 새 track 생성
    for (int idx : unmatched_centroids)
    {
        CreateNewTrack(centroids[idx], time_ms);
    }
}

void Tracker::associate(const std::vector<Point2D> &centroids,
                         std::vector<std::pair<int,int>> &matched,
                         std::vector<int> &unmatched_centroids,
                         std::vector<int> &unmatched_tracks)
{
    struct Candidate {
        int track_idx;
        int centroid_idx;
        float dist;
    };

    std::vector<Candidate> candidates;

    // 1. 모든 (track, centroid) 페어의 거리 계산 (threshold 이내만)
    for (int track_idx = 0; track_idx < (int)tracks.size(); ++track_idx)
    {
        float state_x = tracks[track_idx].kf.getState()[0];
        float state_y = tracks[track_idx].kf.getState()[1];

        for (int centroid_idx = 0; centroid_idx < (int)centroids.size(); ++centroid_idx)
        {
            float dx = state_x - centroids[centroid_idx].x;
            float dy = state_y - centroids[centroid_idx].y;
            float dist = dx * dx + dy * dy;   // 제곱거리 (sqrt 생략, 비교만 하니까 상관없음)

            if (dist < match_threshold * match_threshold)
            {
                candidates.push_back({track_idx, centroid_idx, dist});
            }
        }
    }

    // 2. 거리 오름차순 정렬 → 가장 가까운 페어부터 확정
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) { return a.dist < b.dist; });

    std::vector<bool> track_used(tracks.size(), false);
    std::vector<bool> centroid_used(centroids.size(), false);

    // 3. 짧은 거리부터 순서대로 확정, 이미 쓰인 track/centroid는 skip
    for (const auto &c : candidates)
    {
        if (track_used[c.track_idx] || centroid_used[c.centroid_idx])
            continue;

        matched.push_back({c.track_idx, c.centroid_idx});
        track_used[c.track_idx] = true;
        centroid_used[c.centroid_idx] = true;
    }

    // 4. 매칭 안 된 것들 정리
    for (int i = 0; i < (int)tracks.size(); ++i)
        if (!track_used[i]) unmatched_tracks.push_back(i);

    for (int j = 0; j < (int)centroids.size(); ++j)
        if (!centroid_used[j]) unmatched_centroids.push_back(j);

}

void Tracker::CreateNewTrack(const Point2D &centroid, uint64_t time_ms)
{
    Track new_track;
    new_track.id = next_id++;
    new_track.kf.init(centroid.x, centroid.y, time_ms);
    tracks.push_back(new_track);
}

std::vector<Track> Tracker::getTracks() const
{
    return tracks;
}

void Tracker::setTracks(std::vector<Track> &tracks_)
{
    tracks = tracks_;
}
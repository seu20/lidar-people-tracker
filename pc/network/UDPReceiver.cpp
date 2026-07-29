#include "UDPReceiver.h"
#include <cstring>
#include <cstddef>   // [수정 3] offsetof
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <algorithm>

UDPReceiver::UDPReceiver(int port, SharedState *state)
    : state_(state), running_(false)
{
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1) {
        perror("UDP socket creation failed");
        exit(1);
    }

    // stop() 호출 시 recvfrom()이 계속 블로킹돼있지 않도록 타임아웃 설정
    // (SDKThread에서 겪었던 "블로킹된 스레드가 안 깨어나는" 문제 방지)
    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // 200ms마다 한 번씩 running_ 체크하러 깨어남
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("UDP bind failed");
        exit(1);
    }
}

UDPReceiver::~UDPReceiver()
{
    if (running_) stop();
    close(sockfd);
}

void* UDPReceiver::threadfunc(void *arg)
{
    static_cast<UDPReceiver*>(arg)->run();
    return nullptr;
}

bool UDPReceiver::start()
{
    running_ = true;
    if (pthread_create(&thread_id_, nullptr, &UDPReceiver::threadfunc, this) != 0) {
        std::cerr << "UDP receive thread not created!" << std::endl;
        running_ = false;
        return false;
    }
    return true;
}

void UDPReceiver::stop()
{
    if (!running_) return;
    running_ = false;
    pthread_join(thread_id_, nullptr);
}

void UDPReceiver::run()
{
    static uint8_t buf[65536];   // UDP 최대 크기 기준으로 넉넉히

    while (running_)
    {
        ssize_t n = recvfrom(sockfd, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) continue;   // 타임아웃(-1, EAGAIN)이거나 빈 패킷 - 그냥 다시 루프 돌면서 running_ 체크

        handlePacket(buf, static_cast<size_t>(n));
    }
}

// [수정 3] 기존에는 헤더 길이만 확인하고 payload는 검사하지 않았다.
//          pointCount가 1400인데 패킷이 잘려서 오면 버퍼 밖 11KB를 읽게 된다.
//          송신부(sendPointFrame 등)의 send_size 계산과 정확히 대칭이 되도록 검사한다.
void UDPReceiver::handlePacket(const uint8_t *data, size_t len)
{
    if (len < sizeof(MsgType)) return;

    MsgType type = *reinterpret_cast<const MsgType*>(data);

    switch (type)
    {
        case MsgType::POINTS:
        {
            constexpr size_t HDR = offsetof(PointFrame, points);
            if (len < HDR) return;

            const PointFrame *frame = reinterpret_cast<const PointFrame*>(data);
            uint16_t count = std::min<uint16_t>(frame->header.pointCount, MAX_POINTS);
            if (len < HDR + count * sizeof(Point2D)) return;   // ★ payload 길이 확인

            std::vector<Point2D> points(frame->points, frame->points + count);
            state_->updatePoints(std::move(points), frame->header.frameId);
            break;
        }
        case MsgType::OBJECTS:
        {
            constexpr size_t HDR = offsetof(ObjectFrame, objects);
            if (len < HDR) return;

            const ObjectFrame *frame = reinterpret_cast<const ObjectFrame*>(data);
            uint16_t count = std::min<uint16_t>(frame->header.ObjectCount, MAX_OBJECTS);
            if (len < HDR + count * sizeof(TrackedObject)) return;   // ★

            std::vector<TrackedObject> objects(frame->objects, frame->objects + count);
            state_->updateObjects(std::move(objects), frame->header.frameId);
            break;
        }
        case MsgType::BACKGROUND:
        {
            constexpr size_t HDR = offsetof(BackgroundFrame, background);
            if (len < HDR) return;

            const BackgroundFrame *frame = reinterpret_cast<const BackgroundFrame*>(data);
            uint16_t count = std::min<uint16_t>(frame->numBins, MAX_BINS);
            if (len < HDR + count * sizeof(float)) return;   // ★

            std::vector<float> bg(frame->background, frame->background + count);
            state_->updateBackground(std::move(bg));
            break;
        }
        default:
            std::cerr << "Unknown MsgType: " << static_cast<int>(type) << std::endl;
            break;
    }
}
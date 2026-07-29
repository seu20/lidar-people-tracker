#include "TCPSender.h"
#include <cstring>
#include <cerrno>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

TCPSender::TCPSender(const std::string &ip, int port)
{
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        perror("TCP socket creation failed");
        exit(1);
    }

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
        perror("Invalid RPi address");
        exit(1);
    }

    // RPi가 TCPReceiver::WaitConnection()에서 accept() 대기 중이어야 연결됨
    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("Connect to RPi failed");
        exit(1);
    }

    std::cout << "Connected to RPi at " << ip << ":" << port << std::endl;
}

TCPSender::~TCPSender()
{
    if (sockfd >= 0) close(sockfd);
}

// [수정 1] 세 가지를 한 번에 처리한다.
//   (a) MSG_NOSIGNAL - 상대가 연결을 끊은 뒤 send()가 SIGPIPE로 프로세스를 죽이는 것을 막음
//   (b) 반환값 확인   - 지금까지는 실패해도 아무도 몰랐음
//   (c) 부분 전송 처리 - TCP는 스트림이라 요청한 만큼 다 못 보낼 수 있음
bool TCPSender::sendFrame(TCPCmdType cmd, const char *what)
{
    TCPFrame frame{ cmd };
    const char *p = reinterpret_cast<const char*>(&frame);
    size_t remaining = sizeof(frame);

    while (remaining > 0)
    {
        ssize_t n = send(sockfd, p, remaining, MSG_NOSIGNAL);
        if (n > 0) {
            p         += n;
            remaining -= static_cast<size_t>(n);
            continue;
        }
        if (n == -1 && errno == EINTR) continue;   // 시그널로 중단된 경우 재시도

        std::cerr << what << " failed: "
                  << (n == -1 ? strerror(errno) : "connection closed") << std::endl;
        return false;
    }
    return true;
}

bool TCPSender::sendStart() { return sendFrame(TCPCmdType::START, "sendStart"); }
bool TCPSender::sendStop()  { return sendFrame(TCPCmdType::STOP,  "sendStop");  }
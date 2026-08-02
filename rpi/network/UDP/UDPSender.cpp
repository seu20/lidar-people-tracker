#include "UDPSender.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cerrno>

/*
struct sockaddr_in {
    sa_family_t    sin_family;   // 주소 체계 (IPv4인지 등)
    in_port_t      sin_port;      // 포트 번호
    struct in_addr sin_addr;      // IP 주소
    char           sin_zero[8];   // 패딩 (안 씀, 무시해도 됨)
};
*/

UDPSender::UDPSender(const std::string& ip, int port)
{
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1)
    {
        perror("UDP Socket not created!");
        exit(1);
    }
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;     // 주소 체계
    dest_addr.sin_port = htons(port);   // 컴퓨터가 쓰는 리틀 엔디언을 네트워크가 사용하는 빅엔디언으로 변환 (지정된 포트)
    if (inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr) <= 0)    //c_str은 c style 문자열로 변환해줌, c++ 문자열은 + 기능있는데 이런 거 다 뺌 (지정된 ip)
    {
        perror("Invalid address/ Address not supported");
        exit(1);
    }
}

UDPSender::~UDPSender()
{
    if (sockfd>= 0) close(sockfd); 
}

// 이동 생성자
// std::move 사용할때 socket fd 바꿔주는 역할
UDPSender::UDPSender(UDPSender&& other) noexcept
    : sockfd(other.sockfd), dest_addr(other.dest_addr)
{
    other.sockfd = -1;   // 원본은 무효화 - 소멸자에서 close(-1) 되도록
}

// 이동 대입 연산자
UDPSender& UDPSender::operator=(UDPSender&& other) noexcept
{
    if (this != &other) {          // 자기 자신에 대입하는 경우 방지
        if (sockfd >= 0) {
            close(sockfd);          // 기존에 갖고 있던 소켓은 정리
        }
        sockfd = other.sockfd;
        dest_addr = other.dest_addr;
        other.sockfd = -1;          // 원본 무효화
    }
    return *this;
}
//복사 생성&대입 불가 
UDPSender(const UDPSender&) = delete;
UDPSender& operator=(const UDPSender&) = delete;

void UDPSender::send(const void *data, size_t len)
{
    ssize_t sent = sendto(sockfd, data, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (sent == -1)
    {
        std::cerr << "sendto failed: " << strerror(errno) << std::endl;
    }
}

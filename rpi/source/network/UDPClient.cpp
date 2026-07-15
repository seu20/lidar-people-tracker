#include "UDPSender.h"
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

/*
struct sockaddr_in {
    sa_family_t    sin_family;   // 주소 체계 (IPv4인지 등)
    in_port_t      sin_port;      // 포트 번호
    struct in_addr sin_addr;      // IP 주소
    char           sin_zero[8];   // 패딩 (안 씀, 무시해도 됨)
};
*/

UdpSender::UdpSender(const std::string& ip, int port)
{
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1)
    {
        perror("UDP Socket not created!");
        exit(1);
    }
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;     // 주소 체계
    dest_addr.sin_port = htons(port);   // 컴퓨터가 쓰는 리틀 엔디언을 네트워크가 사용하는 빅엔디언으로 변환
    inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr);    //c_str은 c style 문자열로 변환해줌, c++ 문자열은 + 기능있는데 이런 거 다 뺌
}

bool UdpSender::send(const ScanFrame &frame)
{
    ssize_t sent = sendto(sockfd, &frame, sizeof(ScanFrame), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (sent == -1)
    {
        std::cerr << "sendto failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

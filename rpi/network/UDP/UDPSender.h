#pragma once
#include <string>
#include "Protocol.h"
#include <netinet/in.h>

class UDPSender {
private:
    int sockfd;
    struct sockaddr_in dest_addr;

public:
    UDPSender(const std::string& ip, int port);
    ~UDPSender();

    //복사 금지
    UDPSender(const UDPSender&) = delete;
    UDPSender& operator=(const UDPSender&) = delete;

    // 이동은 허용 (FusionThread 생성자에서 std::move로 넘기고 계셨으니 필요함)
    UDPSender(UDPSender&& other) noexcept;
    UDPSender& operator=(UDPSender&& other) noexcept;

    void send(const void *data, size_t len);
};
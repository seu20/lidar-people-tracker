#pragma once
#include "Protocol.h"
#include <string>

class TCPSender {
private:
    int sockfd;

    // [수정 1] START/STOP 공통 전송 경로. 성공하면 true.
    bool sendFrame(TCPCmdType cmd, const char *what);

public:
    TCPSender(const std::string &ip, int port);
    ~TCPSender();

    TCPSender(const TCPSender&) = delete;
    TCPSender& operator=(const TCPSender&) = delete;

    // [수정 1] 반환값으로 전송 성공 여부를 알린다 (RPi 연결 끊김 감지용)
    bool sendStart();
    bool sendStop();
};
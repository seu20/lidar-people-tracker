#include "TCPReceiver.h"
#include <iostream>
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

TCPReceiver::TCPReceiver(int port)
{
    listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if ( listen_fd == -1 )
    {
        perror("socket creation failed");
        exit(-1);
    }
    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(listen_addr));   // 초기화
    listen_addr.sin_family = AF_INET;   // IPv4 
    listen_addr.sin_port = htons(port); // 포트 설정
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);   //모든 IP에서 접근 허용

    if (bind(listen_fd, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) == -1) // master socket 커널과 연결
    {
        perror("bind failed");
        exit(-1);
    }
    if (listen(listen_fd, 2) == -1)   // 연결요청 대기(backlog 2: 두개까지만 받음)
    {
        perror("listen failed");
        exit(-1);
    }
}

TCPReceiver::~TCPReceiver()
{
    close(listen_fd);
}

bool TCPReceiver::WaitConnection()
{
    data_socket  = accept(listen_fd, nullptr, nullptr); //연결요청
    if (data_socket == -1)
    {
        perror("Data Socket not accpeted!");
        return false;
    }
    return true;
}

TCPFrame TCPReceiver::receiveCommand()
{
    struct TCPFrame control_msg;
    ssize_t received = recv(data_socket, &control_msg, sizeof(control_msg), 0);
    if (received <= 0)
    {
        std::cerr << "Connection Lost!" << std::endl;
        exit(-1);
    }
    return control_msg;
}


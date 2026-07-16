#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <string>

class TCPSender {
private:
    int sockfd;
    struct sockaddr_in dest_addr;
public:
    TCPSender(const std::string& ip, int port);
    ~TCPSender();
    void sendData(const int& frame);
    void receiveData(int& frame);
};

#pragma once
#include <sys/socket.h>
#include <netinet/in.h>

class UDPServer {
private:
    int sockfd;
public:
    UDPServer(int udp_port);
    ~UDPServer();
    void receiveData(int &frame);
};
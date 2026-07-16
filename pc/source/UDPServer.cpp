#include "UDPServer.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

UDPServer::UDPServer(int udp_port)
{
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1)
    {
        perror("Socket not created");
        exit(1);
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(udp_port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        perror("Binding Failed");
        exit(1);
    }

}

UDPServer::~UDPServer()
{
    close(sockfd);
}

void UDPServer::receiveData(int &frame)
{
    struct sockaddr_in udp_sender_addr;
    socklen_t sender_len = sizeof(udp_sender_addr);
    ssize_t received = recvfrom(sockfd, &frame, sizeof(frame), 0, (struct sockaddr *)&udp_sender_addr, &sender_len);
    if (received <= 0)
    {
        perror("Receive Failed");
        exit(1);
    }
}

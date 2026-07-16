#include "TCPSender.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cerrno>

TCPSender::TCPSender(const std::string& rpi_ip, int port)
{
    memset(&dest_addr, 0, sizeof(dest_addr));
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); 
    if (sockfd < 0)
    {
        perror("socket creation failed");
        exit(1);
    }
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, rpi_ip.c_str(), &dest_addr.sin_addr) <= 0)
    {
        perror("invalid address");
        exit(1);
    }
    if (connect(sockfd, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) == -1)
    {
        perror("Connection Failed");
        exit(1);
    }
}

TCPSender::~TCPSender() {
    close(sockfd);
}

void TCPSender::sendData(const int &frame) {
    ssize_t sent = send(sockfd, &frame, sizeof(frame), 0);
    if (sent <= 0)
    {
        perror("Sending Failed");
        exit(1);
    }
}
void TCPSender::receiveData(int &frame) {
    ssize_t received = recv(sockfd, &frame, sizeof(frame), 0);
    if (received <= 0)
    {
        perror("Receiving Failed");
        exit(1);
    }
}
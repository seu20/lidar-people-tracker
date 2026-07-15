#include "Protocol.h"
#include <string>
#include <netinet/in.h>

class UdpSender {
private:
    int sockfd;
    struct sockaddr_in dest_addr;

public:
    UdpSender(const std::string& ip, int port);
    ~UdpSender(close(sockfd););
    bool send(const ScanFrame& frame);
};
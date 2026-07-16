#include <iostream>
#include "TCPSender.h"
#include "UDPServer.h"

#define UDP_PORT   3000
#define TCP_PORT   2000

int main(int argc, char** argv)
{
    int frame;
    UDPServer udpserver(UDP_PORT);
    std::cout << "Socket prepared" << std::endl;
    while(true)
    {
        udpserver.receiveData(frame);
        std::cout << frame << std::endl;
    }
}
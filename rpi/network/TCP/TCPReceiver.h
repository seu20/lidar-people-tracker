#include "Protocol.h"
#include <string>
#include <netinet/in.h>
#include <unistd.h>
    
class TCPReceiver {
private:
    int listen_fd;
    int client_fd;
    int data_socket;
public:
    TCPReceiver(int port);
    ~TCPReceiver();
    bool WaitConnection();
    TCPFrame receiveCommand();
};
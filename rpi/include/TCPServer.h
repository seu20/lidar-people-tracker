#include "Protocol.h"
#include <string>
#include <netinet/in.h>
    
class TCPServer {
private:
    int listen_fd;
    int client_fd;
    int data_socket;
public:
    TCPServer(int port);
    ~TCPServer { close(listen_fd) };
    bool WaitConnection();
    ControlFrame receiveCommand();
};
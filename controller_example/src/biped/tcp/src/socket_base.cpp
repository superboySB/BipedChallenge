#include "socket_base.hpp"


using namespace std;
using namespace nlohmann;


SocketBase::SocketBase(const std::string &host_ip, const unsigned int port)
{
    //setup a socket and connection tools 
    struct hostent* host = gethostbyname( host_ip.c_str() ); 

    bzero( (char*)&addr_, sizeof(addr_) ); 

    addr_.sin_family = AF_INET; 
    addr_.sin_addr.s_addr = inet_addr(
        inet_ntoa( *(struct in_addr*)*host->h_addr_list )
        );
    addr_.sin_port = htons(port);

    host_ip_ = host_ip;
    port_ = port;
}


string SocketBase::recvMsg_(const int conn_sock)
{
    char msg_size_bin[4] = {0};
    char msg_buf[BUF_SIZE] = {0};

    recvAll_(conn_sock, msg_size_bin, 4);
    int msg_size = bin2int(msg_size_bin);

    assert(msg_size <= BUF_SIZE);

    recvAll_(conn_sock, msg_buf, msg_size);

    return string(msg_buf);
}
    

int SocketBase::recvAll_(const int conn_sock, char *buf, size_t length)
{
    size_t size_read = 0;

    while(length > 0)
    {
        int recv_size = recv(conn_sock, buf + size_read, length, 0);

        if(recv_size == -1)
            return -1;
        
        size_read += recv_size;
        length -= recv_size;
    }

    return size_read;
}


int SocketBase::sendMsg_(const int conn_sock, const std::string &msg)
{
    string dsize_bin = int2binstr(msg.length());

    if(sendAll_(conn_sock, (dsize_bin + msg).c_str(), 4 + msg.length()) == -1)
    {
        cerr << "failed to send response" << endl;
        return -1;
    }
    
    return 0;
}


int SocketBase::sendAll_(const int conn_sock, const char* buf, size_t length)
{
    int byte_sent = 0;

    while(length > 0)
    {
        int tmp = send(conn_sock, buf + byte_sent, length, 0);
        
        if(tmp == -1)
            return -1;
        
        byte_sent += tmp;
        length -= tmp;
    }

    return byte_sent;
}
#ifndef _SIMPLE_TCP_SERVER_H
#define _SIMPLE_TCP_SERVER_H


#include "socket_base.hpp"

#define BUF_SIZE    10000


class TcpServer: public SocketBase
{
public:
    /**
     * Constructor
     * 
     * @param host host IP
     * @param port port number
     */
    TcpServer(const std::string &host_ip, const unsigned int port);

public:
    /**
     * Launch the server, create socket and binding
     */
    int launch();
    void close_server();


protected:
    virtual std::string processMsg_(const char *msg);

private:
    /**
     * Create socket binding
     * 
     * @param addr socket address information
     * @return socket descriptor
     */
    int createSocket_(const sockaddr_in &addr);

    /**
     * Request handler
     * 
     * @param conn_sock socket descriptor of connnected socket
     * @return status code
     */
    int connectionHandler_(const int conn_sock);

private:
    int sock_;
    int conn_sock;
};

#endif

#ifndef _SIMPLE_SOCKET_BASE_H
#define _SIMPLE_SOCKET_BASE_H

#include <stdio.h> 
#include <stdlib.h>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h> 
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <assert.h>

#include "data_converter.hpp"


#define BUF_SIZE    10000


class SocketBase
{
public:
    /**
     * Constructor
     * 
     * @param host host IP
     * @param port port number
     */
    SocketBase(const std::string &host_ip, const unsigned int port);


protected:
    /**
     * Receive message
     * 
     * @param conn_sock socket descriptor of connnected socket
     * @return message in binary string
     */
    std::string recvMsg_(const int conn_sock);


    /**
     * Receive all content given a package length
     * 
     * @param conn_sock socket descriptor of connnected socket
     * @param buf message buffer, i.e., where message stored
     * @param length length of message in bytes
     * @return bytes received
     */
    int recvAll_(const int conn_sock, char* buf, size_t length);


    /**
     * Send all content given a package length
     * 
     * @param conn_sock socket descriptor of connnected socket
     * @param msg message in string
     * @return bytes sent
     */
    int sendMsg_(const int conn_sock, const std::string &msg);


    /**
     * Send all content given a package length
     * 
     * @param conn_sock socket descriptor of connnected socket
     * @param buf message to be sent
     * @param length length of message in bytes
     * @return bytes sent
     */
    int sendAll_(const int conn_sock, const char* buf, size_t length);


protected:
    sockaddr_in addr_;
    std::string host_ip_;
    int port_;
};

#endif
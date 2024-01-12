#include "tcp_server.hpp"


using namespace std;
using namespace nlohmann;


TcpServer::TcpServer(const std::string &host_ip, const unsigned int port):
    SocketBase(host_ip, port)
{
    sock_ = -1;
}


void TcpServer::close_server()
{
    shutdown(conn_sock, SHUT_RDWR);
    shutdown(sock_, SHUT_RDWR);
    close(conn_sock);
    close(sock_);
    sock_=-1;
    std::cout << "close server\n";
}

int TcpServer::launch()
{
    sock_ = createSocket_(addr_);

    cout << "[INFO] Server is launched at: " << host_ip_ << ":" << port_ << endl;

    while(sock_!=-1)
    {
        //receive a request from client using accept
        //we need a new address to connect with the client
        sockaddr_in conn_addr;
        socklen_t conn_addr_size = sizeof(conn_addr);
        //accept, create a new socket descriptor to 
        //handle the new connection with client
        conn_sock = accept(sock_, (sockaddr *)&conn_addr, &conn_addr_size);
        
        if(conn_sock < 0)
        {
            cerr << "[ERROR] Fail to accept the connection from client" << endl;
        }

        thread th(&TcpServer::connectionHandler_, this, conn_sock);
        // have the thread run independently
        th.detach();
    }
}


int TcpServer::connectionHandler_(const int conn_sock)
{
    cout << "[INFO] New connection" << endl;

    while(sock_!=-1)
    {
        string msg = recvMsg_(conn_sock);
        
        if(msg.length() == 0)
        {
            break;
        }

        string resp_bin = processMsg_( msg.c_str() );
        
        // send response back
        sendMsg_(conn_sock, resp_bin);
    }

    close(conn_sock);
    cout << "[INFO] Connection closed" << endl;
    return 0;
}


std::string TcpServer::processMsg_(const char *msg)
{
    json jmsg = bin2json(msg);

    json resp;
    resp["content"] = "Hello from TCP Server";
    
    return json2binstr(resp);
}


int TcpServer::createSocket_(const sockaddr_in &addr)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    if(sock < 0)
    {
        cerr << "[ERROR] Socket creation failed!" << endl;
        exit(1);
    }

    if(bind(sock, (struct sockaddr*) &addr, sizeof(addr)) < 0)
    {
        cerr << "[ERROR] Server binding failed!" << endl;
        exit(1);
    }

    //listen for up to 5 requests at a time
    listen(sock, 5);

    return sock;
}

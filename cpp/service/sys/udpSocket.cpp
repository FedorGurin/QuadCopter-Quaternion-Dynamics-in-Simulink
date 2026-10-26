#include "udpSocket.h"
#include <string.h>
#include <iostream>

#if defined(_WIN32)
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef int socklen_t;
    #define CLOSE_SOCKET closesocket
    static bool isTimeout(){return WSAGetLastError() == WSAETIMEDOUT;}
    //! инициализация Winsock один раз на программу
    static void initNet()
    {
        static bool done = false;
        if(!done)
        {
            WSADATA wsa;
            WSAStartup(MAKEWORD(2, 2), &wsa);
            done = true;
        }
    }
#else
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <errno.h>
    #define CLOSE_SOCKET ::close
    static bool isTimeout(){return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;}
    static void initNet(){}
#endif

static const intptr_t INVALID_SOCK = -1;

UdpSocket::UdpSocket()
{
    sock    = INVALID_SOCK;
    numPort = 0;
    hasFrom = false;
    memset(lastFrom, 0, sizeof(lastFrom));
    static_assert(sizeof(sockaddr_in) <= sizeof(lastFrom), "sockaddr_in size");
}
UdpSocket::~UdpSocket()
{
    close();
}
bool UdpSocket::init(uint16_t port, std::string ip)
{
    initNet();
    numPort = port;
    ipBind  = ip;
    sock    = (intptr_t)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock == INVALID_SOCK)
    {
        std::cout << "UdpSocket::init(): can't create socket" << std::endl;
        return false;
    }
    int opt = 1;
    setsockopt((int)sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));
    return true;
}
bool UdpSocket::bindTo()
{
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(numPort);
    addr.sin_addr.s_addr = ipBind.empty() ? htonl(INADDR_ANY) : inet_addr(ipBind.c_str());
    if(::bind((int)sock, (sockaddr *)&addr, sizeof(addr)) != 0)
    {
        std::cout << "UdpSocket::bindTo(): can't bind port " << numPort << std::endl;
        return false;
    }
    return true;
}
bool UdpSocket::setTimeout(uint32_t msec)
{
#if defined(_WIN32)
    DWORD tv = msec;
#else
    timeval tv;
    tv.tv_sec  = msec / 1000;
    tv.tv_usec = (msec % 1000) * 1000;
#endif
    return setsockopt((int)sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) == 0;
}
int UdpSocket::receiveFrom(uint8_t *buf, uint32_t size)
{
    sockaddr_in from;
    socklen_t   len = sizeof(from);
    int bytes = (int)recvfrom((int)sock, (char *)buf, (int)size, 0, (sockaddr *)&from, &len);
    if(bytes < 0)
        return isTimeout() ? 0 : -1;
    memcpy(lastFrom, &from, sizeof(from));
    hasFrom = true;
    return bytes;
}
int UdpSocket::sendTo(const uint8_t *buf, uint32_t size, uint16_t port)
{
    if(hasFrom == false)
        return -1;
    sockaddr_in to;
    memcpy(&to, lastFrom, sizeof(to));
    to.sin_port = htons(port);
    return (int)sendto((int)sock, (const char *)buf, (int)size, 0, (sockaddr *)&to, sizeof(to));
}
int UdpSocket::sendTo(const uint8_t *buf, uint32_t size, uint16_t port, const std::string &ip)
{
    sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family      = AF_INET;
    to.sin_port        = htons(port);
    to.sin_addr.s_addr = inet_addr(ip.c_str());
    return (int)sendto((int)sock, (const char *)buf, (int)size, 0, (sockaddr *)&to, sizeof(to));
}
void UdpSocket::close()
{
    if(sock != INVALID_SOCK)
    {
        CLOSE_SOCKET((int)sock);
        sock = INVALID_SOCK;
    }
}

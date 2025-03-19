/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/10
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <cerrno>
#include <cstring>
#include <utility>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <dobot_v4_bringup/tcp_socket.h>

/**
 ***********************************************************************************************************************
    定义了TcpClient类，用于处理与机械臂控制器的TCP连接
 ***********************************************************************************************************************
 */

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)           //构造函数
{
}

TcpClient::~TcpClient()                                                                                                        //析构函数
{
    close();
}

void TcpClient::close()                                                                                                        //关闭TCP连接
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

void TcpClient::connect()                                                                                                      //建立TCP连接
{
    if (fd_ < 0)                                                                         //检查文件描述符 fd_ 是否有效。如果无效，则创建一个新的 TCP 套接字。
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};                                                               //使用 sockaddr_in 结构配置服务器地址和端口信息。

    memset(&addr, 0, sizeof(addr));
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);

    if (::connect(fd_, (sockaddr*)&addr, sizeof(addr)) < 0)                              //调用 ::connect 函数与服务器建立连接，如果连接失败，则抛出 TcpClientException 异常。               
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(errno));
    is_connected_ = true;                                                                //如果连接成功，则将 is_connected_ 设为 true，并输出成功连接的信息。

    ROS_INFO("%s : connect successfully", toString().c_str());
}

void TcpClient::disConnect()                                                                                                   //断开TCP连接
{
    if (is_connected_)
    {
        fd_ = -1;
        is_connected_ = false;
        ::close(fd_);
    }
}

bool TcpClient::isConnect() const                                                                                              //检查当前是否已连接
{
    return is_connected_;
}

void TcpClient::tcpSend(const void* buf, uint32_t len)                                                                         //发送数据到服务器。buf：指向待发送数据的缓冲区。 len：待发送数据长度
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    ROS_INFO("send : %s", (const char*)buf);

    const auto* tmp = (const uint8_t*)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, MSG_NOSIGNAL);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void* buf, uint32_t len, uint32_t& has_read, uint32_t timeout)                                         //从服务器接收数据，可设定超时时间
{
    uint8_t* tmp = (uint8_t*)buf;    // NOLINT(modernize-use-auto)

    fd_set read_fds;
    timeval tv = { 0, 0 };

    has_read = 0;
    while(len)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        int err = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        else if (err == 0)
        {
            return false;
        }

        err = (int)::read(fd_, tmp, len);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        else if (err == 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }
        len -= err;
        tmp += (err - 1);

        if(tmp[0] == ';'){ 
            has_read += err;
            return true;
        }

        tmp++;
        has_read += err;
    }

    return true;
}

std::string TcpClient::toString()                                                                                             //返回连接的IP和端口信息的字符串表示，格式是ip：port，例如 192.168.0.1：9090
{
    return ip_ + ":" + std::to_string(port_);
}

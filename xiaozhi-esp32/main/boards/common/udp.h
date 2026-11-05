#ifndef _UDP_H_
#define _UDP_H_

#include <string>
#include <functional>
#include <sys/socket.h>

class Udp {
public:
    Udp();
    virtual ~Udp();

    bool Bind(int port);
    void Close();
    
    // 发送数据到指定 IP 和端口
    bool Send(const std::string& ip, int port, const std::string& data);
    
    // 设置接收回调
    void SetReceiveCallback(std::function<void(const std::string&, const std::string&, int)> callback);

private:
    int socket_fd_;
    std::function<void(const std::string&, const std::string&, int)> callback_;

    void ReceiveTask();
};

#endif // _UDP_H_
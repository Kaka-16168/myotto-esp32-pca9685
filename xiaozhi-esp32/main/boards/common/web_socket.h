#ifndef _WEB_SOCKET_H_
#define _WEB_SOCKET_H_

#include <string>

class WebSocket {
public:
    WebSocket() = default;
    virtual ~WebSocket() = default;
    virtual bool Connect(const std::string& url) { return false; }
    virtual void Disconnect() {}
    virtual bool IsConnected() { return false; }
};

#endif

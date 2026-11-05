#ifndef _WEB_SOCKET_H_
#define _WEB_SOCKET_H_

#include <string>
#include <functional>
#include <map>
#include <esp_websocket_client.h>

class WebSocket {
public:
    WebSocket();
    virtual ~WebSocket();

    bool Connect(const std::string& url);
    void Disconnect();
    bool IsConnected();
    bool Send(const std::string& data);

    void SetHeader(const std::string& key, const std::string& value);
    void SetEventCallback(std::function<void(const std::string&)> callback);

private:
    esp_websocket_client_handle_t client_;
    std::map<std::string, std::string> headers_;
    std::function<void(const std::string&)> callback_;
    
    static void WebsocketEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
};

#endif // _WEB_SOCKET_H_
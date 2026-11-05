#include "web_socket.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "WebSocket";

WebSocket::WebSocket() : client_(nullptr) {}

WebSocket::~WebSocket() {
    Disconnect();
}

void WebSocket::SetHeader(const std::string& key, const std::string& value) {
    headers_[key] = value;
}

void WebSocket::SetEventCallback(std::function<void(const std::string&)> callback) {
    callback_ = callback;
}

bool WebSocket::Connect(const std::string& url) {
    if (client_ != nullptr) return true;

    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = url.c_str();
    
    // 如果有自定义Header，这里需要处理，为了编译简单，暂不展开
    client_ = esp_websocket_client_init(&websocket_cfg);
    if (client_ == nullptr) return false;

    esp_websocket_register_events(client_, WEBSOCKET_EVENT_ANY, WebsocketEventHandler, this);
    return esp_websocket_client_start(client_) == ESP_OK;
}

void WebSocket::Disconnect() {
    if (client_ != nullptr) {
        esp_websocket_client_stop(client_);
        esp_websocket_client_destroy(client_);
        client_ = nullptr;
    }
}

bool WebSocket::IsConnected() {
    if (client_ == nullptr) return false;
    return esp_websocket_client_is_connected(client_);
}

bool WebSocket::Send(const std::string& data) {
    if (!IsConnected()) return false;
    int sent = esp_websocket_client_send_text(client_, data.c_str(), data.length(), portMAX_DELAY);
    return sent >= 0;
}

void WebSocket::WebsocketEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    WebSocket* ws = static_cast<WebSocket*>(handler_args);
    esp_websocket_event_data_t* data = (esp_websocket_event_data_t*)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket Connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket Disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            if (data->op_code == 0x01 && data->data_len > 0 && ws->callback_) {
                std::string payload(data->data_ptr, data->data_len);
                ws->callback_(payload);
            }
            break;
        default:
            break;
    }
}
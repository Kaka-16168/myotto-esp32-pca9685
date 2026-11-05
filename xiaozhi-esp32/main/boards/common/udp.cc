#include "udp.h"
#include <esp_log.h>
#include <lwip/sockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "Udp";

Udp::Udp() : socket_fd_(-1) {}

Udp::~Udp() {
    Close();
}

void Udp::SetReceiveCallback(std::function<void(const std::string&, const std::string&, int)> callback) {
    callback_ = callback;
}

bool Udp::Bind(int port) {
    Close();
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_fd_ < 0) return false;

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_port = htons(port);

    if (bind(socket_fd_, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        Close();
        return false;
    }

    // 开启接收任务
    xTaskCreate([](void* arg){
        static_cast<Udp*>(arg)->ReceiveTask();
    }, "udp_recv", 4096, this, 5, nullptr);

    return true;
}

void Udp::Close() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool Udp::Send(const std::string& ip, int port, const std::string& data) {
    if (socket_fd_ < 0) return false;

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr);

    int err = sendto(socket_fd_, data.c_str(), data.length(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    return err >= 0;
}

void Udp::ReceiveTask() {
    char rx_buffer[1024];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (socket_fd_ >= 0) {
        int len = recvfrom(socket_fd_, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            rx_buffer[len] = 0;
            if (callback_) {
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &source_addr.sin_addr, ip_str, sizeof(ip_str));
                std::string data(rx_buffer, len);
                callback_(ip_str, data, ntohs(source_addr.sin_port));
            }
        } else if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
    }
    vTaskDelete(NULL);
}
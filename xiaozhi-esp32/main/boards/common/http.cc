#include "http.h"
#include <esp_log.h>

static const char* TAG = "Http";

Http::Http() {}
Http::~Http() {}

void Http::SetHeader(const std::string& key, const std::string& value) {
    headers_[key] = value;
}

bool Http::Get(const std::string& url, std::string& response) {
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.method = HTTP_METHOD_GET;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return false;
    }

    for (const auto& header : headers_) {
        esp_http_client_set_header(client, header.first.c_str(), header.second.c_str());
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
        if (status == 200) {
            if (content_length > 0) {
                response.resize(content_length);
                esp_http_client_read(client, &response[0], content_length);
            } else {
                char buffer[1024];
                int read_len = 0;
                while ((read_len = esp_http_client_read(client, buffer, sizeof(buffer))) > 0) {
                    response.append(buffer, read_len);
                }
            }
            esp_http_client_cleanup(client);
            return true;
        } else {
            ESP_LOGE(TAG, "HTTP GET failed, status=%d", status);
        }
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return false;
}

bool Http::Post(const std::string& url, const std::string& data, std::string& response) {
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) return false;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    for (const auto& header : headers_) {
        esp_http_client_set_header(client, header.first.c_str(), header.second.c_str());
    }
    esp_http_client_set_post_field(client, data.c_str(), data.length());

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status >= 200 && status < 300) {
            char buffer[1024];
            int read_len = 0;
            while ((read_len = esp_http_client_read(client, buffer, sizeof(buffer))) > 0) {
                response.append(buffer, read_len);
            }
            esp_http_client_cleanup(client);
            return true;
        }
    }

    esp_http_client_cleanup(client);
    return false;
}
#ifndef HTTP_H
#define HTTP_H

#include <string>
#include <map>
#include <esp_http_client.h>

class Http {
public:
    Http();
    ~Http();

    // 发送 GET 请求
    bool Get(const std::string& url, std::string& response);

    // 发送 POST 请求
    bool Post(const std::string& url, const std::string& data, std::string& response);

    // 设置请求头
    void SetHeader(const std::string& key, const std::string& value);

private:
    std::map<std::string, std::string> headers_;
};

#endif // HTTP_H
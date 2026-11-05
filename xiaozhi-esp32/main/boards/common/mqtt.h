#ifndef _MQTT_H_
#define _MQTT_H_

#include <string>
#include <functional>
#include <mqtt_client.h>

class Mqtt {
public:
    Mqtt();
    virtual ~Mqtt();

    bool Connect(const std::string& uri);
    void Disconnect();
    bool IsConnected();
    bool Publish(const std::string& topic, const std::string& data);

    void SetEventCallback(std::function<void(const std::string&, const std::string&)> callback);

private:
    esp_mqtt_client_handle_t client_;
    std::function<void(const std::string&, const std::string&)> callback_;

    static void MqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
};

#endif // _MQTT_H_
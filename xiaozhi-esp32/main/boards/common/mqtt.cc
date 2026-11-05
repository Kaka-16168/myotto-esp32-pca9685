#include "mqtt.h"
#include <esp_log.h>

static const char* TAG = "Mqtt";

Mqtt::Mqtt() : client_(nullptr) {}

Mqtt::~Mqtt() {
    Disconnect();
}

void Mqtt::SetEventCallback(std::function<void(const std::string&, const std::string&)> callback) {
    callback_ = callback;
}

bool Mqtt::Connect(const std::string& uri) {
    if (client_ != nullptr) return true;

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = uri.c_str();

    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (client_ == nullptr) return false;

    esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, MqttEventHandler, this);
    return esp_mqtt_client_start(client_) == ESP_OK;
}

void Mqtt::Disconnect() {
    if (client_ != nullptr) {
        esp_mqtt_client_stop(client_);
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
    }
}

bool Mqtt::IsConnected() {
    return client_ != nullptr;
}

bool Mqtt::Publish(const std::string& topic, const std::string& data) {
    if (client_ == nullptr) return false;
    int msg_id = esp_mqtt_client_publish(client_, topic.c_str(), data.c_str(), data.length(), 1, 0);
    return msg_id >= 0;
}

void Mqtt::MqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    Mqtt* mqtt = static_cast<Mqtt*>(handler_args);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_DATA:
            if (event->topic_len > 0 && event->data_len > 0 && mqtt->callback_) {
                std::string topic(event->topic, event->topic_len);
                std::string data(event->data, event->data_len);
                mqtt->callback_(topic, data);
            }
            break;
        default:
            break;
    }
}
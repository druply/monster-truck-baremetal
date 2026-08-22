// production_mqtt_client.hpp
#include <mqtt/async_client.h>
#include <mqtt/callback.h>
//#include <spdlog/spdlog.h>
#include <functional>
#include <map>

class ProductionMQTTClient : public virtual mqtt::callback {
private:
    std::unique_ptr<mqtt::async_client> client_;
    mqtt::connect_options connOpts_;
    std::map<std::string, std::function<void(const std::string&)>> handlers_;
    
public:
    ProductionMQTTClient(const std::string& broker, const std::string& clientId)
        : client_(std::make_unique<mqtt::async_client>(broker, clientId)) {
        
        connOpts_.set_keep_alive_interval(30);
        connOpts_.set_clean_session(false);  // Persistent session
        connOpts_.set_automatic_reconnect(true);
        connOpts_.set_connect_timeout(10);
        
        client_->set_callback(*this);
    }
    
    bool connect() {
        try {
            // spdlog::info("Connecting to Mosquitto broker at {}", client_->get_server_uri());
            client_->connect(connOpts_)->wait();
            // spdlog::info("Connected successfully");
            return true;
        } catch (const mqtt::exception& e) {
            // spdlog::error("Connection failed: {}", e.what());
            return false;
        }
    }
    
    void subscribe(const std::string& topic, int qos = 1) {
        client_->subscribe(topic, qos)->wait();
        // spdlog::info("Subscribed to topic: {}", topic);
    }
    
    void publish(const std::string& topic, const std::string& payload, int qos = 1) {
        try {
            auto msg = mqtt::make_message(topic, payload);
            msg->set_qos(qos);
            msg->set_retained(false);
            client_->publish(msg)->wait();
            // spdlog::debug("Published to {}: {}", topic, payload);
        } catch (const mqtt::exception& e) {
            // spdlog::error("Publish failed: {}", e.what());
        }
    }
    
    void registerHandler(const std::string& topic, 
                        std::function<void(const std::string&)> handler) {
        handlers_[topic] = handler;
    }
    
    // Callback methods
    void connected(const std::string& cause) override {
        // spdlog::info("MQTT Connected: {}", cause);
    }
    
    void connection_lost(const std::string& cause) override {
        // spdlog::warn("MQTT Connection lost: {}", cause);
        // Auto-reconnect is handled by Paho
    }
    
    void message_arrived(mqtt::const_message_ptr msg) override {
        // spdlog::info("Message arrived on topic: {}", msg->get_topic());
        
        auto it = handlers_.find(msg->get_topic());
        if (it != handlers_.end()) {
            it->second(msg->to_string());
        }
    }
    
    void delivery_complete(mqtt::delivery_token_ptr token) override {
        // spdlog::debug("Delivery complete for token: {}", token->get_message_id());
    }
};
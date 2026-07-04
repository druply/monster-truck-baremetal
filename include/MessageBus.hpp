#pragma once

#include "imessagebus.hpp"
#include <boost/asio.hpp>
#include <vector>
#include <unordered_map>

class EmbeddedMessageBus final : public IMessageBus {
private:
    boost::asio::io_context& m_ioc;
    boost::asio::io_context::strand m_bus_strand;

    using GenericCallback = std::function<void(const std::any&)>;
    std::unordered_map<Topic, std::vector<GenericCallback>> m_registry;

protected:
    // Implement the hidden, type-erased registration function
    void do_subscribe(Topic topic, 
                      void* target_executor, 
                      std::function<void(const std::any&)> type_erased_callback) override {
        
        // Safely cast the anonymous executor pointer back to a Boost Strand
        auto* target_strand = static_cast<boost::asio::io_context::strand*>(target_executor);

        boost::asio::post(m_bus_strand, [this, topic, target_strand, callback = std::move(type_erased_callback)]() mutable {
            
            auto generic_handler = [target_strand, callback = std::move(callback)](const std::any& any_msg) {
                // Dispatch directly to the consumer's protective strand
                boost::asio::post(*target_strand, [callback, any_msg]() {
                    callback(any_msg);
                });
            };

            m_registry[topic].push_back(std::move(generic_handler));
        });
    }

    // Implement the hidden, type-erased publish function
    void do_publish(Topic topic, const std::any& type_erased_message) noexcept override {
        boost::asio::post(m_bus_strand, [this, topic, type_erased_message]() {
            auto it = m_registry.find(topic);
            if (it != m_registry.end()) {
                for (const auto& dispatch_to_strand : it->second) {
                    dispatch_to_strand(type_erased_message);
                }
            }
        });
    }

public:
    explicit EmbeddedMessageBus(boost::asio::io_context& ioc) 
        : m_ioc(ioc), m_bus_strand(ioc) {}
        
    ~EmbeddedMessageBus() noexcept override = default;
};

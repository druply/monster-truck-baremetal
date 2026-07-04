#pragma once

#include <memory>
#include <functional>
#include <any>

// Cheap numeric representation of your system topics
enum class Topic : uint16_t {
    EncoderData,
    MotorAction
};

// Pure abstract interface defining execution behavior
class IMessageBus {
protected:
    // Core type-erased virtual routing functions hidden from public use
    virtual void do_subscribe(Topic topic, 
                              void* target_executor, 
                              std::function<void(const std::any&)> type_erased_callback) = 0;
                              
    virtual void do_publish(Topic topic, const std::any& type_erased_message) noexcept = 0;

public:
    virtual ~IMessageBus() noexcept = default;

    // Public Type-Safe Template API (Statically maps to the virtual engine)
    template <typename PayloadType, typename ExecutorType>
    void subscribe(Topic topic, 
                   ExecutorType& target_executor, 
                   std::function<void(std::shared_ptr<const PayloadType>)> callback) {
        
        // Type-erase the template callback into a standard std::any layout
        auto type_erased_callback = [callback](const std::any& any_msg) {
            auto type_safe_ptr = std::any_cast<std::shared_ptr<const PayloadType>>(any_msg);
            callback(type_safe_ptr);
        };

        // Pass down using a raw pointer handle to mask executor details
        do_subscribe(topic, static_cast<void*>(&target_executor), std::move(type_erased_callback));
    }

    template <typename PayloadType>
    void publish(Topic topic, std::shared_ptr<const PayloadType> message) noexcept {
        // Wrap the payload ptr in a std::any container and send it through the interface
        std::any wrapped_message = std::move(message);
        do_publish(topic, wrapped_message);
    }
};

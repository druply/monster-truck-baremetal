// ActionRegistry.hpp
#pragma once
#include <unordered_set>
#include <string>
#include <mutex>

class ActionRegistry {
public:
    static ActionRegistry& getInstance() {
        static ActionRegistry instance;
        return instance;
    }
    void registerAction(const std::string& action) {
        std::lock_guard<std::mutex> lock(mtx);
        registry.insert(action);
    }
    bool exists(const std::string& action) {
        std::lock_guard<std::mutex> lock(mtx);
        return registry.count(action) > 0;
    }
private:
    std::unordered_set<std::string> registry;
    std::mutex mtx;
};

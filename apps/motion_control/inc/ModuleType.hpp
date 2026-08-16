#pragma once
#include <memory>
class IMessageBroker;

class ModuleType{

    public:
		~ModuleType() = default;
    	virtual void init(void) noexcept = 0;
		virtual void run(void) noexcept = 0;
		virtual void deInit(void) noexcept = 0;
};
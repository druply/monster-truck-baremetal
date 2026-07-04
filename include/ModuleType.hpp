#pragma once
#include <memory>
class IMessageBroker;

class ModuleType{

    public:
		~ModuleType() = default;
    	virtual void init(void) = 0;
		virtual void run(void) = 0;
		virtual void deInit(void) = 0;
};
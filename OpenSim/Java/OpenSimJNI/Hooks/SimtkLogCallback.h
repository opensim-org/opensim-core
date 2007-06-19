#ifndef _SimtkLogCallback_h_
#define _SimtkLogCallback_h_

#include <OpenSim/Common/LogCallback.h>
#include <OpenSim/Common/LogManager.h>
#include <stdio.h>

namespace OpenSim {

class SimtkLogCallback : public LogCallback
{
public:
	virtual void log(const std::string &str) { }

	void addToLogManager() {
		LogManager::getInstance()->getOutBuffer()->addLogCallback(this);
		LogManager::getInstance()->getErrBuffer()->addLogCallback(this);
	}

	void removeFromLogManager() {
		LogManager::getInstance()->getOutBuffer()->removeLogCallback(this);
		LogManager::getInstance()->getErrBuffer()->removeLogCallback(this);
	}

};

}

#endif

#ifndef _LogCallback_h_
#define _LogCallback_h_

#include "osimCommonDLL.h"
#include <string>

namespace OpenSim {

class OSIMCOMMON_API LogCallback
{
public:
	virtual void log(const std::string &str) = 0;
};

}

#endif

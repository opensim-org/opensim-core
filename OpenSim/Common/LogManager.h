#ifndef _LogManager_h_
#define _LogManager_h_

#include "osimCommonDLL.h"
#include "Array.h"
#include "LogCallback.h"
#include <iostream>
#include <sstream>

namespace OpenSim {

class OSIMCOMMON_API StreamLogCallback : public LogCallback
{
private:
	std::ostream *_out;
	bool _ownsStream;
	
public:
	StreamLogCallback(const std::string &aFilename);
	StreamLogCallback(std::ostream *aOut, bool aOwnsStream = true);
	~StreamLogCallback();
	void log(const std::string &aStr);
};

class OSIMCOMMON_API LogBuffer : public std::stringbuf
{
public:
	LogBuffer();
	bool addLogCallback(LogCallback *aLogCallback);
	bool removeLogCallback(LogCallback *aLogCallback);

private:
	Array<LogCallback*> _logCallbacks;

	int sync();
};

class OSIMCOMMON_API LogManager
{
public:
	// Expose these members so users can manipulate output formats by calling functions on LogManager::out/err
	static LogBuffer out;
	static LogBuffer err;

	// LogManager's cout and cerr act as the normal standard output and error (i.e. they write to the terminal)
	static std::ostream cout;
	static std::ostream cerr;

	LogManager();
	~LogManager();

	static LogManager *getInstance();

	LogBuffer *getOutBuffer();
	LogBuffer *getErrBuffer();
};

}

#endif

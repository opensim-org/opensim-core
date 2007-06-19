#include "LogManager.h"
#include <fstream>

using namespace OpenSim;

// Initialize static members
LogBuffer LogManager::out;
LogBuffer LogManager::err;
std::ostream LogManager::cout(std::cout.rdbuf()); // This cout writes to the actual standard out
std::ostream LogManager::cerr(std::cerr.rdbuf()); // This cerr writes to the actual standard error

// Initialize a static log manager to force the constructor to be called
LogManager logManager;

//=============================================================================
// FileLogCallback
//=============================================================================
StreamLogCallback::StreamLogCallback(const std::string &filename)
	: _out(new std::ofstream(filename.c_str())), _ownsStream(true)
{}

StreamLogCallback::StreamLogCallback(std::ostream *aOut, bool aOwnsStream)
	: _out(aOut), _ownsStream(aOwnsStream)
{}

StreamLogCallback::~StreamLogCallback()
{
	if(_ownsStream) delete _out;
}

void StreamLogCallback::log(const std::string &str)
{
	*_out << str << std::flush;
}

//=============================================================================
// LogBuffer
//=============================================================================
LogBuffer::LogBuffer()
{
}

// Assumes caller owns this output stream (will never be deleted)
bool LogBuffer::
addLogCallback(LogCallback *aLogCallback)
{
	if(_logCallbacks.findIndex(aLogCallback) >= 0) return false;
	_logCallbacks.append(aLogCallback); 
	return true;
}

bool LogBuffer::
removeLogCallback(LogCallback *aLogCallback)
{
	int index = _logCallbacks.findIndex(aLogCallback);
	if(index < 0) return false;
	_logCallbacks.remove(index); 
	return true;
}

int LogBuffer::
sync()
{
	// Pass current string to all log callbacks
	for(int i=0; i<_logCallbacks.getSize(); i++) _logCallbacks[i]->log(str());
	// Reset current buffer contents
	str("");
	return std::stringbuf::sync();
}

//=============================================================================
// LogManager
//=============================================================================
LogManager::LogManager()
{
	// Seems to be causing crashes in the GUI... maybe a multithreading issue.
#if 1
	// Change the underlying streambuf for the standard cout/cerr to our custom buffers
	std::cout.rdbuf(&out);
	std::cerr.rdbuf(&err);

	// Example setup: redirect output to both the terminal and an output file
	out.addLogCallback(new StreamLogCallback(&cout,false));
	out.addLogCallback(new StreamLogCallback("out.log"));
	err.addLogCallback(new StreamLogCallback(&cerr,false));
	err.addLogCallback(new StreamLogCallback("err.log"));
#endif
}

LogManager::~LogManager()
{
	std::cout << std::flush;
	std::cerr << std::flush;
}

LogManager *LogManager::getInstance()
{
	return &logManager;
}

LogBuffer *LogManager::getOutBuffer()
{
	return &out;
}

LogBuffer *LogManager::getErrBuffer()
{
	return &err;
}

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
// LogBuffer
//=============================================================================
LogBuffer::LogBuffer() : 
	_outputStream(0),
	_secondaryOutputStream(0)
{
}

// Assumes caller owns this output stream (will never be deleted)
void LogBuffer::
setOutputStream(std::ostream *aOutputStream)
{
	_outputStream = aOutputStream;
}

// Assumes caller owns this output stream (will never be deleted)
void LogBuffer::
setSecondaryOutputStream(std::ostream *aSecondaryOutputStream)
{
	_secondaryOutputStream = aSecondaryOutputStream;
}

int LogBuffer::
sync()
{
	// Write to up to two output streams
	if (_outputStream) (*_outputStream) << str() << std::flush;
	if (_secondaryOutputStream) (*_secondaryOutputStream) << str() << std::flush;
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
	out.setOutputStream(&cout);
	out.setSecondaryOutputStream(new std::ofstream("out.log"));
	err.setOutputStream(&cerr);
	err.setSecondaryOutputStream(new std::ofstream("err.log"));
#endif
}

LogManager::~LogManager()
{
	std::cout << std::flush;
	std::cerr << std::flush;
}

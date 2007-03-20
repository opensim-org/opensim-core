#ifndef _LogManager_h_
#define _LogManager_h_

#include "osimCommonDLL.h"
#include <iostream>
#include <sstream>

namespace OpenSim {

class OSIMCOMMON_API LogBuffer : public std::stringbuf
{
    public:
        LogBuffer();
        void setOutputStream(std::ostream *aOutputStream);
        void setSecondaryOutputStream(std::ostream *aSecondaryOutputStream);

    private:
		// Currently supports writing to up to two streams (e.g. terminal and file), but could
		// use an array to support an arbitrary number of streams.
        std::ostream *_outputStream;
        std::ostream *_secondaryOutputStream;

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
};

}

#endif

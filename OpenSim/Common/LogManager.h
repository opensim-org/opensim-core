#ifndef _LogManager_h_
#define _LogManager_h_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

// DebugUtilities.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2006, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/
#include "DebugUtilities.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <stdexcept>

namespace OpenSim {
namespace DebugUtilities {

void Fatal_Error(const char *msg, const char *function, const char *file, unsigned int line)
{
	std::ostringstream string_stream;
	string_stream << "Fatal Error: " << msg << " (function = " << function << ", file = " << file << ", line = " << line << ")";
	std::cerr << string_stream.str() << std::endl;
	throw std::runtime_error(string_stream.str());
	assert(false);
	exit(1);
}

/**
 * Basically a utility that lets me take a bash script that has lines such as
 * export VAR=value
 * and insert these environment variables into this executable's environment.
 * Mostly for testing/debugging applications.
 */
void AddEnvironmentVariablesFromFile(const std::string &aFileName)
{
	if(aFileName.empty()) return;
	std::ifstream input(aFileName.c_str());
	std::string line;
	// Take any line that starts with "export" and set the environment variable that follows
	while (getline(input,line)) {
		if(line.find("export") != std::string::npos) {
			std::string env=line.substr(7);
			std::cout << "Setting environment '" << env << "'" << std::endl;
#ifdef WIN32
			_putenv(env.c_str());
#else
			putenv(const_cast<char*>(env.c_str()));
#endif
		}
	}
}

}
}

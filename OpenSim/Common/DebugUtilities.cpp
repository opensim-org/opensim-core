// DebugUtilities.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2006, Stanford University. All rights reserved. 
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

#ifndef _DebugUtilities_h_
#define _DebugUtilities_h_
// DebugUtilities.h
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
#include "osimCommonDLL.h"
#include <string>

#ifdef WIN32
#define OPENSIM_NORETURN(declaration) __declspec(noreturn) declaration
#else
#define OPENSIM_NORETURN(declaration) declaration __attribute__ ((noreturn))
#endif

#define OPENSIM_ERROR_IF_NOT_OVERRIDDEN() \
	OpenSim::DebugUtilities::Fatal_Error("Function is not overridden", __FUNCTION__,__FILE__,__LINE__);

#define OPENSIM_FUNCTION_NOT_IMPLEMENTED() \
	OpenSim::DebugUtilities::Fatal_Error("Function is not (fully) implemented", __FUNCTION__,__FILE__,__LINE__);

namespace OpenSim {
namespace DebugUtilities {

OPENSIM_NORETURN(OSIMCOMMON_API void Fatal_Error(const char *msg, const char *function, const char *file, unsigned int line));

OSIMCOMMON_API void AddEnvironmentVariablesFromFile(const std::string &aFileName);

}
}

#endif

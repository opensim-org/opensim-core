#ifndef _LoadOpenSimLibrary_h_
#define _LoadOpenSimLibrary_h_
// LoadOpenSimLibrary.h
// author: Frank C. Anderson, Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#include <string>
#include "osimCommonDLL.h"

#ifndef _WIN32
#define OPENSIM_PORTABLE_HMODULE void *
#define OPENSIM_PORTABLE_HINSTANCE void *
#define WINAPI
#include <dlfcn.h>
#define GetProcAddress(handle, proc) dlsym(handle, proc)
#else
#define OPENSIM_PORTABLE_HMODULE HMODULE
#define OPENSIM_PORTABLE_HINSTANCE HINSTANCE
#endif

namespace OpenSim
{

OSIMCOMMON_API OPENSIM_PORTABLE_HMODULE WINAPI LoadOpenSimLibrary(const std::string &lpLibFileName, bool verbose);
OSIMCOMMON_API void LoadOpenSimLibrary(const std::string &aLibraryName);
OSIMCOMMON_API void LoadOpenSimLibraries(int argc,char **argv);

}

#endif // __LoadOpenSimLibrary_h__
